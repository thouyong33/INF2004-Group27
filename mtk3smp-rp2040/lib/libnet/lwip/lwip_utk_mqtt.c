
#include "lwip_utk_mqtt.h"
#include "mqtt_config.h"
#include "telemetry_codec.h"
#include "buddy1_hooks.h"

#include "lwip/apps/mqtt.h"
#include "lwip/dhcp.h"
#include "lwip/dns.h"
#include "lwip/err.h"
#include "lwip/ip_addr.h"
#include "lwip/ip4_addr.h"
#include "lwip/sys.h"


/* Console logging function provided by the project. */
extern int tm_printf(const unsigned char *format, ...);

/* Topics added at integration (31 Oct); defaults so an older local
 * mqtt_config.h still builds */
#ifndef MQTT_TOPIC_EVENT
#define MQTT_TOPIC_EVENT  "inf2004/group27/event"
#endif
#ifndef MQTT_TOPIC_ONLINE
#define MQTT_TOPIC_ONLINE "inf2004/group27/online"
#endif
#ifndef MQTT_TOPIC_SYS
#define MQTT_TOPIC_SYS    "inf2004/group27/sys"
#endif

/* Events sent per poll at most (the output buffer is small) */
#define EVENTS_PER_POLL    4U
/* Timing figures for the evidence log, this often */
#define SYS_INTERVAL_MS    1000U
/* Broker gone this long after a connection: FAULT_WIFI_LOSS */
#define LINK_LOSS_MS       3000U

static uint32_t last_sys_ms;
static uint8_t was_connected;
static uint8_t link_lost;
static uint32_t down_since_ms;

/* MQTT client and broker address */
static mqtt_client_t *mqtt_client;
static ip_addr_t broker_address;

static mqtt_status_t mqtt_status = {
    .state = MQTT_STATE_WAIT_NETWORK,
    .dns_result = ERR_INPROGRESS
};

static uint32_t last_dns_attempt_ms;
static uint32_t last_connect_attempt_ms;

/* Periodic telemetry timing */
#define TELEMETRY_INTERVAL_MS 500U

static uint32_t last_telemetry_ms;
static uint8_t telemetry_due_immediately = 1;
static uint8_t heartbeat_counter = 0U;

/*
 * Provisional compact telemetry format.
 *
 * Byte 0: state (bits 0-2), speed (bits 3-7)
 * Byte 1: remaining speed bits (bits 0-3),
 *         distance (bits 4-7)
 * Byte 2: remaining distance bits (bits 0-3)
 *
 * Total meaningful bits: 20
 */
static void pack_telemetry(
    uint8_t payload[3],
    uint8_t state,
    uint16_t speed_mm_s,
    uint8_t distance_cm)
{
    state &= 0x07U;
    speed_mm_s &= 0x01FFU;

    payload[0] = (uint8_t)(
        state | ((speed_mm_s & 0x1FU) << 3)
    );

    payload[1] = (uint8_t)(
        ((speed_mm_s >> 5) & 0x0FU) |
        ((distance_cm & 0x0FU) << 4)
    );

    payload[2] = (uint8_t)(
        (distance_cm >> 4) & 0x0FU
    );
}



/*
 * Binary MQTT command protocol (provisional):
 * 0x00 = START
 * 0x01 = STOP
 *
 * FINISH (0x02) remains reserved until confirmed.
 * 0x10-0x15 = start one test run: l o b a r, and p (print the
 * continuous run's log) - added at integration (31 Oct).
 *
 * A valid command is passed to buddy1_command(), which feeds the
 * course test as a keystroke (START = c, the continuous course;
 * STOP = s).
 */
#define MQTT_COMMAND_START 0x00U
#define MQTT_COMMAND_STOP  0x01U
#define MQTT_COMMAND_RUN_FIRST 0x10U
#define MQTT_COMMAND_RUN_LAST  0x15U

static int command_known(uint8_t command)
{
    return (command == MQTT_COMMAND_START) ||
           (command == MQTT_COMMAND_STOP) ||
           ((command >= MQTT_COMMAND_RUN_FIRST) &&
            (command <= MQTT_COMMAND_RUN_LAST));
}

static uint8_t received_command;
static uint32_t received_command_length;
static uint8_t command_received;
static uint8_t command_invalid;

/* Called when an MQTT message arrives on a subscribed topic. */
static void mqtt_incoming_publish(
    void *arg,
    const char *topic,
    u32_t total_length)
{
    (void)arg;
    (void)topic;

    received_command = 0U;
    received_command_length = 0U;
    command_received = 0U;

    /* Only accept a one-byte binary command. */
    command_invalid = (total_length != 1U);
}

/* Called with the incoming MQTT message payload. */
static void mqtt_incoming_data(
    void *arg,
    const u8_t *data,
    u16_t length,
    u8_t flags)
{
    (void)arg;

    /*
     * Preserve the first byte if valid, and count all
     * incoming bytes, including fragmented payloads.
     */
    if (length > 0U) {
        if (data == NULL) {
            command_invalid = 1U;
        } else if (received_command_length == 0U) {
            received_command = data[0];
        }

        received_command_length += length;
    }

    if (flags & MQTT_DATA_FLAG_LAST) {
        if (!command_invalid &&
            received_command_length == 1U &&
            command_known(received_command)) {
            command_received = 1U;
            buddy1_command(received_command);
        } else {
            command_invalid = 1U;
            command_received = 0U;
        }
    }
}

/* Called when the subscription request finishes. */
static void mqtt_subscribe_callback(
    void *arg,
    err_t result)
{
    (void)arg;
    (void)result;
}


/* MQTT connection settings */
/* Last will (31 Oct): the broker publishes "0" on the online topic
 * if the car drops off without disconnecting; keep-alive 10 s so it
 * is noticed within ~15 s. */
static const struct mqtt_connect_client_info_t client_info = {
    .client_id = MQTT_CLIENT_ID,
    .keep_alive = 10,
    .will_topic = MQTT_TOPIC_ONLINE,
    .will_msg = "0",
    .will_msg_len = 1,
    .will_qos = 0,
    .will_retain = 1
};


/* Called when the MQTT connection changes state. */
static void mqtt_connection_callback(
    mqtt_client_t *client,
    void *arg,
    mqtt_connection_status_t status)
{
    (void)client;
    (void)arg;


    if (status == MQTT_CONNECT_ACCEPTED) {
        telemetry_due_immediately = 1;
        mqtt_status.state = MQTT_STATE_CONNECTED;
        was_connected = 1U;
        if (link_lost) {
            link_lost = 0U;
            buddy1_link_lost(false);
        }
        (void)mqtt_publish(client, MQTT_TOPIC_ONLINE, "1", 1, 0, 1,
                           NULL, NULL);

        /* Subscribe to commands from the laptop. */
        mqtt_subscribe(
            client,
            MQTT_TOPIC_COMMAND,
            0,
            mqtt_subscribe_callback,
            NULL
        );
    } else {
        telemetry_due_immediately = 1;
        mqtt_status.state = MQTT_STATE_CONNECT_FAILED;
    }
}



/* Save the address returned by DNS. */
static void save_broker_address(const ip_addr_t *address)
{
    const ip4_addr_t *ip = ip_2_ip4(address);

    ip_addr_copy(broker_address, *address);

    mqtt_status.broker_ip[0] = ip4_addr1(ip);
    mqtt_status.broker_ip[1] = ip4_addr2(ip);
    mqtt_status.broker_ip[2] = ip4_addr3(ip);
    mqtt_status.broker_ip[3] = ip4_addr4(ip);

    mqtt_status.dns_result = ERR_OK;
    mqtt_status.state = MQTT_STATE_BROKER_RESOLVED;
}

/* Called when DNS finishes resolving the broker. */
static void mqtt_dns_callback(
    const char *hostname,
    const ip_addr_t *address,
    void *arg)
{
    (void)hostname;
    (void)arg;

    if (mqtt_status.state != MQTT_STATE_RESOLVING) {
        return;
    }

    if (address == NULL) {
        mqtt_status.dns_result = ERR_VAL;
        mqtt_status.state = MQTT_STATE_DNS_FAILED;
        return;
    }

    save_broker_address(address);
}

/* Attempt an actual MQTT connection. */
static void mqtt_start_connection(void)
{
    err_t result;

    if (mqtt_client == NULL) {
        mqtt_client = mqtt_client_new();

        if (mqtt_client == NULL) {
            mqtt_status.state = MQTT_STATE_CONNECT_FAILED;
            return;
        }
    }


    /* Register callbacks for incoming MQTT messages. */
    mqtt_set_inpub_callback(
        mqtt_client,
        mqtt_incoming_publish,
        mqtt_incoming_data,
        NULL
    );


    last_connect_attempt_ms = sys_now();
    mqtt_status.state = MQTT_STATE_CONNECTING;

    result = mqtt_client_connect(
        mqtt_client,
        &broker_address,
        MQTT_BROKER_PORT,
        mqtt_connection_callback,
        NULL,
        &client_info
    );

    if (result != ERR_OK) {
        mqtt_status.state = MQTT_STATE_CONNECT_FAILED;
    }
}

void lwip_utk_mqtt_poll(struct netif *netif)
{
    ip_addr_t address;
    err_t result;
    uint32_t now = sys_now();

    /* FAULT_WIFI_LOSS once the broker has been gone LINK_LOSS_MS after
     * a connection (the car drives on; the controller only logs it) */
    if (mqtt_status.state == MQTT_STATE_CONNECTED) {
        down_since_ms = now;
    } else if (was_connected && !link_lost &&
               (uint32_t)(now - down_since_ms) >= LINK_LOSS_MS) {
        link_lost = 1U;
        buddy1_link_lost(true);
    }

    /* Wait for an active Wi-Fi connection and DHCP lease. */
    if (netif == NULL ||
        !netif_is_up(netif) ||
        !netif_is_link_up(netif) ||
        !dhcp_supplied_address(netif)) {

        if (mqtt_client != NULL &&
            mqtt_client_is_connected(mqtt_client)) {
            mqtt_disconnect(mqtt_client);
        }

        mqtt_status.state = MQTT_STATE_WAIT_NETWORK;
        return;
    }

 /*
 * Detect when the MQTT connection is no longer active,
 * even if Wi-Fi and DHCP still appear available.
 */
if (mqtt_status.state == MQTT_STATE_CONNECTED &&
    (mqtt_client == NULL ||
     !mqtt_client_is_connected(mqtt_client))) {

    mqtt_status.state = MQTT_STATE_CONNECT_FAILED;
    last_connect_attempt_ms = now;
    telemetry_due_immediately = 1U;
    return;
}



    /* Publish simulated binary telemetry every 500 ms. */
if (mqtt_status.state == MQTT_STATE_CONNECTED) {

    if (telemetry_due_immediately ||
        (uint32_t)(now - last_telemetry_ms) >= TELEMETRY_INTERVAL_MS) {

        /*
         * Temporary simulated telemetry:
         * Byte 0: robot state (1 = running)
         * Byte 1: simulated speed (100 mm/s)
         * Byte 2: simulated distance (25 cm)
         *
         * These fields are provisional.
         * Final packing depends on agreed ranges and precision.
         */
        /* Live Telemetry V2 (31 Oct): the car fills the fields */
        telemetry_data_t telemetry = {0};

        buddy1_fill_telemetry(&telemetry);
        telemetry.version = TELEMETRY_V2_VERSION;
        telemetry.heartbeat = heartbeat_counter;

uint8_t telemetry_payload[TELEMETRY_V2_SIZE];

if (!telemetry_encode(&telemetry, telemetry_payload)) {
    return;
}



        err_t publish_result = mqtt_publish(
            mqtt_client,
            MQTT_TOPIC_TELEMETRY,
            telemetry_payload,
            sizeof(telemetry_payload),
            0,
            0,
            NULL,
            NULL
        );

        if (publish_result == ERR_OK) {
            last_telemetry_ms = now;
            telemetry_due_immediately = 0;
            heartbeat_counter++;
        } else {
             /* Diagnostic: limit logging to once every 5 seconds. */
             static uint32_t last_publish_error_log_ms = 0;

             if ((uint32_t)(now - last_publish_error_log_ms) >= 5000U) {
               last_publish_error_log_ms = now;
               tm_printf(
                   (const unsigned char *)"[mqtt-debug] Publish failed: err=%d\n",
                   (int)publish_result
                );
             }
        }
    }

    /* Controller events, oldest first; one that does not fit the
     * output buffer waits for a later poll */
    {
        uint8_t event[BUDDY1_EVENT_LEN];
        unsigned int sent = 0U;

        while ((sent < EVENTS_PER_POLL) && buddy1_event_peek(event)) {
            if (mqtt_publish(mqtt_client, MQTT_TOPIC_EVENT, event,
                             sizeof(event), 0, 0, NULL, NULL) != ERR_OK) {
                break;
            }
            buddy1_event_done();
            sent++;
        }
    }

    /* Timing figures once a second (QoS 0, best effort) */
    if ((uint32_t)(now - last_sys_ms) >= SYS_INTERVAL_MS) {
        uint8_t sys[BUDDY1_SYS_LEN];

        last_sys_ms = now;
        buddy1_sys_pack(sys);
        (void)mqtt_publish(mqtt_client, MQTT_TOPIC_SYS, sys,
                           sizeof(sys), 0, 0, NULL, NULL);
    }
    return;
}

    if (mqtt_status.state == MQTT_STATE_CONNECTING) {
        return;
    }



    /* Broker address is available: begin MQTT connection. */
    if (mqtt_status.state == MQTT_STATE_BROKER_RESOLVED) {
        mqtt_start_connection();
        return;
    }

    /* Retry failed MQTT connections after 5 seconds. */
    if (mqtt_status.state == MQTT_STATE_CONNECT_FAILED) {
        if ((uint32_t)(now - last_connect_attempt_ms) >= 5000U) {
            mqtt_start_connection();
        }
        return;
    }

    /* Wait for an outstanding DNS request. */
    if (mqtt_status.state == MQTT_STATE_RESOLVING) {
        if ((uint32_t)(now - last_dns_attempt_ms) >= 10000U) {
            mqtt_status.dns_result = ERR_TIMEOUT;
            mqtt_status.state = MQTT_STATE_DNS_FAILED;
        }
        return;
    }

    /* Avoid excessive DNS retry attempts. */
    if (mqtt_status.state == MQTT_STATE_DNS_FAILED &&
        (uint32_t)(now - last_dns_attempt_ms) < 10000U) {
        return;
    }

    last_dns_attempt_ms = now;
    mqtt_status.state = MQTT_STATE_RESOLVING;

    result = dns_gethostbyname(
        MQTT_BROKER_HOST,
        &address,
        mqtt_dns_callback,
        NULL
    );

    if (result == ERR_OK) {
        save_broker_address(&address);
    } else if (result != ERR_INPROGRESS) {
        mqtt_status.dns_result = result;
        mqtt_status.state = MQTT_STATE_DNS_FAILED;
    }
}

void lwip_utk_mqtt_get_status(mqtt_status_t *status)
{
    if (status != NULL) {
        *status = mqtt_status;
    }
}


/* Retrieve a received command for debugging. */
int lwip_utk_mqtt_get_command(char *buffer, uint16_t buffer_size)
{
    if (buffer == NULL || buffer_size < 2U || !command_received) {
        return 0;
    }

    if (command_invalid || received_command_length != 1U) {
        command_received = 0U;
        return 0;
    }

    /* Convert the validated binary command into readable text. */
    if (received_command == MQTT_COMMAND_START) {
        buffer[0] = 'S';
        buffer[1] = '\0';
    } else if (received_command == MQTT_COMMAND_STOP) {
        buffer[0] = 'P';
        buffer[1] = '\0';
    } else if (command_known(received_command)) {
        buffer[0] = "lobarp"[received_command - MQTT_COMMAND_RUN_FIRST];
        buffer[1] = '\0';
    } else {
        command_received = 0U;
        return 0;
    }

    command_received = 0U;
    return 1;
}