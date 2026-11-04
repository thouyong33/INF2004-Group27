/**
 * @file lwip_utk_mqtt.c
 * @brief Buddy 1: MQTT client (telemetry, events, timing figures,
 *        commands), polled from the radio task.
 */

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
/* Periodic telemetry timing */
#define TELEMETRY_INTERVAL_MS 500U
/* Retry a failed connection, and limit DNS retries, this often */
#define CONNECT_RETRY_MS   5000U
#define DNS_RETRY_MS       10000U
/* Log a failed publish at most this often */
#define PUBLISH_LOG_MS     5000U

/*
 * Binary MQTT command protocol:
 * 0x00 = START
 * 0x01 = STOP
 *
 * FINISH (0x02) remains reserved until confirmed.
 * 0x10-0x16 = start one test run: l o b a r, p (print the
 * continuous run's log) and c (the continuous course with barcode
 * commands off, a lapping test) - added at integration.
 *
 * A valid command is passed to buddy1_command(), which feeds the
 * course test as a keystroke (START = f, the full course with barcode
 * commands carried out; STOP = s).
 */
#define MQTT_COMMAND_START 0x00U
#define MQTT_COMMAND_STOP  0x01U
#define MQTT_COMMAND_RUN_FIRST 0x10U
#define MQTT_COMMAND_RUN_LAST  0x16U

static uint32_t g_last_sys_ms;
static uint8_t g_b_was_connected;
static uint8_t g_b_link_lost;
static uint32_t g_down_since_ms;

/* MQTT client and broker address */
static mqtt_client_t *g_p_mqtt_client;
static ip_addr_t g_broker_address;

static mqtt_status_t g_mqtt_status =
{
    .state = MQTT_STATE_WAIT_NETWORK,
    .dns_result = ERR_INPROGRESS
};

static uint32_t g_last_dns_attempt_ms;
static uint32_t g_last_connect_attempt_ms;

static uint32_t g_last_telemetry_ms;
static uint8_t g_b_telemetry_due_now = 1U;
static uint8_t g_heartbeat_counter = 0U;

/* The command being received (set by the MQTT callbacks) */
static uint8_t g_received_command;
static uint32_t g_received_command_length;
static uint8_t g_b_command_received;
static uint8_t g_b_command_invalid;

static int
command_known(uint8_t command)
{
    return (MQTT_COMMAND_START == command)
           || (MQTT_COMMAND_STOP == command)
           || ((command >= MQTT_COMMAND_RUN_FIRST)
               && (command <= MQTT_COMMAND_RUN_LAST));
}

/* Called when an MQTT message arrives on a subscribed topic. */
static void
mqtt_incoming_publish(void *arg, const char *topic, u32_t total_length)
{
    (void)arg;
    (void)topic;

    g_received_command = 0U;
    g_received_command_length = 0U;
    g_b_command_received = 0U;

    /* Only accept a one-byte binary command. */
    g_b_command_invalid = (1U != total_length);
}

/* Called with the incoming MQTT message payload. */
static void
mqtt_incoming_data(void *arg, const u8_t *data, u16_t length, u8_t flags)
{
    (void)arg;

    /*
     * Preserve the first byte if valid, and count all
     * incoming bytes, including fragmented payloads.
     */
    if (length > 0U)
    {
        if (NULL == data)
        {
            g_b_command_invalid = 1U;
        }
        else if (0U == g_received_command_length)
        {
            g_received_command = data[0];
        }
        else
        {
            /* a later fragment: only counted */
        }

        g_received_command_length += length;
    }

    if (0U != (flags & MQTT_DATA_FLAG_LAST))
    {
        if (!g_b_command_invalid && (1U == g_received_command_length)
            && command_known(g_received_command))
        {
            g_b_command_received = 1U;
            buddy1_command(g_received_command);
        }
        else
        {
            g_b_command_invalid = 1U;
            g_b_command_received = 0U;
        }
    }
}

/* Called when the subscription request finishes. */
static void
mqtt_subscribe_callback(void *arg, err_t result)
{
    (void)arg;
    (void)result;
}

/* MQTT connection settings */
/* Last will (31 Oct): the broker publishes "0" on the online topic
 * if the car drops off without disconnecting; keep-alive 10 s so it
 * is noticed within ~15 s. */
static const struct mqtt_connect_client_info_t g_client_info =
{
    .client_id = MQTT_CLIENT_ID,
    .keep_alive = 10,
    .will_topic = MQTT_TOPIC_ONLINE,
    .will_msg = "0",
    .will_msg_len = 1,
    .will_qos = 0,
    .will_retain = 1
};

/* Called when the MQTT connection changes state. */
static void
mqtt_connection_callback(mqtt_client_t *client, void *arg,
                         mqtt_connection_status_t status)
{
    (void)client;
    (void)arg;

    if (MQTT_CONNECT_ACCEPTED == status)
    {
        g_b_telemetry_due_now = 1U;
        g_mqtt_status.state = MQTT_STATE_CONNECTED;
        g_b_was_connected = 1U;
        if (g_b_link_lost)
        {
            g_b_link_lost = 0U;
            buddy1_link_lost(false);
        }
        (void)mqtt_publish(client, MQTT_TOPIC_ONLINE, "1", 1, 0, 1,
                           NULL, NULL);

        /* Subscribe to commands from the laptop. */
        mqtt_subscribe(client, MQTT_TOPIC_COMMAND, 0,
                       mqtt_subscribe_callback, NULL);
    }
    else
    {
        g_b_telemetry_due_now = 1U;
        g_mqtt_status.state = MQTT_STATE_CONNECT_FAILED;
    }
}

/* Save the address returned by DNS. */
static void
save_broker_address(const ip_addr_t *p_address)
{
    const ip4_addr_t *p_ip = ip_2_ip4(p_address);

    ip_addr_copy(g_broker_address, *p_address);

    g_mqtt_status.broker_ip[0] = ip4_addr1(p_ip);
    g_mqtt_status.broker_ip[1] = ip4_addr2(p_ip);
    g_mqtt_status.broker_ip[2] = ip4_addr3(p_ip);
    g_mqtt_status.broker_ip[3] = ip4_addr4(p_ip);

    g_mqtt_status.dns_result = ERR_OK;
    g_mqtt_status.state = MQTT_STATE_BROKER_RESOLVED;
}

/* Called when DNS finishes resolving the broker. */
static void
mqtt_dns_callback(const char *hostname, const ip_addr_t *p_address,
                  void *arg)
{
    (void)hostname;
    (void)arg;

    if (MQTT_STATE_RESOLVING != g_mqtt_status.state)
    {
        return;
    }

    if (NULL == p_address)
    {
        g_mqtt_status.dns_result = ERR_VAL;
        g_mqtt_status.state = MQTT_STATE_DNS_FAILED;
        return;
    }

    save_broker_address(p_address);
}

/* Attempt an actual MQTT connection. */
static void
mqtt_start_connection(void)
{
    err_t result;

    if (NULL == g_p_mqtt_client)
    {
        g_p_mqtt_client = mqtt_client_new();

        if (NULL == g_p_mqtt_client)
        {
            g_mqtt_status.state = MQTT_STATE_CONNECT_FAILED;
            return;
        }
    }

    /* Register callbacks for incoming MQTT messages. */
    mqtt_set_inpub_callback(g_p_mqtt_client, mqtt_incoming_publish,
                            mqtt_incoming_data, NULL);

    g_last_connect_attempt_ms = sys_now();
    g_mqtt_status.state = MQTT_STATE_CONNECTING;

    result = mqtt_client_connect(g_p_mqtt_client, &g_broker_address,
                                 MQTT_BROKER_PORT, mqtt_connection_callback,
                                 NULL, &g_client_info);

    if (ERR_OK != result)
    {
        g_mqtt_status.state = MQTT_STATE_CONNECT_FAILED;
    }
}

void
lwip_utk_mqtt_poll(struct netif *netif)
{
    ip_addr_t address;
    err_t result;
    uint32_t now = sys_now();

    /* FAULT_WIFI_LOSS once the broker has been gone LINK_LOSS_MS after
     * a connection (the car drives on; the controller only logs it) */
    if (MQTT_STATE_CONNECTED == g_mqtt_status.state)
    {
        g_down_since_ms = now;
    }
    else if (g_b_was_connected && !g_b_link_lost
             && ((uint32_t)(now - g_down_since_ms) >= LINK_LOSS_MS))
    {
        g_b_link_lost = 1U;
        buddy1_link_lost(true);
    }
    else
    {
        /* still connecting, or the loss is already reported */
    }

    /* Wait for an active Wi-Fi connection and DHCP lease. */
    if ((NULL == netif) || !netif_is_up(netif) || !netif_is_link_up(netif)
        || !dhcp_supplied_address(netif))
    {
        if ((NULL != g_p_mqtt_client)
            && mqtt_client_is_connected(g_p_mqtt_client))
        {
            mqtt_disconnect(g_p_mqtt_client);
        }

        g_mqtt_status.state = MQTT_STATE_WAIT_NETWORK;
        return;
    }

    /*
     * Detect when the MQTT connection is no longer active,
     * even if Wi-Fi and DHCP still appear available.
     */
    if ((MQTT_STATE_CONNECTED == g_mqtt_status.state)
        && ((NULL == g_p_mqtt_client)
            || !mqtt_client_is_connected(g_p_mqtt_client)))
    {
        g_mqtt_status.state = MQTT_STATE_CONNECT_FAILED;
        g_last_connect_attempt_ms = now;
        g_b_telemetry_due_now = 1U;
        return;
    }

    /* Connected: telemetry every TELEMETRY_INTERVAL_MS, then events and
     * the timing figures */
    if (MQTT_STATE_CONNECTED == g_mqtt_status.state)
    {
        if (g_b_telemetry_due_now
            || ((uint32_t)(now - g_last_telemetry_ms)
                >= TELEMETRY_INTERVAL_MS))
        {
            /* Live Telemetry V2 (31 Oct): the car fills the fields */
            telemetry_data_t telemetry = {0};

            buddy1_fill_telemetry(&telemetry);
            telemetry.version = TELEMETRY_V2_VERSION;
            telemetry.heartbeat = g_heartbeat_counter;

            uint8_t telemetry_payload[TELEMETRY_V2_SIZE];

            if (!telemetry_encode(&telemetry, telemetry_payload))
            {
                return;
            }

            err_t publish_result = mqtt_publish(g_p_mqtt_client,
                                                MQTT_TOPIC_TELEMETRY,
                                                telemetry_payload,
                                                sizeof(telemetry_payload),
                                                0, 0, NULL, NULL);

            if (ERR_OK == publish_result)
            {
                g_last_telemetry_ms = now;
                g_b_telemetry_due_now = 0U;
                g_heartbeat_counter++;
            }
            else
            {
                /* Diagnostic: limit logging to once every 5 seconds. */
                static uint32_t last_publish_error_log_ms = 0U;

                if ((uint32_t)(now - last_publish_error_log_ms)
                    >= PUBLISH_LOG_MS)
                {
                    last_publish_error_log_ms = now;
                    tm_printf((const unsigned char *)"[mqtt-debug] Publish"
                              " failed: err=%d\n", (int)publish_result);
                }
            }
        }

        /* Controller events, oldest first; one that does not fit the
         * output buffer waits for a later poll */
        {
            uint8_t event[BUDDY1_EVENT_LEN];
            unsigned int sent = 0U;

            while ((sent < EVENTS_PER_POLL) && buddy1_event_peek(event))
            {
                if (ERR_OK != mqtt_publish(g_p_mqtt_client, MQTT_TOPIC_EVENT,
                                           event, sizeof(event), 0, 0, NULL,
                                           NULL))
                {
                    break;
                }
                buddy1_event_done();
                sent++;
            }
        }

        /* Timing figures once a second (QoS 0, best effort) */
        if ((uint32_t)(now - g_last_sys_ms) >= SYS_INTERVAL_MS)
        {
            uint8_t sys[BUDDY1_SYS_LEN];

            g_last_sys_ms = now;
            buddy1_sys_pack(sys);
            (void)mqtt_publish(g_p_mqtt_client, MQTT_TOPIC_SYS, sys,
                               sizeof(sys), 0, 0, NULL, NULL);
        }
        return;
    }

    if (MQTT_STATE_CONNECTING == g_mqtt_status.state)
    {
        return;
    }

    /* Broker address is available: begin MQTT connection. */
    if (MQTT_STATE_BROKER_RESOLVED == g_mqtt_status.state)
    {
        mqtt_start_connection();
        return;
    }

    /* Retry failed MQTT connections after CONNECT_RETRY_MS. */
    if (MQTT_STATE_CONNECT_FAILED == g_mqtt_status.state)
    {
        if ((uint32_t)(now - g_last_connect_attempt_ms) >= CONNECT_RETRY_MS)
        {
            mqtt_start_connection();
        }
        return;
    }

    /* Wait for an outstanding DNS request. */
    if (MQTT_STATE_RESOLVING == g_mqtt_status.state)
    {
        if ((uint32_t)(now - g_last_dns_attempt_ms) >= DNS_RETRY_MS)
        {
            g_mqtt_status.dns_result = ERR_TIMEOUT;
            g_mqtt_status.state = MQTT_STATE_DNS_FAILED;
        }
        return;
    }

    /* Avoid excessive DNS retry attempts. */
    if ((MQTT_STATE_DNS_FAILED == g_mqtt_status.state)
        && ((uint32_t)(now - g_last_dns_attempt_ms) < DNS_RETRY_MS))
    {
        return;
    }

    g_last_dns_attempt_ms = now;
    g_mqtt_status.state = MQTT_STATE_RESOLVING;

    result = dns_gethostbyname(MQTT_BROKER_HOST, &address, mqtt_dns_callback,
                               NULL);

    if (ERR_OK == result)
    {
        save_broker_address(&address);
    }
    else if (ERR_INPROGRESS != result)
    {
        g_mqtt_status.dns_result = result;
        g_mqtt_status.state = MQTT_STATE_DNS_FAILED;
    }
    else
    {
        /* resolving: mqtt_dns_callback() finishes it */
    }
}

void
lwip_utk_mqtt_get_status(mqtt_status_t *p_status)
{
    if (NULL != p_status)
    {
        *p_status = g_mqtt_status;
    }
}

/* Retrieve a received command for debugging. */
int
lwip_utk_mqtt_get_command(char *p_buffer, uint16_t buffer_size)
{
    if ((NULL == p_buffer) || (buffer_size < 2U) || !g_b_command_received)
    {
        return 0;
    }

    if (g_b_command_invalid || (1U != g_received_command_length))
    {
        g_b_command_received = 0U;
        return 0;
    }

    /* Convert the validated binary command into readable text. */
    if (MQTT_COMMAND_START == g_received_command)
    {
        p_buffer[0] = 'S';
        p_buffer[1] = '\0';
    }
    else if (MQTT_COMMAND_STOP == g_received_command)
    {
        p_buffer[0] = 'P';
        p_buffer[1] = '\0';
    }
    else if (command_known(g_received_command))
    {
        p_buffer[0] =
            "lobarpc"[g_received_command - MQTT_COMMAND_RUN_FIRST];
        p_buffer[1] = '\0';
    }
    else
    {
        g_b_command_received = 0U;
        return 0;
    }

    g_b_command_received = 0U;
    return 1;
}
