/**
 * @file mqtt_tel.c
 * @brief Buddy 1: lwIP MQTT client publishing the car's status and events.
 *
 * Everything here except mqtt_tel_get_info() runs in the radio task:
 * cyw43_utk_app_poll() is called on each of its 2 ms polls, and lwIP's
 * callbacks come from inside that same poll. Publishing is QoS 0: a lost
 * status is replaced by the next one 200 ms later. An event that does not
 * fit the output buffer stays queued and goes on a later poll, so events
 * logged while the broker is away are sent when it is back (the latest
 * TEL_EVENT_KEPT of them).
 *
 * Keys typed on the laptop arrive on car27/cmd and go into the console
 * input (remote_keys.h). Once connected, losing the broker for longer
 * than MQTT_TEL_LOSS_MS raises FAULT_WIFI_LOSS; reconnecting clears it.
 */

#if TM_WIFI_MQTT

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "lwip/apps/mqtt.h"
#include "lwip/err.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwip/sys.h"

#include "mqtt_config.h"
#include "buddy1_telemetry/mqtt_tel.h"
#include "buddy1_telemetry/tel_source.h"
#include "common/remote_keys.h"

#define MQTT_TEL_STATUS_MS     200u    /* 5 Hz */
#define MQTT_TEL_SYS_MS        1000u
#define MQTT_TEL_RETRY_MS      2000u
#define MQTT_TEL_LOSS_MS       3000u   /* broker gone this long: fault */
#define MQTT_TEL_KEEPALIVE_S   10u
#define MQTT_TEL_EVT_PER_POLL  4u

#define MQTT_TEL_TOPIC_ONLINE  "car27/online"
#define MQTT_TEL_TOPIC_STATUS  "car27/status"
#define MQTT_TEL_TOPIC_EVENT   "car27/event"
#define MQTT_TEL_TOPIC_SYS     "car27/sys"
#define MQTT_TEL_TOPIC_CMD     "car27/cmd"

void cyw43_utk_app_poll(struct netif *p_netif);

static const struct mqtt_connect_client_info_t g_client_info = {
    .client_id = "car27",
    .client_user = NULL,
    .client_pass = NULL,
    .keep_alive = MQTT_TEL_KEEPALIVE_S,
    .will_topic = MQTT_TEL_TOPIC_ONLINE,
    .will_msg = "0",
    .will_msg_len = 1u,
    .will_qos = 0u,
    .will_retain = 1u,
};

static mqtt_client_t *gp_client = NULL;
static ip_addr_t g_broker;
static bool g_b_have_broker = false;
static bool g_b_tried = false;
static bool g_b_was_up = false;       /* connected at least once */
static bool g_b_loss = false;         /* FAULT_WIFI_LOSS raised */
static uint32_t g_try_ms = 0u;
static uint32_t g_down_ms = 0u;       /* when the connection was lost */
static uint32_t g_status_ms = 0u;
static uint32_t g_sys_ms = 0u;
static uint32_t g_evt_next = 0u;      /* next event index of the run */
static volatile mqtt_tel_info_t g_info = { MQTT_TEL_ST_NO_IP, 0, 0u, 0u,
                                           0u, 0u };

static void
on_cmd_topic(void *p_arg, const char *p_topic, u32_t tot_len)
{
    (void)p_arg;
    (void)p_topic;                    /* car27/cmd is the only subscription */
    (void)tot_len;
}

static void
on_cmd_data(void *p_arg, const u8_t *p_data, u16_t len, u8_t flags)
{
    u16_t i;

    (void)p_arg;
    (void)flags;
    for (i = 0u; i < len; i++)
    {
        if ((p_data[i] >= (u8_t)'a') && (p_data[i] <= (u8_t)'z'))
        {
            console_inject(p_data[i]);
        }
    }
}

static void
on_connection(mqtt_client_t *p_client, void *p_arg,
              mqtt_connection_status_t status)
{
    (void)p_arg;
    g_info.last_result = (int32_t)status;
    if (MQTT_CONNECT_ACCEPTED == status)
    {
        g_info.state = MQTT_TEL_ST_UP;
        g_info.connects++;
        g_b_was_up = true;
        if (g_b_loss)
        {
            g_b_loss = false;
            tel_wifi_loss(false);
        }
        (void)mqtt_publish(p_client, MQTT_TEL_TOPIC_ONLINE, "1", 1u, 0u, 1u,
                           NULL, NULL);
        mqtt_set_inpub_callback(p_client, on_cmd_topic, on_cmd_data, NULL);
        (void)mqtt_subscribe(p_client, MQTT_TEL_TOPIC_CMD, 0u, NULL, NULL);
    }
    else
    {
        /* refused, timed out or dropped: try again after the pause */
        if (MQTT_TEL_ST_UP == g_info.state)
        {
            g_down_ms = sys_now();
        }
        g_info.state = MQTT_TEL_ST_RETRY;
        g_try_ms = sys_now();
    }
}

static void
check_loss(uint32_t now)
{
    if (g_b_was_up && !g_b_loss && (MQTT_TEL_ST_UP != g_info.state)
        && ((now - g_down_ms) >= MQTT_TEL_LOSS_MS))
    {
        g_b_loss = true;
        tel_wifi_loss(true);          /* logged; the car drives on */
    }
}

static void
try_connect(uint32_t now)
{
    err_t err;

    if (g_b_tried && ((now - g_try_ms) < MQTT_TEL_RETRY_MS))
    {
        return;
    }
    g_b_tried = true;
    g_try_ms = now;
    err = mqtt_client_connect(gp_client, &g_broker,
                              (u16_t)UTK_MQTT_BROKER_PORT, on_connection,
                              NULL, &g_client_info);
    if (ERR_OK == err)
    {
        g_info.state = MQTT_TEL_ST_CONNECTING;
    }
    else if (ERR_ISCONN != err)      /* ISCONN: an attempt is under way */
    {
        g_info.state = MQTT_TEL_ST_RETRY;
        g_info.last_result = (int32_t)err;
    }
}

static void
publish_status(uint32_t now)
{
    uint8_t buf[TEL_STATUS_LEN];

    if ((now - g_status_ms) < MQTT_TEL_STATUS_MS)
    {
        return;
    }
    /* keep to the 200 ms grid (a poll's lateness is not added each time)
       unless far behind, e.g. after a reconnect */
    g_status_ms = ((now - g_status_ms) < (2u * MQTT_TEL_STATUS_MS))
                  ? (g_status_ms + MQTT_TEL_STATUS_MS) : now;
    tel_status_pack(buf);
    if (ERR_OK == mqtt_publish(gp_client, MQTT_TEL_TOPIC_STATUS, buf,
                               (u16_t)TEL_STATUS_LEN, 0u, 0u, NULL, NULL))
    {
        g_info.status_sent++;
    }
    else
    {
        g_info.drops++;
    }
}

static void
publish_sys(uint32_t now)
{
    uint8_t buf[TEL_SYS_LEN];

    if ((now - g_sys_ms) < MQTT_TEL_SYS_MS)
    {
        return;
    }
    g_sys_ms = now;
    tel_sys_pack(buf, g_info.drops);
    (void)mqtt_publish(gp_client, MQTT_TEL_TOPIC_SYS, buf,
                       (u16_t)TEL_SYS_LEN, 0u, 0u, NULL, NULL);
}

static void
publish_events(void)
{
    uint8_t buf[TEL_EVENT_LEN];
    uint32_t n = tel_event_count();
    uint32_t sent = 0u;

    if (n < g_evt_next)
    {
        g_evt_next = 0u;                     /* a new run has started */
    }
    if ((n - g_evt_next) > TEL_EVENT_KEPT)
    {
        g_evt_next = n - TEL_EVENT_KEPT;     /* the oldest are gone */
    }
    while ((g_evt_next < n) && (sent < MQTT_TEL_EVT_PER_POLL))
    {
        if (!tel_event_pack(g_evt_next, buf))
        {
            g_evt_next++;                    /* overwritten meanwhile */
            continue;
        }
        if (ERR_OK != mqtt_publish(gp_client, MQTT_TEL_TOPIC_EVENT, buf,
                                   (u16_t)TEL_EVENT_LEN, 0u, 0u, NULL, NULL))
        {
            break;                           /* buffer full: next poll */
        }
        g_evt_next++;
        g_info.events_sent++;
        sent++;
    }
}

void
cyw43_utk_app_poll(struct netif *p_netif)
{
    uint32_t now = sys_now();

    check_loss(now);
    if (!netif_is_up(p_netif) || !netif_is_link_up(p_netif)
        || ip4_addr_isany_val(*netif_ip4_addr(p_netif)))
    {
        if (MQTT_TEL_ST_UP == g_info.state)
        {
            /* link gone: close now rather than wait out the keep-alive
               (the broker then sends the last will) */
            mqtt_disconnect(gp_client);
            g_down_ms = now;
        }
        g_info.state = MQTT_TEL_ST_NO_IP;
        return;
    }
    if (NULL == gp_client)
    {
        g_b_have_broker = (0 != ipaddr_aton(UTK_MQTT_BROKER_ADDRESS,
                                            &g_broker));
        gp_client = mqtt_client_new();
    }
    if ((NULL == gp_client) || !g_b_have_broker)
    {
        g_info.state = MQTT_TEL_ST_RETRY;
        g_info.last_result = (int32_t)ERR_ARG;
        return;
    }
    if (!mqtt_client_is_connected(gp_client))
    {
        try_connect(now);
        return;
    }
    publish_status(now);
    publish_sys(now);
    publish_events();
}

void
mqtt_tel_get_info(mqtt_tel_info_t *p_info)
{
    if (NULL != p_info)
    {
        p_info->state = g_info.state;
        p_info->last_result = g_info.last_result;
        p_info->connects = g_info.connects;
        p_info->status_sent = g_info.status_sent;
        p_info->events_sent = g_info.events_sent;
        p_info->drops = g_info.drops;
    }
}

#endif /* TM_WIFI_MQTT */
