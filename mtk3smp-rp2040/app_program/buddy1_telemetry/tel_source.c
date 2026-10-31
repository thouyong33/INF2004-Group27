/**
 * @file tel_source.c
 * @brief Buddy 1: the car's side of the MQTT telemetry (the hooks in
 * lib/libnet/lwip/buddy1_hooks.h, used by the MQTT client in
 * lwip_utk_mqtt.c).
 *
 * Called from the radio task. It only copies under the controller's and
 * the motion task's own locks, so it never waits on the control loop.
 * Built only with WIFI_MQTT=1.
 */

#if TM_WIFI_MQTT

#include "controller/vehicle_ctrl.h"
#include "buddy2_motion/motion_task.h"
#include "common/faults.h"
#include "common/remote_keys.h"
#include "buddy1_hooks.h"

#define TEL_SPEED_MAX_MM_S   550     /* the codec's range */
#define TEL_OBST_MAX_MM      4095

static uint32_t g_evt_next = 0u;     /* next event index of the run */

static void
put16(uint8_t *p_b, uint16_t v)
{
    p_b[0] = (uint8_t)v;
    p_b[1] = (uint8_t)(v >> 8);
}

static void
put32(uint8_t *p_b, uint32_t v)
{
    put16(p_b, (uint16_t)v);
    put16(&p_b[2], (uint16_t)(v >> 16));
}

static int16_t
clamp_speed(float cm_s)
{
    int32_t mm_s = (int32_t)(cm_s * 10.0f);

    mm_s = (mm_s > TEL_SPEED_MAX_MM_S) ? TEL_SPEED_MAX_MM_S : mm_s;
    mm_s = (mm_s < -TEL_SPEED_MAX_MM_S) ? -TEL_SPEED_MAX_MM_S : mm_s;
    return (int16_t)mm_s;
}

void
buddy1_fill_telemetry(telemetry_data_t *p_data)
{
    robot_status_t st;
    odometry_t odo = {0};
    uint8_t cmd;

    ctrl_get_status(&st);
    (void)motion_get_odometry(&odo);
    cmd = (uint8_t)((st.flags >> CTRL_FLAG_CMD_SHIFT) & 0x07u);

    p_data->state = st.state;
    p_data->left_speed_mm_s = clamp_speed(odo.left_speed_cm_s);
    p_data->right_speed_mm_s = clamp_speed(odo.right_speed_cm_s);
    /* run distance; wraps every 65.5 m */
    p_data->distance_mm = (uint16_t)((uint32_t)st.total_cm * 10u);
    p_data->barcode_command = cmd;
    p_data->barcode_valid = (CTRL_CMD_NONE != cmd);
    p_data->max_hump_mm = st.max_ramp_mm;
    p_data->obstacle_detected = (st.obstacle_mm >= 0);
    p_data->obstacle_distance_mm = (st.obstacle_mm < 0) ? 0u
        : ((st.obstacle_mm > TEL_OBST_MAX_MM) ? (uint16_t)TEL_OBST_MAX_MM
                                              : (uint16_t)st.obstacle_mm);
    p_data->fault_flags = st.faults;
}

bool
buddy1_event_peek(uint8_t p_out[BUDDY1_EVENT_LEN])
{
    uint32_t n = ctrl_log_count();
    ctrl_log_t e;

    if (n < g_evt_next)
    {
        g_evt_next = 0u;                     /* a new run has started */
    }
    if ((n - g_evt_next) > CTRL_LOG_LEN)
    {
        g_evt_next = n - CTRL_LOG_LEN;       /* the oldest are gone */
    }
    while (g_evt_next < n)
    {
        if (ctrl_log_at(g_evt_next, &e))
        {
            put32(&p_out[0], e.t_ms);
            put32(&p_out[4], (uint32_t)e.d_mm);
            p_out[8] = e.type;
            p_out[9] = e.arg;
            put16(&p_out[10], (uint16_t)e.val);
            put16(&p_out[12], (uint16_t)g_evt_next);
            return true;
        }
        g_evt_next++;                        /* overwritten meanwhile */
    }
    return false;
}

void
buddy1_event_done(void)
{
    g_evt_next++;
}

void
buddy1_sys_pack(uint8_t p_out[BUDDY1_SYS_LEN])
{
    static uint32_t s_prev_samples = 0u;
    uint32_t samples = ctrl_sample_count();
    uint32_t busy = 0u;
    uint32_t pass = 0u;

    motion_get_cpu(&busy, &pass);
    put16(&p_out[0], (uint16_t)motion_loop_rate());
    put32(&p_out[2], busy);
    put16(&p_out[6], (uint16_t)pass);
    put16(&p_out[8], (uint16_t)(samples - s_prev_samples));
    put16(&p_out[10], (uint16_t)ctrl_sample_gap_max_ms());
    put16(&p_out[12], 0u);
    s_prev_samples = samples;
}

/* The command byte as a course-test key: START = the continuous course,
   STOP = s, 0x10-0x15 = l o b a r p (lwip_utk_mqtt.c) */
void
buddy1_command(uint8_t command)
{
    static const char keys[] = "lobarp";

    if (0x00u == command)
    {
        console_inject((uint8_t)'c');
    }
    else if (0x01u == command)
    {
        console_inject((uint8_t)'s');
    }
    else if ((command >= 0x10u) && (command <= 0x15u))
    {
        console_inject((uint8_t)keys[command - 0x10u]);
    }
    else
    {
        /* rejected by the client already */
    }
}

void
buddy1_link_lost(bool b_lost)
{
    if (b_lost)
    {
        fault_set(FAULT_WIFI_LOSS);   /* logged; the car drives on */
    }
    else
    {
        fault_clear(FAULT_WIFI_LOSS);
    }
}

#endif /* TM_WIFI_MQTT */
