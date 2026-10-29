/**
 * @file tel_source.c
 * @brief Packs the controller's status and events for Buddy 1 (see
 * tel_source.h for the byte layout).
 *
 * Called from the radio task; it only copies under the controller's own
 * lock, so it never waits on the control loop.
 */

#include "controller/vehicle_ctrl.h"
#include "buddy1_telemetry/tel_source.h"

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

void
tel_status_pack(uint8_t p_out[TEL_STATUS_LEN])
{
    robot_status_t st;

    ctrl_get_status(&st);
    p_out[0] = st.state;
    p_out[1] = st.faults;
    p_out[2] = st.last_evt;
    p_out[3] = st.flags;
    put16(&p_out[4], (uint16_t)st.dist_mm);
    put16(&p_out[6], (uint16_t)st.heading_cdeg);
    put16(&p_out[8], (uint16_t)st.roll_cdeg);
    put16(&p_out[10], (uint16_t)st.obstacle_mm);
    put16(&p_out[12], (uint16_t)st.obstacle_width_mm);
    p_out[14] = st.max_ramp_mm;
    p_out[15] = st.spare;
    put16(&p_out[16], st.total_cm);
}

uint32_t
tel_event_count(void)
{
    return ctrl_log_count();
}

bool
tel_event_pack(uint32_t idx, uint8_t p_out[TEL_EVENT_LEN])
{
    ctrl_log_t e;

    if (!ctrl_log_at(idx, &e))
    {
        return false;
    }
    put32(&p_out[0], e.t_ms);
    put32(&p_out[4], (uint32_t)e.d_mm);
    p_out[8] = e.type;
    p_out[9] = e.arg;
    put16(&p_out[10], (uint16_t)e.val);
    put16(&p_out[12], (uint16_t)idx);
    return true;
}
