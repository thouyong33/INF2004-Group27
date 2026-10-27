/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * app_main.c
 * Demo application for the micro T-Kernel 3.0 RP2040 SMP port.
 *
 * Four tasks and one pass of every IPC primitive the kernel offers, wired
 * into a single producer/consumer pipeline so the objects are used the
 * way they are meant to be used rather than poked in isolation:
 *
 * producer --[fixed memory pool]--> record
 *           --[mutex]-------------> shared sequence counter
 *           --[message buffer]----> consumer
 * consumer --[semaphore]---------> credit returned to producer
 *          --[event flag]--------> monitor woken
 * monitor prints a status line
 * blink LED liveness, independent of all of the above
 *
 * Under SMP the producer is pinned to processor 1 and the consumer to
 * processor 2, so every message crosses a core boundary and the printed
 * processor numbers differ. Built with SMP=0 the same source runs
 * unchanged on one core.
 *
 * Deliberately small. Read it top to bottom, change one thing, reflash.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#if TK_SUPPORT_SMP
#include <tk/smp.h>
/* Which processor is the caller on? Numbered from 1, so processor 1 is
   RP2040 core 0 and processor 2 is core 1. */
#define THIS_PRC() tk_get_prc()
#else
/* Single-core build: there is only ever processor 1. tk_get_prc() is
   declared by the kernel headers but not linked in this configuration, so
   the demo asks through this macro and the rest of the source is identical
   for both builds. */
#define THIS_PRC() 1
#endif

#include "common/usb_console_compat.h"
#include "demo/demo_tasks.h"

// ADDED: Robot task includes
#include "buddy3_line_barcode/barcode_task.h"
#include "buddy3_line_barcode/ir_sensor.h"
#include "buddy3_line_barcode/line.h"
#include "buddy3_line_barcode/barcode.h"
#include "buddy3_line_barcode/barcode_scan.h"
#include "common/pin_defs.h"
#include "buddy2_motion/motion_task.h"
#include "buddy4_imu/imu.h"
#include "buddy4_imu/terrain.h"
#include "buddy5_ultrasonic/ultrasonic.h"
#include "buddy5_ultrasonic/servo.h"
#include "buddy5_ultrasonic/scan.h"
#include "buddy5_ultrasonic/obstacle.h"
#include "common/console_in.h"
#include "common/faults.h"
#include "common/ctrl_events.h"
#include "controller/vehicle_ctrl.h"

#if TM_WIFI_CYW43
/* Plain C types on purpose - see the note in cyw43_utk.h. */
#include "cyw43_utk.h"
#endif

/* ------------------------------------------------------------------ *
 * Tunables
 * ------------------------------------------------------------------ */

/* The producer/consumer/monitor pipeline is the port's IPC demo. It is
   off for the robot build: set APP_DEMO_PIPELINE to 1 to bring it back.
   The blink task is independent and always runs. */
#define APP_DEMO_PIPELINE 0

#if APP_DEMO_PIPELINE
#define N_RECORDS   4      /* blocks in the fixed memory pool */
#define CREDITS     2      /* messages in flight at once */
#define PRODUCE_MS  500    /* one record every half second */
#define REPORT_MS   2000   /* monitor prints this often */

#define FLG_PRODUCED (1U << 0)
#define FLG_CONSUMED (1U << 1)
#endif /* APP_DEMO_PIPELINE */

/* Every task uses 4096 bytes. The SMP kernel path is deeper than the
   single-core one -- each critical section runs the global ready-queue
   assignment on the calling task's stack -- so 1-2 KiB is no longer enough
   for a task that makes blocking kernel calls. */
#define STACK_SZ 4096

#if APP_DEMO_PIPELINE
/* ------------------------------------------------------------------ *
 * One record travelling through the pipeline
 * ------------------------------------------------------------------ */

typedef struct {
    UW seq;        /* sequence number, mutex-guarded */
    INT made_on;   /* processor that produced it */
    UW payload;    /* something to check on arrival */
} RECORD;

/* Kernel object ids, filled in by usermain(). */
LOCAL ID mpfid;   /* fixed-size memory pool */
LOCAL ID mtxid;   /* guards next_seq */
LOCAL ID mbfid;   /* producer -> consumer */
LOCAL ID semid;   /* consumer -> producer (credits) */
LOCAL ID flgid;   /* both -> monitor */

/* Shared state. next_seq is guarded by the mutex; the counters below are
   written by one task each and only read by the monitor, so they need no
   lock of their own. */
LOCAL UW next_seq;
LOCAL UW produced, consumed, dropped;
LOCAL INT last_made_on, last_seen_on;

/* Backing store for the pool and the message buffer. Static, because a
   kernel object must not own memory that can go out of scope. */
LOCAL UW mpf_buf[(N_RECORDS * sizeof(RECORD) + sizeof(UW) - 1) / sizeof(UW)
                 + N_RECORDS];
LOCAL UW mbf_buf[(CREDITS * sizeof(RECORD)) / sizeof(UW) + CREDITS * 4];

/* ------------------------------------------------------------------ *
 * Producer -- pinned to processor 1 under SMP
 * ------------------------------------------------------------------ */

LOCAL void producer_task(INT stacd, void *exinf)
{
    RECORD *rec;
    RECORD msg;
    ER er;

    (void)stacd; (void)exinf;

    while(1) {
        /* Wait for a credit: the consumer returns one per message, so
           the producer can never outrun it by more than CREDITS. */
        er = tk_wai_sem(semid, 1, 1000);
        if(er < E_OK) { dropped++; continue; }

        /* A block from the fixed-size pool, rather than a local, to
           show tk_get_mpf/tk_rel_mpf round-tripping. */
        er = tk_get_mpf(mpfid, (void **)&rec, 100);
        if(er < E_OK) { dropped++; tk_sig_sem(semid, 1); continue; }

        /* The sequence counter is the one piece of state two tasks
           could race on, so it is the one thing under a mutex. */
        tk_loc_mtx(mtxid, TMO_FEVR);
        rec->seq = ++next_seq;
        tk_unl_mtx(mtxid);

        rec->made_on = THIS_PRC();
        rec->payload = rec->seq * 7U;

        msg = *rec;   /* copy out before releasing */
        tk_rel_mpf(mpfid, rec);

        er = tk_snd_mbf(mbfid, &msg, (INT)sizeof msg, 1000);
        if(er < E_OK) { dropped++; tk_sig_sem(semid, 1); continue; }

        produced++;
        last_made_on = msg.made_on;
        tk_set_flg(flgid, FLG_PRODUCED);

        tk_dly_tsk(PRODUCE_MS);
    }
}

/* ------------------------------------------------------------------ *
 * Consumer -- pinned to processor 2 under SMP
 * ------------------------------------------------------------------ */

LOCAL void consumer_task(INT stacd, void *exinf)
{
    RECORD msg;
    INT sz;

    (void)stacd; (void)exinf;

    while(1) {
        sz = tk_rcv_mbf(mbfid, &msg, TMO_FEVR);
        if(sz != (INT)sizeof msg) { dropped++; continue; }

        /* Cheap integrity check: the payload is a known function of
           the sequence number, so a torn or reordered message shows
           up immediately. */
        if(msg.payload != msg.seq * 7U) { dropped++; continue; }

        consumed++;
        last_seen_on = THIS_PRC();
        tk_set_flg(flgid, FLG_CONSUMED);

        /* Return the credit. */
        tk_sig_sem(semid, 1);
    }
}

/* ------------------------------------------------------------------ *
 * Monitor -- woken by the event flag, prints a line
 * ------------------------------------------------------------------ */

LOCAL void monitor_task(INT stacd, void *exinf)
{
    UINT ptn;
    ER er;
    INT i;

    (void)stacd; (void)exinf;

    /* On a USB-CDC build the console is a 4 KB ring drained by the host.
       Anything printed before the host enumerates goes into a ring nobody
       is reading and is lost, so wait for the link first. Bounded at ~20 s
       so a headless board still runs; on a UART build tm_usb_state() is a
       constant and this falls straight through. */
    for(i = 0; i < 200 && tm_usb_state() < 2; i++) {
        tk_dly_tsk(100);
    }
    tk_dly_tsk(500);

    tm_printf((UB *)"\n=== uT-Kernel 3.0 / RP2040 demo ===\n");
#if TK_SUPPORT_SMP
    tm_printf((UB *)"SMP build: %d processors\n", TK_MAX_CORE);
#else
    tm_printf((UB *)"single-core build\n");
#endif
    tm_printf((UB *)"producer -> [mbf] -> consumer, %d credits, %d ms period\n\n",
              CREDITS, PRODUCE_MS);

    while(1) {
        /* Wait until both ends have moved at least once since the
           last report. TWF_ANDW = wait for every bit; the flag is
           cleared as part of the wait so the next round starts
           clean. */
        er = tk_wai_flg(flgid, FLG_PRODUCED | FLG_CONSUMED,
                        TWF_ANDW | TWF_CLR, &ptn, REPORT_MS);

        if(er == E_TMOUT) {
            tm_printf((UB *)"[monitor] stalled: produced=%u consumed=%u\n",
                      produced, consumed);
            continue;
        }

        tm_printf((UB *)"[monitor] seq=%u produced=%u consumed=%u dropped=%u"
                  " made_on=prc%d seen_on=prc%d\n",
                  next_seq, produced, consumed, dropped,
                  last_made_on, last_seen_on);

        /* One line every REPORT_MS cannot overrun a 4 KB ring, but say
           so out loud if the console ever does drop bytes. */
        if(tm_usb_dropped_bytes() != 0) {
            tm_printf((UB *)"[monitor] console dropped %u bytes\n",
                      tm_usb_dropped_bytes());
        }

        tk_dly_tsk(REPORT_MS);
    }
}

#endif /* APP_DEMO_PIPELINE */

#if TM_WIFI_CYW43
/* ------------------------------------------------------------------ *
 * WiFi status -- only built when WIFI=cyw43
 *
 * The radio is not driven from here. A dedicated service task owns the
 * CYW43439, lwIP and the echo session -- pinned to processor 1 under SMP,
 * and simply the only processor there is on a single-core build. This task
 * only reads the snapshot that service publishes and prints what changed.
 * Reading is safe from any task: cyw43_utk_get_status() takes a seqlock
 * copy, which is why the application goes through it rather than calling
 * lwip_utk_udp_get_status() directly.
 * ------------------------------------------------------------------ */

#define WIFI_IDLE_MS 3000  /* nothing happening: report rarely */
#define WIFI_BUSY_MS 200   /* session running: drain the echo ring */

/* Compose, then emit. tm_printf is atomic per call, but a line built from
   several calls can still be spliced by a higher-priority task printing
   between them -- so every line below is formatted into a local buffer and
   emitted with exactly one call. */
LOCAL void fmt_ipv4(UB *dst, const UB *a)
{
    tm_sprintf(dst, (UB *)"%u.%u.%u.%u", a[0], a[1], a[2], a[3]);
}

LOCAL void wifi_status_task(INT stacd, void *exinf)
{
    T_CYW43_UTK_STATUS st;
    UW last_link = 0xffffffffU;
    UW cursor = 0;   /* echoes printed so far */
    UB ip[16];
    UW last_dhcp = 0xffffffffU;
    UW stalled_polls = 0;
    UW shown_session = 0xffffffffU;
    UW summarised = 0xffffffffU;
    INT i;

    (void)stacd; (void)exinf;

    /* Same enumeration wait the monitor uses. Without it this task races
       ahead of the banner and its first lines land before the demo has
       introduced itself. */
    for(i = 0; i < 200 && tm_usb_state() < 2; i++) {
        tk_dly_tsk(100);
    }
    tk_dly_tsk(700);   /* let the monitor print its banner first */

    while(1) {
        cyw43_utk_get_status(&st);

        /* Link transitions, printed once each rather than every tick. */
        if(st.link_up != last_link) {
            last_link = st.link_up;
            if(st.link_up) {
                tm_printf((UB *)"[wifi] link UP rssi=%d mac=%02x:%02x:%02x:%02x:%02x:%02x\n",
                          st.link_rssi, st.mac[0], st.mac[1],
                          st.mac[2], st.mac[3], st.mac[4], st.mac[5]);
                /* The address is reported by its own transition
                   below -- DHCP usually finishes after this. */
            } else {
                tm_printf((UB *)"[wifi] link DOWN\n");
            }
        }

        /* DHCP completes some time after the link comes up, so report it
           on its own transition. Without this the address is only ever
           printed in the rare case it lands in the same poll as link-up,
           and a run that never gets an address looks like silence. */
        if(st.dhcp_complete != last_dhcp) {
            last_dhcp = st.dhcp_complete;
            if(st.dhcp_complete) {
                UB gw[16], dns[16];
                fmt_ipv4(ip, st.dhcp_address);
                fmt_ipv4(gw, st.dhcp_gateway);
                fmt_ipv4(dns, st.dhcp_dns);
                tm_printf((UB *)"[wifi] ip=%s gw=%s dns=%s\n",
                          ip, gw, dns);
            }
        }

        if(!st.udp_enabled) {
            tk_dly_tsk(WIFI_IDLE_MS);
            continue;
        }

        /* Nothing sent yet? Say what is still missing rather than going
           quiet. The session cannot start until the interface is up and
           has an address. */
        if(st.udp_packets_sent == 0 && st.udp_write_seq == 0) {
            if(++stalled_polls == 25U) {   /* ~5 s at the busy rate */
                tm_printf((UB *)"[udp] waiting: link=%u netif_up=%u"
                          " link_up=%u dhcp=%u addr=%u.%u.%u.%u\n",
                          st.link_up, st.netif_up, st.netif_link_up,
                          st.dhcp_complete, st.dhcp_address[0],
                          st.dhcp_address[1], st.dhcp_address[2],
                          st.dhcp_address[3]);
                stalled_polls = 0;
            }
        } else {
            stalled_polls = 0;
        }

        /* Announce the session once, when the first packet goes out.
           Before that the target is still 0.0.0.0:0 and printing it is
           noise, not information. */
        if(st.udp_packets_sent != 0 && st.udp_session_count != shown_session) {
            shown_session = st.udp_session_count;
            fmt_ipv4(ip, st.udp_target);
            tm_printf((UB *)"[udp] session %u: echoing to %s:%u,"
                      " %u packets of %u bytes\n",
                      st.udp_session_count + 1U, ip, st.udp_port,
                      st.udp_expected_packets, st.udp_payload_size);
        }

        /* Drain the ring: print every echo that arrived since last look.
           If the reader ever falls more than LWIP_UTK_UDP_RING behind,
           skip forward and say so rather than printing stale slots. */
        if(st.udp_write_seq > cursor) {
            if(st.udp_write_seq - cursor > CYW43_UDP_RING) {
                tm_printf((UB *)"[udp] (%u echoes not shown, reader behind)\n",
                          st.udp_write_seq - cursor - CYW43_UDP_RING);
                cursor = st.udp_write_seq - CYW43_UDP_RING;
            }
            while(cursor < st.udp_write_seq) {
                UW slot = cursor % CYW43_UDP_RING;
                UB line[96];
                INT n;

                n = tm_sprintf(line, (UB *)"[udp] echo #%u, %u bytes:",
                               st.udp_ring_seq[slot],
                               st.udp_ring_len[slot]);
                for(i = 0; i < 12 && i < (INT)st.udp_ring_len[slot]; i++) {
                    n += tm_sprintf(&line[n], (UB *)" %02x",
                                    st.udp_ring_payload[slot][i]);
                }
                if(st.udp_ring_len[slot] > 12) {
                    n += tm_sprintf(&line[n], (UB *)" ...");
                }
                tm_printf((UB *)"%s\n", line);
                cursor++;
            }
        }

        if(st.udp_complete && st.udp_session_count != summarised) {
            summarised = st.udp_session_count;
            fmt_ipv4(ip, st.udp_target);
            tm_printf((UB *)"[udp] %s:%u sent=%u recv=%u matched=%u/%u"
                      " corrupt=%u retries=%u errs=%u %u ms\n",
                      ip, st.udp_port, st.udp_packets_sent,
                      st.udp_packets_received, st.udp_packets_matched,
                      st.udp_expected_packets, st.udp_corrupt,
                      st.udp_retries, st.udp_send_errors,
                      st.udp_elapsed_ms);
            tm_printf((UB *)"[udp] session %u complete: result=%d%s\n",
                      st.udp_session_count + 1U, st.udp_result,
                      (st.udp_packets_matched == st.udp_expected_packets
                       && st.udp_corrupt == 0)
                      ? " ALL PACKETS ECHOED" : "");
        }

        /* Poll fast whenever a session could be running, not only once
           one has been observed running. A session lasts about 1.4 s
           and repeats every 5 s; at the idle rate this task would sleep
           straight through one and find the ring already wrapped. Once
           the link is up and UDP is enabled, stay at the fast rate. */
        tk_dly_tsk((st.udp_enabled && st.link_up)
                   ? WIFI_BUSY_MS : WIFI_IDLE_MS);
    }
}

LOCAL T_CTSK ctsk_wifi = {
    .itskpri = 8,
    .stksz = STACK_SZ,
    .task = wifi_status_task,
    .tskatr = TA_HLNG | TA_RNG3,
};
#endif /* TM_WIFI_CYW43 */

/* ------------------------------------------------------------------ *
 * Task and object definitions
 * ------------------------------------------------------------------ */

#if APP_DEMO_PIPELINE
LOCAL T_CTSK ctsk_producer = {
    .itskpri = 5,
    .stksz = STACK_SZ,
    .task = producer_task,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

LOCAL T_CTSK ctsk_consumer = {
    .itskpri = 6,
    .stksz = STACK_SZ,
    .task = consumer_task,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC,
    .assprc = TP_PRC2,
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

LOCAL T_CTSK ctsk_monitor = {
    .itskpri = 7,
    .stksz = STACK_SZ,
    .task = monitor_task,
    .tskatr = TA_HLNG | TA_RNG3,
};

#endif /* APP_DEMO_PIPELINE */

LOCAL T_CTSK ctsk_blink = {
    .itskpri = 10,
    .stksz = STACK_SZ,
    .task = blink_task,
    .tskatr = TA_HLNG | TA_RNG3,
};

#if APP_DEMO_PIPELINE
LOCAL T_CMPF cmpf = {
    .mpfatr = TA_TFIFO | TA_RNG3,
    .mpfcnt = N_RECORDS,
    .blfsz = sizeof(RECORD),
    .bufptr = mpf_buf,
};

LOCAL T_CMTX cmtx = {
    .mtxatr = TA_TFIFO | TA_INHERIT,
};

LOCAL T_CMBF cmbf = {
    .mbfatr = TA_TFIFO,
    .bufsz = sizeof mbf_buf,
    .maxmsz = sizeof(RECORD),
    .bufptr = mbf_buf,
};

LOCAL T_CSEM csem = {
    .sematr = TA_TFIFO,
    .isemcnt = CREDITS,
    .maxsem = CREDITS,
};

LOCAL T_CFLG cflg = {
    .flgatr = TA_TFIFO | TA_WMUL,
    .iflgptn = 0,
};
#endif /* APP_DEMO_PIPELINE */

/* ------------------------------------------------------------------ *
 * Robot motion tests (Buddy 2)
 *
 * MOTION_TEST_MODE picks which test usermain() runs:
 *   MOTION_TEST_FIXED_TIME - encoder diagnostics: 1 s full-speed runs,
 *                            5 with wheels lifted, then 5 on the floor.
 *   MOTION_TEST_DISTANCE   - floor: 50 cm and 30 cm at 300 mm/s, then 50 cm
 *                            at 500 mm/s under PI, 10 s apart to measure.
 *   MOTION_TEST_HAND       - motors stay OFF; prints encoder counters
 *                            every second while the wheels are turned
 *                            by hand. Ground truth with no motor noise.
 *   MOTION_TEST_DUTY_SWEEP - wheels LIFTED; steps PWM duty 10..100 %
 *                            forward then reverse and prints each
 *                            wheel's steady speed. Open-loop motor
 *                            map for PID feed-forward and dead band.
 *   MOTION_TEST_PI_STEP    - closed-loop step response: 300, 500, 150,
 *                            0 mm/s forward, then -300 and 0. Logged in
 *                            RAM and printed afterwards as [TRACE] CSV
 *                            plus a [PI] summary per step.
 *   IMU_TEST_BRINGUP       - Buddy 4 step 1: I2C1 bus scan on GP2/GP3,
 *                            LSM303D WHO_AM_I, then 5 s of raw accel
 *                            (mg) and mag (counts). Motors stay off.
 *   IMU_TEST_ACCEL_6FACE   - Buddy 4 step 3: hold the car still in six
 *                            orientations (prompted); averages each and
 *                            prints per-axis offset and scale.
 *   IMU_TEST_MAG_CAL       - Buddy 4 step 4: mag circle turned by hand
 *                            (motors off), then spinning in place under
 *                            PI (motors on); prints centre/radius of each.
 *   IMU_TEST_HEADING       - Buddy 4 step 5: heading at 4 hand-set 90 deg
 *                            positions, then a clockwise spin on the floor
 *                            comparing compass and encoder rotation.
 *   IMU_TEST_HUMP_LOG      - Buddy 4 step 6: four 60 cm floor passes
 *                            (flat, 1 cm, 2.4 cm at 200 mm/s, 2.4 cm at 100) logging pitch,
 *                            roll, vertical accel and distance.
 *   IMU_TEST_TERRAIN       - Buddy 4 step 7: the real-time hump and
 *                            collision detector over flat, small, tall.
 *   ULTRA_TEST_RANGE       - Buddy 5 step 1: HC-SR04 at 10/20/30/50/100 cm,
 *                            mean/spread/misses per point, linear fit.
 *   SERVO_TEST_JOG         - Buddy 5 step 2: jog the scan servo with keys
 *                            and mark centre and the safe limits.
 *   SERVO_TEST_MAP         - Buddy 5 step 3: sweep a narrow target, turn the
 *                            car a measured angle, sweep again: us/deg,
 *                            window in degrees and beam width.
 *   ULTRA_TEST_BEAM        - Buddy 5 step 3b: servo fixed, car turns +/-45
 *                            deg past a narrow target at 20/35/50 cm.
 *   ULTRA_TEST_SCAN        - Buddy 5 step 4: coarse/fine obstacle scan
 *                            (scan.c), profile and pass-left/right advice.
 *   DEMO_ROAM              - Buddies 2+4+5 together: drive at 150 mm/s,
 *                            stop at an obstacle within 20 cm, turn 45 deg
 *                            left on the spot (compass checks the turn),
 *                            repeat until clear. Any key pauses/resumes.
 *   IR_TEST_LOG            - Buddy 3 step 1: live IR readout, then hand-
 *                            pushed passes printing black/white run widths
 *                            per sensor (encoder distance, motors off).
 *   IR_TEST_AO             - Buddy 3 step 1b: left IR analog out (GP27)
 *                            against distance, hand-pushed over a barcode:
 *                            levels, bar widths and the raw profile.
 *   MOTION_TEST_SAFETY     - Tribunal 1 phase 1, car lifted: loop rate and
 *                            PI error, STOP over queued commands, stall
 *                            (both wheels held), encoder check (one held).
 *   MOTION_TEST_CPU        - motion task CPU (busy us/s, longest pass) idle
 *                            and at 150/300/500 mm/s, and stack high-water.
 *   IMU_TEST_RAMP_ROLL     - phase 2 step 1: 50 cm at 150 mm/s with only the
 *                            right wheel over a ramp (flat/small/tall); logs
 *                            roll, pitch, heading, wheel distances, jolt.
 *   MOTION_TEST_STRAIGHT   - key-paced 50 cm straight runs at 150 mm/s to
 *                            measure sideways drift (per-wheel trim).
 *   IMU_TEST_SIDE_HUMP     - phase 2: roll-based one-sided hump detector
 *                            live (hump mask on), 5 tall-ramp runs, with
 *                            the bracket-strike IMPACT check replayed.
 *   OBST_TEST_STOP         - phase 3 step 1: obstacle watch task; runs at
 *                            300 mm/s to a box (5) and a bottle (2), stop
 *                            gap and a per-ping trace.
 *   DEMO_STOP              - obstacle demo v1 (course): drive, stop at the
 *                            block (>= 5 cm nose gap), stay stopped.
 *   SCAN_TEST_ABORT        - phase 3 step 2: scan in the obstacle task; full
 *                            scan, abort with STOP mid-scan, timeout fault.
 *   DEMO_AVOID             - obstacle demo v2 (course): stop at the block,
 *                            scan, go round it, back onto the original path.
 *   LINE_TEST_FOLLOW       - Buddy 3: line following runs on the mats with
 *                            a 20 ms trace (stops on distance, raised mats).
 *   LINE_TEST_RAMP         - course scenario 1: one-sided ramp under the
 *                            right wheel while line following; hump mask
 *                            fed live, IMU log + line trace.
 *   CTRL_TEST_COURSE       - vehicle controller task: course runs (line +
 *                            ramp, or line + obstacle stop), event log and
 *                            robot_status every 500 ms.
 *   BARCODE_TEST_DRIVE     - Buddy 3 barcode: driven scan (50 mm/s) of the
 *                            slit-taped left IR AO, decoder v2 (barcode.c)
 *                            reading Code 39 both ways.
 *   OBST_TEST_WIDTH        - Buddy 5: drive, stop at an object, scan; width
 *                            estimate plus every reading, for calibrating
 *                            the width against objects of known size.
 * To switch, change the MOTION_TEST_MODE default below and rebuild.
 * ------------------------------------------------------------------ */
#define MOTION_TEST_FIXED_TIME   1
#define MOTION_TEST_DISTANCE     2
#define MOTION_TEST_HAND         3
#define MOTION_TEST_DUTY_SWEEP   4
#define MOTION_TEST_PI_STEP      5
#define IMU_TEST_BRINGUP         6
#define IMU_TEST_ACCEL_6FACE     7
#define IMU_TEST_MAG_CAL         8
#define IMU_TEST_HEADING         9
#define IMU_TEST_HUMP_LOG        10
#define IMU_TEST_TERRAIN         11
#define ULTRA_TEST_RANGE         12
#define SERVO_TEST_JOG           13
#define SERVO_TEST_MAP           14
#define ULTRA_TEST_BEAM          15
#define ULTRA_TEST_SCAN          16
#define DEMO_ROAM                17
#define IR_TEST_LOG              18
#define IR_TEST_AO               19
#define MOTION_TEST_SAFETY       20
#define MOTION_TEST_CPU          21
#define IMU_TEST_RAMP_ROLL       22
#define MOTION_TEST_STRAIGHT     23
#define IMU_TEST_SIDE_HUMP       24
#define OBST_TEST_STOP           25
#define DEMO_STOP                26
#define SCAN_TEST_ABORT          27
#define DEMO_AVOID               28
#define LINE_TEST_FOLLOW         29
#define LINE_TEST_RAMP           30
#define CTRL_TEST_COURSE         31
#define BARCODE_TEST_DRIVE       32
#define OBST_TEST_WIDTH          33

#ifndef MOTION_TEST_MODE
#define MOTION_TEST_MODE         OBST_TEST_WIDTH
#endif

#define IMU_SAMPLE_COUNT         20
#define IMU_SAMPLE_MS            250

#define FACE_COUNT               6
#define FACE_MOVE_S              10    /* time to reposition the car */
#define FACE_SAMPLES             100   /* averaged per face */
#define FACE_SAMPLE_MS           20    /* 2 s per face */
#define FACE_DOMINANT_MG         700   /* one axis must read > 0.7 g */

#define MAG_SAMPLE_MS            20    /* mag ODR is 50 Hz */
#define MAG_HAND_MS              20000
#define MAG_SPIN_MM_S            100   /* wheel speed: ~100 deg/s on the spot */
#define MAG_SPIN_SETTLE_MS       800
#define MAG_SPIN_MS              12000 /* ~3 turns */
#define MAG_TURN_UM              358142L /* pi x 114 mm track width */

#define HDG_SAMPLE_MS            20
#define HDG_MOVE_S               8     /* time to turn the car by hand */
#define HDG_AVG_SAMPLES          50    /* 1 s average per position */
#define HDG_SPIN_MM_S            100   /* about 100 deg/s on the spot */
#define HDG_SPIN_MS              8000  /* about 2.2 turns */
#define HDG_TRACK_MM             114   /* ruler, 2026-10-07; confirmed by compass spin */

#define HUMP_LOG_LEN             400
#define HUMP_SAMPLE_MS           20
#define HUMP_DIST_MM             600
#define HUMP_SETUP_S             15    /* time to place the ramps */
#define HUMP_WHEELBASE_MM        80    /* drive axle to castor, ruler 2026-10-09 */
#define HUMP_FILT_N              7     /* moving average, ~0.16 s / ~3 cm */
#define HUMP_BASE_SAMPLES        25    /* rest tilt, 0.5 s before driving */
#define HUMP_SKIP_MM             60    /* skip the speed-up after the start */
#define HUMP_PLAT_MM             30    /* descent plateau window */
#define HUMP_PASSES              4

#define TERR_SPEED_MM_S          200   /* validated speed for hump height */
#define TERR_DIST_MM             600
#define TERR_MAX_EVENTS          8

#define US_POINTS                6
#define US_SAMPLES               30
#define US_PERIOD_MS             70    /* datasheet: >= 60 ms between pings */

#define SERVO_JOG_START_US       1500  /* nominal servo neutral */
#define SERVO_JOG_SMALL_US       10
#define SERVO_JOG_BIG_US         50

#define MAP_STEP_US              10
#define MAP_SETTLE_MS            120
#define MAP_TARGET_CM            25
#define MAP_NEAR_CM              40    /* closer than this = the target */
#define MAP_TURN_DEG             15
#define MAP_TURN_MM_S            50
#define MAP_TURN_BRAKE_EDGES     8

#define BEAM_LOG_LEN             120
#define BEAM_HALF_DEG            45
#define BEAM_TURN_MM_S           60
#define BEAM_SCAN_MM_S           25    /* ~25 deg/s: ~2 deg per ping */
#define BEAM_BRAKE_CDEG          150

#define SCAN_HALF_DEG            45
#define SCAN_DETECT_MM           600
#define SCAN_RUNS                3
#define SCAN_CAR_HALF_MM         70    /* half track + half a wheel */

#define ROAM_SPEED_MM_S          150
#define ROAM_STOP_MM             200   /* raw sensor reading; see roam notes */
#define ROAM_CONFIRM             2     /* consecutive near pings to stop */
#define ROAM_PING_EVERY          3     /* ~21+ ms loop periods per ping (>= 60 ms) */
#define ROAM_ECHO_MAX_US         5830  /* ~1 m: no-echo pings end early */
#define ROAM_TURN_CDEG           4500
#define ROAM_TURN_MM_S           60    /* ~60 deg/s on the spot */
#define ROAM_TURN_BRAKE_CDEG     150
#define ROAM_BACK_MM_S           100
#define ROAM_BACK_MS             500   /* ~5 cm back after a bump */
#define ROAM_START_S             5
#define ROAM_SPD_WIN_MS          100   /* wheel speed window */
#define ROAM_SPD_RING            8     /* > ROAM_SPD_WIN_MS / loop period */
#define ROAM_SPD_SETTLE_MS       400   /* skip speed checks while starting */
#define ROAM_JOLT_CONFIRM_MS     250   /* a jolt must slow the wheels within this */
#define ROAM_JOLT_SLOW_PCT       60    /* ...to below this % of the command */
#define ROAM_IGNORED_MAX         4

#define IR_LOG_LEN               400   /* sensor changes per pass */
#define IR_SHORT_MM10            10    /* runs under 1 mm count as chatter */
#define IR_AO_LEN                900   /* steps per pass: ~295 mm */
#define IR_AO_ROW                20    /* profile values per line (1 mm each) */

#define RAMP_SPEED_MM_S          150
#define RAMP_DIST_MM             500
#define RAMP_LOG_LEN             300   /* 20 ms samples: 6 s */
#define SIDE_RUNS                5

#define STRAIGHT_RUNS            5
#define STRAIGHT_DIST_CM         50
#define STRAIGHT_SPEED_MM_S      150

#define OBST_RUNS                7
#define OBST_BOX_RUNS            5
#define OBST_MAX_DIST_MM         1500  /* drive limit if nothing stops it */

#define DEMOSTOP_MM_S            300   /* = OBS_WATCH_MAX_MM_S, validated */
#define DEMOSTOP_MAX_MM          3000  /* stop anyway after 3 m */
#define SCANAB_LONG_MS           90000u /* diagnostic: let a slow scan finish */
#define SCANAB_A_RUNS            3

/* Car footprint (user, 23 Oct): 20 cm long, axle 13 cm behind the nose
   (7 cm of car behind it); sonar ~6 cm behind the nose. Not symmetric:
   wheels 14 cm across, the left IR bracket pokes out 1-2 cm further */
#define AVOID_HALF_R_MM          70    /* centre to the right wheel's outside */
#define AVOID_HALF_L_MM          85    /* centre to the left IR bracket */
#define AVOID_SENSOR_AHEAD_MM    70    /* sonar ahead of the axle */
/* Gap beside the block: v2b (20 mm) rubbed a wheel on the straight. Each
   90 deg turn lands within ~1-2 deg (2 deg over the ~55 cm straight is
   ~20 mm) and the scan places the block's edge within ~20-30 mm; turns
   are now trimmed to ~0.6 deg, and 50 mm covers the rest */
#define AVOID_GAP_MM             50
#define AVOID_DEPTH_MM           200   /* block depth: not measurable, assumed */
/* Axle past the block's back before turning in: after the 90 deg turn the
   car's trailing side is a half width behind the axle and must clear the
   block's back face on the sideways leg home */
#define AVOID_PAST_MM            (AVOID_HALF_L_MM + AVOID_GAP_MM)
#define AVOID_TRIM_CDEG          60    /* re-turn if a turn misses by more */
#define AVOID_AFTER_MM           400   /* carry on along the path afterwards */
#define AVOID_LEG_MM_S           200
#define AVOID_LEG_BRAKE_MM       35    /* stop command this early (~v^2/2a) */
#define AVOID_TURN_MM_S          60
#define AVOID_TRIM_MM_S          30    /* slow for the small correcting turn */

/* Raised mats: wheels start at the mat edge with the nose ~13 cm in; on
   the straight + cross mats (61 cm) stop with the wheels <= ~45 cm along */
#define LINE_TEST_MM_S           100   /* tuning speed (first runs at 150 swung hard) */
#define LINE_TEST_MAX_MM         450

#define DIST_PAUSE_S             10

#define PI_HOLD_MS               1500
#define PI_MEASURE_MS            500
#define PI_STOP_MS               1000
#define PI_MAX_STEPS             4

#define SWEEP_STEP_PCT           10
#define SWEEP_SETTLE_MS          800
#define SWEEP_MEASURE_MS         500
#define SWEEP_UM_PER_EDGE        328  /* keep in step with UM_PER_EDGE */

#define HAND_PRINT_MS            1000

#define FIXED_RUN_COUNT          5
#define FIXED_RUN_MS             1000
#define FIXED_SETTLE_MS          1000
#define FIXED_PHASE_GAP_S        15

#if MOTION_TEST_MODE == MOTION_TEST_FIXED_TIME
/* One batch of fixed-time runs. Runs alternate forward and reverse so the
   car stays roughly in place on the floor. Counters are read after the
   wheels have coasted to a stop, so every edge of the run is included. */
LOCAL void motion_fixed_time_runs(const char *phase)
{
    INT run;
    int8_t dir;
    char label[32];

    for(run = 1; run <= FIXED_RUN_COUNT; run++) {
        dir = ((run % 2) == 1) ? 100 : -100;

        motion_reset_odometry();
        motion_set_speed(dir, dir);
        tk_dly_tsk(FIXED_RUN_MS);
        motion_set_speed(0, 0);
        tk_dly_tsk(FIXED_SETTLE_MS);

        tm_sprintf((UB *)label, (UB *)"%s run %d %s", phase, run,
                   (dir > 0) ? "fwd" : "rev");
        motion_print_encoder_diag(label);
    }
}

LOCAL void motion_test_fixed_time(void)
{
    INT s;

    tm_printf((UB *)"\n[TEST] Encoder diagnostics: %d x %d ms runs per phase\n",
              FIXED_RUN_COUNT, FIXED_RUN_MS);
    tm_printf((UB *)"[TEST] PHASE 1: wheels LIFTED off the ground\n");
    motion_fixed_time_runs("LIFTED");

    tm_printf((UB *)"\n[TEST] PHASE 2: put the car ON THE FLOOR now\n");
    for(s = FIXED_PHASE_GAP_S; s > 0; s--) {
        tm_printf((UB *)"[TEST] floor runs start in %d s\n", s);
        tk_dly_tsk(1000);
    }
    motion_fixed_time_runs("FLOOR");

    tm_printf((UB *)"\n=== Encoder diagnostics complete ===\n");
}
#elif MOTION_TEST_MODE == MOTION_TEST_DUTY_SWEEP
/* Hold each duty long enough to reach steady speed, then count edges over
   a fixed window. Edge rate x 0.322 mm/edge gives wheel speed in mm/s. */
LOCAL void motion_sweep_direction(INT sign, const char *name)
{
    INT duty;
    enc_diag_t l0, r0, l1, r1;
    UW l_rate, r_rate, l_mm_s, r_mm_s, ratio_pct;

    for(duty = SWEEP_STEP_PCT; duty <= 100; duty += SWEEP_STEP_PCT) {
        motion_set_speed((int8_t)(sign * duty), (int8_t)(sign * duty));
        tk_dly_tsk(SWEEP_SETTLE_MS);

        motion_get_encoder_diag(&l0, &r0);
        tk_dly_tsk(SWEEP_MEASURE_MS);
        motion_get_encoder_diag(&l1, &r1);

        l_rate = (l1.hw_edges - l0.hw_edges) * 1000 / SWEEP_MEASURE_MS;
        r_rate = (r1.hw_edges - r0.hw_edges) * 1000 / SWEEP_MEASURE_MS;
        l_mm_s = l_rate * SWEEP_UM_PER_EDGE / 1000;
        r_mm_s = r_rate * SWEEP_UM_PER_EDGE / 1000;
        ratio_pct = (l_rate > 0) ? (r_rate * 100 / l_rate) : 0;

        tm_printf((UB *)"[SWEEP] %s duty=%3d%%  L=%4u edges/s (%3u mm/s)"
                  "  R=%4u edges/s (%3u mm/s)  R/L=%u%%\n",
                  name, duty, l_rate, l_mm_s, r_rate, r_mm_s, ratio_pct);
    }

    motion_set_speed(0, 0);
    tk_dly_tsk(1500);
}

LOCAL void motion_test_duty_sweep(void)
{
    tm_printf((UB *)"\n[TEST] Duty sweep: keep the wheels LIFTED\n");
    tm_printf((UB *)"[TEST] Starting in 5 s\n");
    tk_dly_tsk(5000);

    motion_sweep_direction(1, "fwd");
    motion_sweep_direction(-1, "rev");

    tm_printf((UB *)"\n=== Duty sweep complete ===\n");
}
#elif MOTION_TEST_MODE == IMU_TEST_BRINGUP
/* Print the ID registers that tell common accel/mag chips apart:
     0x0F WHO_AM_I: 0x49 LSM303D, 0x41 LSM303C accel, 0x33 LSM303DLHC/AGR
                    accel, 0x3D LSM303C/LIS3MDL mag, 0x40 LSM303AGR mag
     0x00:          0xE5 ADXL345
     0x0D:          0x2A MMA8452Q, 0x1A MMA8451Q */
LOCAL void imu_identify(UB addr)
{
    static const UB regs[] = { 0x00, 0x0D, 0x0F };
    INT i;
    uint8_t v;

    for(i = 0; i < (INT)sizeof(regs); i++) {
        if(E_OK == i2c1_read_regs(addr, regs[i], &v, 1u)) {
            tm_printf((UB *)"[IMU]   0x%02x reg 0x%02x = 0x%02x\n", addr, regs[i], v);
        } else {
            tm_printf((UB *)"[IMU]   0x%02x reg 0x%02x = (no answer)\n", addr, regs[i]);
        }
    }
}

/* Buddy 4 step 1: prove the wiring and the I2C driver before any maths. */
LOCAL void imu_test_bringup(void)
{
    UB addr;
    INT found = 0;
    INT i;
    uint8_t id[1];
    imu_raw_t r;
    ER err;

    tm_printf((UB *)"\n[IMU] I2C1 scan on GP2 (SDA) / GP3 (SCL), 100 kHz\n");
    i2c1_init();

    for(addr = 0x08; addr < 0x78; addr++) {
        if(E_OK == i2c1_probe(addr)) {
            tm_printf((UB *)"[IMU] found device at 0x%02x\n", addr);
            found++;
            imu_identify(addr);
        }
    }
    tm_printf((UB *)"[IMU] %d device(s); expect 0x1d (LSM303D)\n", found);

    err = imu_who_am_i(&id[0]);
    if(E_OK == err) {
        tm_printf((UB *)"[IMU] WHO_AM_I = 0x%02x (expect 0x49 = LSM303D)\n", id[0]);
    } else {
        tm_printf((UB *)"[IMU] WHO_AM_I read failed: %d\n", err);
    }

    err = imu_init();
    if(E_OK != err) {
        tm_printf((UB *)"[IMU] imu_init failed: %d\n", err);
        return;
    }
    tk_dly_tsk(100);

    tm_printf((UB *)"[IMU] raw samples: accel in mg, mag in counts. Keep the car flat and still.\n");
    for(i = 0; i < IMU_SAMPLE_COUNT; i++) {
        err = imu_read_raw(&r);
        if(E_OK == err) {
            tm_printf((UB *)"[IMU] a=(%5d,%5d,%5d) mg  m=(%5d,%5d,%5d)\n",
                      r.ax, r.ay, r.az, r.mx, r.my, r.mz);
        } else {
            tm_printf((UB *)"[IMU] read failed: %d\n", err);
        }
        tk_dly_tsk(IMU_SAMPLE_MS);
    }

    tm_printf((UB *)"\n=== IMU bring-up complete ===\n");
}
#elif MOTION_TEST_MODE == IMU_TEST_ACCEL_6FACE
LOCAL INT iabs(INT v)
{
    return (v < 0) ? -v : v;
}

LOCAL UW isqrt_u32(UW v)
{
    UW r = 0;
    UW bit = 1UL << 30;

    while(bit > v) {
        bit >>= 2;
    }
    while(bit != 0) {
        if(v >= r + bit) {
            v -= r + bit;
            r = (r >> 1) + bit;
        } else {
            r >>= 1;
        }
        bit >>= 2;
    }
    return r;
}

/* Buddy 4 step 3. Each face should put +1 g or -1 g on exactly one axis.
   The axis and sign are detected from the data, so the order and the way
   the board is mounted do not matter. For each axis:
     offset = (+1 g reading + -1 g reading) / 2
     scale  = 1000 / ((+1 g reading - -1 g reading) / 2)
   so that corrected = (raw - offset) * scale reads exactly +/-1000 mg. */
LOCAL void imu_test_accel_6face(void)
{
    static const char *const faces[FACE_COUNT] = {
        "UPRIGHT (wheels on the table)",
        "UPSIDE DOWN (wheels in the air)",
        "NOSE UP (front of the car pointing at the ceiling)",
        "NOSE DOWN (front of the car pointing at the table)",
        "LEFT SIDE DOWN",
        "RIGHT SIDE DOWN",
    };
    static const char axis_name[3] = { 'X', 'Y', 'Z' };
    static INT avg[FACE_COUNT][3];
    INT pos[3] = { 0, 0, 0 };
    INT neg[3] = { 0, 0, 0 };
    BOOL have_pos[3], have_neg[3];
    INT f, s, a, n_ok, best, off, half;
    W sum[3];
    imu_raw_t r;

    i2c1_init();
    if(E_OK != imu_init()) {
        tm_printf((UB *)"[CAL] imu_init failed, check the IMU\n");
        return;
    }

    tm_printf((UB *)"\n[CAL] Accelerometer 6-face calibration. Motors stay off.\n");
    tm_printf((UB *)"[CAL] For each face: move the car when asked, then keep it\n");
    tm_printf((UB *)"[CAL] completely still (hands off) while it says HOLD.\n");

    for(a = 0; a < 3; a++) {
        have_pos[a] = FALSE;
        have_neg[a] = FALSE;
    }

    for(f = 0; f < FACE_COUNT; f++) {
        tm_printf((UB *)"\n[CAL] Face %d/6: put the car %s\n", f + 1, faces[f]);
        for(s = FACE_MOVE_S; s > 0; s--) {
            tm_printf((UB *)"[CAL]   measuring in %d s\n", s);
            tk_dly_tsk(1000);
        }
        tm_printf((UB *)"[CAL]   HOLD STILL...\n");

        sum[0] = sum[1] = sum[2] = 0;
        n_ok = 0;
        for(s = 0; s < FACE_SAMPLES; s++) {
            if(E_OK == imu_read_raw(&r)) {
                sum[0] += r.ax;
                sum[1] += r.ay;
                sum[2] += r.az;
                n_ok++;
            }
            tk_dly_tsk(FACE_SAMPLE_MS);
        }
        if(0 == n_ok) {
            tm_printf((UB *)"[CAL]   no samples read, skipping face\n");
            continue;
        }

        for(a = 0; a < 3; a++) {
            avg[f][a] = (INT)(sum[a] / n_ok);
        }

        best = 0;
        for(a = 1; a < 3; a++) {
            if(iabs(avg[f][a]) > iabs(avg[f][best])) {
                best = a;
            }
        }

        tm_printf((UB *)"[CAL]   avg=(%5d,%5d,%5d) mg  |a|=%u mg  -> %c%c\n",
                  avg[f][0], avg[f][1], avg[f][2],
                  isqrt_u32((UW)(avg[f][0] * avg[f][0] + avg[f][1] * avg[f][1]
                                 + avg[f][2] * avg[f][2])),
                  (avg[f][best] >= 0) ? '+' : '-', axis_name[best]);

        if(iabs(avg[f][best]) < FACE_DOMINANT_MG) {
            tm_printf((UB *)"[CAL]   WARNING: no axis above %d mg, car was not square"
                      " on this face; result not used\n", FACE_DOMINANT_MG);
        } else if(avg[f][best] > 0) {
            pos[best] = avg[f][best];
            have_pos[best] = TRUE;
        } else {
            neg[best] = avg[f][best];
            have_neg[best] = TRUE;
        }
    }

    tm_printf((UB *)"\n[CAL] Result (corrected = (raw - offset) * scale):\n");
    for(a = 0; a < 3; a++) {
        if(have_pos[a] && have_neg[a]) {
            off = (pos[a] + neg[a]) / 2;
            half = (pos[a] - neg[a]) / 2;
            tm_printf((UB *)"[CAL] %c: +1g=%5d  -1g=%5d  offset=%4d mg  scale=%d/1000\n",
                      axis_name[a], pos[a], neg[a], off,
                      (half > 0) ? (1000 * 1000 / half) : 0);
        } else {
            tm_printf((UB *)"[CAL] %c: missing %s face, cannot calibrate\n",
                      axis_name[a], have_pos[a] ? "-1g" : "+1g");
        }
    }

    tm_printf((UB *)"\n=== Accel 6-face calibration complete ===\n");
}
#elif MOTION_TEST_MODE == IMU_TEST_MAG_CAL
/* Buddy 4 step 4: magnetometer hard-iron / soft-iron check.

   Turning the car through 360 degrees on a flat floor traces a circle in
   the mag X/Y readings. Its centre is the hard-iron offset (fields fixed to
   the car: motor magnets, steel screws, the battery); the X/Y radii
   differing is soft-iron distortion. Phase A turns the car by hand with the
   motors off; phase B spins it in place under PI with the motors running.
   If the two centres differ, the motor current itself bends the field and
   heading must only be trusted with the motors in a known state. */
typedef struct {
    INT min[3];
    INT max[3];
    INT n;
} mag_range_t;

LOCAL void mag_range_reset(mag_range_t *p)
{
    INT a;

    for(a = 0; a < 3; a++) {
        p->min[a] = 32767;
        p->max[a] = -32768;
    }
    p->n = 0;
}

/* Sample every MAG_SAMPLE_MS for dur_ms. No printing: phase B runs this
   while the motors are under PI, and usermain outranks the motion task. */
LOCAL void mag_collect(mag_range_t *p, INT dur_ms)
{
    INT t;
    INT a;
    INT v[3];
    imu_raw_t s;

    for(t = 0; t < dur_ms; t += MAG_SAMPLE_MS) {
        if(E_OK == imu_read_cal(&s)) {
            v[0] = s.mx;
            v[1] = s.my;
            v[2] = s.mz;
            for(a = 0; a < 3; a++) {
                if(v[a] < p->min[a]) p->min[a] = v[a];
                if(v[a] > p->max[a]) p->max[a] = v[a];
            }
            p->n++;
        }
        tk_dly_tsk(MAG_SAMPLE_MS);
    }
}

LOCAL void mag_report(const char *name, const mag_range_t *p)
{
    static const char axis_name[3] = { 'X', 'Y', 'Z' };
    INT a;
    INT rx, ry;

    tm_printf((UB *)"[MAG] %s: %d samples\n", name, p->n);
    for(a = 0; a < 3; a++) {
        tm_printf((UB *)"[MAG]   %c: min=%6d max=%6d  centre=%6d  radius=%5d\n",
                  axis_name[a], p->min[a], p->max[a],
                  (p->max[a] + p->min[a]) / 2, (p->max[a] - p->min[a]) / 2);
    }
    rx = (p->max[0] - p->min[0]) / 2;
    ry = (p->max[1] - p->min[1]) / 2;
    tm_printf((UB *)"[MAG]   X/Y radius ratio = %d%% (100%% = no soft-iron distortion)\n",
              (ry > 0) ? (rx * 100 / ry) : 0);
}

LOCAL void imu_test_mag_cal(void)
{
    static mag_range_t hand;
    static mag_range_t spin;
    enc_diag_t l, r;
    imu_raw_t s;
    W sum[3];
    INT i, n_ok, dcx, dcy, rad, avg_edges, turns_x100;

    i2c1_init();
    if(E_OK != imu_init()) {
        tm_printf((UB *)"[MAG] imu_init failed, check the IMU\n");
        return;
    }

    /* Check the accel calibration from step 3 */
    tk_dly_tsk(200);
    sum[0] = sum[1] = sum[2] = 0;
    n_ok = 0;
    for(i = 0; i < 25; i++) {
        if(E_OK == imu_read_cal(&s)) {
            sum[0] += s.ax;
            sum[1] += s.ay;
            sum[2] += s.az;
            n_ok++;
        }
        tk_dly_tsk(20);
    }
    if(n_ok > 0) {
        tm_printf((UB *)"\n[MAG] calibrated accel at rest = (%d, %d, %d) mg"
                  " (expect about 0, 0, -1000 upright)\n",
                  (INT)(sum[0] / n_ok), (INT)(sum[1] / n_ok), (INT)(sum[2] / n_ok));
    }

    /* Phase A: by hand, motors off */
    mag_range_reset(&hand);
    tm_printf((UB *)"\n[MAG] Phase A (motors OFF): keep the car flat on the table and\n");
    tm_printf((UB *)"[MAG] turn it slowly by hand through 2 full turns in %d s.\n",
              MAG_HAND_MS / 1000);
    for(i = 5; i > 0; i--) {
        tm_printf((UB *)"[MAG]   start turning in %d s\n", i);
        tk_dly_tsk(1000);
    }
    tm_printf((UB *)"[MAG]   TURN NOW...\n");
    mag_collect(&hand, MAG_HAND_MS);
    tm_printf((UB *)"[MAG]   stop\n\n");
    mag_report("Phase A, by hand, motors off", &hand);

    /* Phase B: spin in place under PI, motors on */
    tm_printf((UB *)"\n[MAG] Phase B (motors ON): put the car on the FLOOR with\n");
    tm_printf((UB *)"[MAG] about 30 cm clear all round. It will spin on the spot.\n");
    for(i = 10; i > 0; i--) {
        tm_printf((UB *)"[MAG]   spinning in %d s\n", i);
        tk_dly_tsk(1000);
    }

    mag_range_reset(&spin);
    motion_reset_odometry();
    motion_set_velocity(MAG_SPIN_MM_S, -MAG_SPIN_MM_S);
    tk_dly_tsk(MAG_SPIN_SETTLE_MS);         /* skip the speed-up */
    mag_collect(&spin, MAG_SPIN_MS);
    motion_set_velocity(0, 0);
    tk_dly_tsk(1500);

    (void)motion_get_encoder_diag(&l, &r);
    avg_edges = (INT)((((l.hw_count < 0) ? -l.hw_count : l.hw_count)
                     + ((r.hw_count < 0) ? -r.hw_count : r.hw_count)) / 2);
    /* one turn on the spot = pi * track width of wheel travel per wheel */
    turns_x100 = avg_edges * SWEEP_UM_PER_EDGE / (MAG_TURN_UM / 100);
    tm_printf((UB *)"\n[MAG] encoders: L=%d R=%d edges -> about %d.%02d turns\n",
              (INT)l.hw_count, (INT)r.hw_count, turns_x100 / 100, turns_x100 % 100);
    mag_report("Phase B, spinning, motors on", &spin);

    /* Compare the circle centres. A shift of d counts on a circle of radius
       R moves the computed heading by up to about 57 * d / R degrees. */
    dcx = (spin.max[0] + spin.min[0]) / 2 - (hand.max[0] + hand.min[0]) / 2;
    dcy = (spin.max[1] + spin.min[1]) / 2 - (hand.max[1] + hand.min[1]) / 2;
    rad = (hand.max[0] - hand.min[0] + hand.max[1] - hand.min[1]) / 4;
    tm_printf((UB *)"\n[MAG] centre shift motors on vs off: dX=%d dY=%d counts\n", dcx, dcy);
    if(rad > 0) {
        tm_printf((UB *)"[MAG] -> up to about %d degrees of heading error if ignored\n",
                  (((dcx < 0) ? -dcx : dcx) + ((dcy < 0) ? -dcy : dcy)) * 57 / rad);
    }

    tm_printf((UB *)"\n=== Mag calibration test complete ===\n");
}
#elif MOTION_TEST_MODE == IMU_TEST_HEADING
/* Buddy 4 step 5: heading and turn rate from the calibrated compass.
   Phase 1 checks heading against known 90 degree turns made by hand.
   Phase 2 spins the car on the floor and compares the compass rotation
   with the rotation the wheel encoders imply for an 11.4 cm track. */

/* Wrap a heading difference into -18000..17999 hundredths of a degree */
LOCAL INT wrap_cdeg(INT d)
{
    while(d >= 18000) {
        d -= 36000;
    }
    while(d < -18000) {
        d += 36000;
    }
    return d;
}

/* Average n headings, unwrapped around the first so 359/1 deg average to 0 */
LOCAL INT heading_avg_cdeg(INT n)
{
    imu_attitude_t att;
    INT i;
    INT ok = 0;
    INT first = -1;
    W acc = 0;
    INT h;

    for(i = 0; i < n; i++) {
        if(E_OK == imu_read_attitude(&att, NULL)) {
            if(first < 0) {
                first = (INT)att.heading_cdeg;
            }
            acc += wrap_cdeg((INT)att.heading_cdeg - first);
            ok++;
        }
        tk_dly_tsk(HDG_SAMPLE_MS);
    }
    if(0 == ok) {
        return -1;
    }

    h = first + (INT)(acc / ok);
    while(h < 0) {
        h += 36000;
    }
    while(h >= 36000) {
        h -= 36000;
    }
    return h;
}

/* Follow the heading for dur_ms and add up the rotation. No printing:
   the motors may be running under PI. */
LOCAL UW now_ms(void)
{
    SYSTIM tim;

    tk_get_tim(&tim);
    return tim.lo;
}

/* Time is read from the kernel clock, not counted in loop steps: each
   step is the 20 ms delay plus the I2C read and maths, so counting steps
   overstated the turn rate by ~16 % in the first run (116 vs ~100 deg/s).
   At each mark the running total and the time it was taken are saved. */
LOCAL void heading_track(INT dur_ms, INT *p_prev, W *p_total,
                         INT mark1_ms, W *p_at_mark1, UW *p_t_mark1,
                         INT mark2_ms, W *p_at_mark2, UW *p_t_mark2)
{
    imu_attitude_t att;
    UW start;
    UW t;
    BOOL got1 = FALSE;
    BOOL got2 = FALSE;

    start = now_ms();
    do {
        if(E_OK == imu_read_attitude(&att, NULL)) {
            *p_total += wrap_cdeg((INT)att.heading_cdeg - *p_prev);
            *p_prev = (INT)att.heading_cdeg;
        }
        t = now_ms() - start;
        if((NULL != p_at_mark1) && (!got1) && (t >= (UW)mark1_ms)) {
            *p_at_mark1 = *p_total;
            *p_t_mark1 = t;
            got1 = TRUE;
        }
        if((NULL != p_at_mark2) && (!got2) && (t >= (UW)mark2_ms)) {
            *p_at_mark2 = *p_total;
            *p_t_mark2 = t;
            got2 = TRUE;
        }
        tk_dly_tsk(HDG_SAMPLE_MS);
    } while((now_ms() - start) < (UW)dur_ms);
}

LOCAL void imu_test_heading(void)
{
    static const char *const dirs[4] = {
        "at your reference line (this is 0 deg)",
        "turned 90 deg CLOCKWISE from the reference",
        "turned 180 deg from the reference",
        "turned 270 deg CLOCKWISE from the reference",
    };
    imu_attitude_t att;
    enc_diag_t l, r;
    INT h[4];
    INT i, s, prev, avg_edges, mag_cdeg, enc_cdeg;
    W total, at2, at6;
    UW t2 = 0;
    UW t6 = 0;
    float enc_deg_f;

    i2c1_init();
    if(E_OK != imu_init()) {
        tm_printf((UB *)"[HDG] imu_init failed, check the IMU\n");
        return;
    }
    tk_dly_tsk(200);

    if(E_OK == imu_read_attitude(&att, NULL)) {
        tm_printf((UB *)"\n[HDG] at rest: pitch=%d roll=%d (x0.01 deg, expect near 0)"
                  " heading=%d.%02d deg\n",
                  att.pitch_cdeg, att.roll_cdeg,
                  att.heading_cdeg / 100, att.heading_cdeg % 100);
    }

    /* Phase 1: four hand-set directions, motors off */
    tm_printf((UB *)"\n[HDG] Phase 1 (motors OFF): line the car up with a table edge\n");
    tm_printf((UB *)"[HDG] or floor tile, then turn it 90 deg CLOCKWISE each time.\n");
    for(i = 0; i < 4; i++) {
        tm_printf((UB *)"\n[HDG] Position %d/4: car %s\n", i + 1, dirs[i]);
        for(s = HDG_MOVE_S; s > 0; s--) {
            tm_printf((UB *)"[HDG]   measuring in %d s\n", s);
            tk_dly_tsk(1000);
        }
        tm_printf((UB *)"[HDG]   HOLD STILL...\n");
        h[i] = heading_avg_cdeg(HDG_AVG_SAMPLES);
        if(h[i] < 0) {
            tm_printf((UB *)"[HDG]   read failed\n");
            return;
        }
        if(0 == i) {
            tm_printf((UB *)"[HDG]   heading=%d.%02d deg\n", h[i] / 100, h[i] % 100);
        } else {
            s = wrap_cdeg(h[i] - h[i - 1]);
            tm_printf((UB *)"[HDG]   heading=%d.%02d deg  step=%d.%02d deg (expect +90)\n",
                      h[i] / 100, h[i] % 100,
                      s / 100, ((s < 0) ? -s : s) % 100);
        }
    }
    s = wrap_cdeg(h[0] + 36000 - h[3]);
    tm_printf((UB *)"[HDG]   last step back to reference would be %d.%02d deg (expect +90)\n",
              s / 100, ((s < 0) ? -s : s) % 100);

    /* Phase 2: spin on the floor, motors on */
    tm_printf((UB *)"\n[HDG] Phase 2 (motors ON): put the car on the FLOOR with about\n");
    tm_printf((UB *)"[HDG] 30 cm clear all round. It will spin CLOCKWISE on the spot.\n");
    for(s = 10; s > 0; s--) {
        tm_printf((UB *)"[HDG]   spinning in %d s\n", s);
        tk_dly_tsk(1000);
    }

    prev = heading_avg_cdeg(25);
    if(prev < 0) {
        tm_printf((UB *)"[HDG] read failed\n");
        return;
    }
    total = 0;
    at2 = 0;
    at6 = 0;

    motion_reset_odometry();
    motion_set_velocity(HDG_SPIN_MM_S, -HDG_SPIN_MM_S);
    heading_track(HDG_SPIN_MS, &prev, &total, 2000, &at2, &t2, 6000, &at6, &t6);
    motion_set_velocity(0, 0);
    heading_track(1500, &prev, &total, 0, NULL, NULL, 0, NULL, NULL);  /* include the coast */

    (void)motion_get_encoder_diag(&l, &r);
    avg_edges = (INT)((((l.hw_count < 0) ? -l.hw_count : l.hw_count)
                     + ((r.hw_count < 0) ? -r.hw_count : r.hw_count)) / 2);

    /* Encoder rotation: each wheel travels pi * track per turn */
    enc_deg_f = ((float)avg_edges * (float)SWEEP_UM_PER_EDGE * 360.0f)
              / ((float)HDG_TRACK_MM * 1000.0f * 3.14159265f);
    enc_cdeg = (INT)(enc_deg_f * 100.0f);
    mag_cdeg = (INT)total;

    tm_printf((UB *)"\n[HDG] compass rotation:  %d deg\n", mag_cdeg / 100);
    tm_printf((UB *)"[HDG] encoder rotation:  %d deg (L=%d R=%d edges, %d mm track)\n",
              enc_cdeg / 100, (INT)l.hw_count, (INT)r.hw_count, HDG_TRACK_MM);
    if(mag_cdeg > 0) {
        tm_printf((UB *)"[HDG] encoder/compass = %d%%  -> effective track width for turns"
                  " about %d mm\n",
                  (INT)((W)enc_cdeg * 100 / mag_cdeg),
                  (INT)((W)HDG_TRACK_MM * enc_cdeg / mag_cdeg));
    }
    tm_printf((UB *)"[HDG] turn rate (compass, 2-6 s of spin): %d deg/s; commanded about %d deg/s\n",
              (t6 > t2) ? (INT)((at6 - at2) * 10 / (W)(t6 - t2)) : 0,
              (INT)((2L * HDG_SPIN_MM_S * 5730L) / (HDG_TRACK_MM * 100L)));

    tm_printf((UB *)"\n=== Heading test complete ===\n");
}
#elif MOTION_TEST_MODE == IMU_TEST_HUMP_LOG
/* Buddy 4 step 6a: log what the IMU sees while driving over the ramps.
   The ramps (14.5 cm long, 5 cm wide, peak 1.0 or 2.4 cm at mid-length)
   are narrower than the 114 mm track, so how the car crosses decides the
   maths: a drive wheel going over tilts the car sideways (roll), the
   castor going over tilts it nose up/down (pitch). This pass only records
   pitch, roll, vertical accel and encoder distance at ~50 Hz into RAM and
   prints them once the car has stopped, plus three candidate heights. */

typedef struct {
    uint16_t t_ms;
    int16_t  dist_mm;
    int16_t  pitch_cdeg;
    int16_t  roll_cdeg;
    int16_t  az_mg;
} hump_sample_t;

LOCAL hump_sample_t hump_log[HUMP_LOG_LEN];

/* sin of an angle in hundredths of a degree, 0..9000: Taylor to x^7,
   under 0.0001 error up to 90 deg. No libm. */
LOCAL float f_sin_small(INT cdeg)
{
    float x = (float)cdeg * (3.14159265f / 18000.0f);
    float x2 = x * x;

    return x * (1.0f - (x2 / 6.0f) * (1.0f - (x2 / 20.0f) * (1.0f - (x2 / 42.0f))));
}

/* Integer square root, for the RMS print */
LOCAL UW isqrt_w(UW v)
{
    UW r = 0;
    UW bit = 1UL << 30;

    while(bit > v) {
        bit >>= 2;
    }
    while(bit != 0) {
        if(v >= r + bit) {
            v -= r + bit;
            r = (r >> 1) + bit;
        } else {
            r >>= 1;
        }
        bit >>= 2;
    }
    return r;
}

LOCAL INT hump_dist_mm(void)
{
    enc_diag_t l, r;

    (void)motion_get_encoder_diag(&l, &r);
    return (INT)(((l.hw_count + r.hw_count) / 2) * SWEEP_UM_PER_EDGE / 1000);
}

LOCAL void hump_pass(const char *name, INT speed_mm_s)
{
    static INT fpit[HUMP_LOG_LEN];
    imu_attitude_t att;
    imu_raw_t s;
    SYSTIM tim;
    UW t0;
    UW t;
    UW stop_t = 0;
    INT n = 0;
    INT i, j, k, cnt;
    INT stop_mm = HUMP_DIST_MM;
    W p0 = 0;
    W r0 = 0;
    INT base_n = 0;
    BOOL stopping = FALSE;
    W acc_p, acc_r;
    W sq_raw = 0;
    W sq_f = 0;
    INT nf = 0;
    INT dp, fp, fr;
    INT max_dp = 0, max_dp_mm = 0;
    INT up = 0, up_mm = 0, down = 0, down_mm = 0, rmax = 0, rmax_mm = 0;
    INT plat = 0, plat_mm = 0;
    INT az_min = 32767, az_max = -32768;
    INT prev_mm = -1;
    float path_h = 0.0f;
    float path_min = 0.0f;
    float rise = 0.0f;
    INT rise_mm = 0;

    /* Baseline at rest, before the motors start. While accelerating the
       accelerometer reads the push as nose-up tilt, and braking as nose-
       down: the first filtered run took its baseline over the first
       100 mm and got +2.7 deg against +1.1 deg at rest, and "found" a
       -6 deg hump on the flat floor while stopping. */
    for(i = 0; i < HUMP_BASE_SAMPLES; i++) {
        if(E_OK == imu_read_attitude(&att, NULL)) {
            p0 += att.pitch_cdeg;
            r0 += att.roll_cdeg;
            base_n++;
        }
        tk_dly_tsk(HUMP_SAMPLE_MS);
    }
    if(base_n > 0) {
        p0 /= base_n;
        r0 /= base_n;
    }

    motion_reset_odometry();
    tk_get_tim(&tim);
    t0 = tim.lo;
    motion_set_velocity((int16_t)speed_mm_s, (int16_t)speed_mm_s);

    /* Record until the distance is covered, then 600 ms more while it
       stops. No printing in here: the motors are under PI. */
    while(n < HUMP_LOG_LEN) {
        if(E_OK == imu_read_attitude(&att, &s)) {
            tk_get_tim(&tim);
            t = tim.lo - t0;
            hump_log[n].t_ms = (uint16_t)t;
            hump_log[n].dist_mm = (int16_t)hump_dist_mm();
            hump_log[n].pitch_cdeg = att.pitch_cdeg;
            hump_log[n].roll_cdeg = att.roll_cdeg;
            hump_log[n].az_mg = s.az;
            n++;

            if((!stopping) && (hump_log[n - 1].dist_mm >= HUMP_DIST_MM)) {
                motion_set_velocity(0, 0);
                stopping = TRUE;
                stop_t = t;
                stop_mm = hump_log[n - 1].dist_mm;
            }
            if(stopping && ((t - stop_t) > 600u)) {
                break;
            }
        }
        tk_dly_tsk(HUMP_SAMPLE_MS);
    }
    if(!stopping) {
        motion_set_velocity(0, 0);
    }
    tk_dly_tsk(500);

    /* Centred moving average of pitch, relative to the rest baseline */
    for(i = 0; i < n; i++) {
        acc_p = 0;
        cnt = 0;
        for(j = -(HUMP_FILT_N / 2); j <= (HUMP_FILT_N / 2); j++) {
            k = i + j;
            if((k >= 0) && (k < n)) {
                acc_p += hump_log[k].pitch_cdeg;
                cnt++;
            }
        }
        fpit[i] = (INT)(acc_p / cnt) - (INT)p0;
    }

    /* Analyse only the constant-speed stretch: from HUMP_SKIP_MM after the
       start to the stop command. The climb and descent are described in
       the comment above the summary prints. */
    for(i = 0; i < n; i++) {
        if((hump_log[i].dist_mm < HUMP_SKIP_MM) || (hump_log[i].dist_mm > stop_mm)) {
            continue;
        }

        dp = hump_log[i].pitch_cdeg - (INT)p0;
        fp = fpit[i];
        sq_raw += (W)(dp / 10) * (dp / 10);
        sq_f += (W)(fp / 10) * (fp / 10);
        nf++;

        if(((dp < 0) ? -dp : dp) > ((max_dp < 0) ? -max_dp : max_dp)) {
            max_dp = dp;
            max_dp_mm = hump_log[i].dist_mm;
        }
        if(fp > up) {
            up = fp;
            up_mm = hump_log[i].dist_mm;
        }
        if(fp < down) {
            down = fp;
            down_mm = hump_log[i].dist_mm;
        }

        acc_r = 0;
        cnt = 0;
        for(j = -(HUMP_FILT_N / 2); j <= (HUMP_FILT_N / 2); j++) {
            k = i + j;
            if((k >= 0) && (k < n)) {
                acc_r += hump_log[k].roll_cdeg;
                cnt++;
            }
        }
        fr = (INT)(acc_r / cnt) - (INT)r0;
        if(((fr < 0) ? -fr : fr) > ((rmax < 0) ? -rmax : rmax)) {
            rmax = fr;
            rmax_mm = hump_log[i].dist_mm;
        }

        if(hump_log[i].az_mg < az_min) az_min = hump_log[i].az_mg;
        if(hump_log[i].az_mg > az_max) az_max = hump_log[i].az_mg;

        /* Climb: path height = sum of sin(pitch) x distance; keep the
           largest rise above the lowest point so far */
        if(prev_mm >= 0) {
            path_h += (float)(hump_log[i].dist_mm - prev_mm) * f_sin_small(fp);
        }
        prev_mm = hump_log[i].dist_mm;
        if(path_h < path_min) {
            path_min = path_h;
        }
        if((path_h - path_min) > rise) {
            rise = path_h - path_min;
            rise_mm = hump_log[i].dist_mm;
        }

        /* Descent plateau: the most nose-down mean over HUMP_PLAT_MM of
           travel. A mean over a stretch is steadier than a single peak. */
        acc_p = 0;
        cnt = 0;
        for(j = i; (j < n) && (hump_log[j].dist_mm < hump_log[i].dist_mm + HUMP_PLAT_MM)
                   && (hump_log[j].dist_mm <= stop_mm); j++) {
            acc_p += fpit[j];
            cnt++;
        }
        if((cnt > 0) && ((INT)(acc_p / cnt) < plat)) {
            plat = (INT)(acc_p / cnt);
            plat_mm = hump_log[i].dist_mm;
        }
    }

    tm_printf((UB *)"\n[HUMP] %s: %d samples, %d mm\n", name, n,
              (n > 0) ? hump_log[n - 1].dist_mm : 0);
    tm_printf((UB *)"[HUMP] t_ms,dist_mm,pitch_cdeg,roll_cdeg,az_mg\n");
    for(i = 0; i < n; i++) {
        tm_printf((UB *)"[HUMP] %u,%d,%d,%d,%d\n",
                  (unsigned int)hump_log[i].t_ms, hump_log[i].dist_mm,
                  hump_log[i].pitch_cdeg, hump_log[i].roll_cdeg, hump_log[i].az_mg);
    }

    /* The castor passes between the ramps and floats while the wheels
       climb, so a hump shows twice (user, 2026-10-09):
         climbing:   nose UP, the body follows the slope; height is the rise
                     of the path
         descending: nose DOWN, castor back on the floor 80 mm ahead;
                     height = 80 mm x sin(pitch)
       The two estimates are independent, so they cross-check each other. */
    tm_printf((UB *)"[HUMP] %s summary (%d..%d mm analysed):\n", name, HUMP_SKIP_MM, stop_mm);
    tm_printf((UB *)"[HUMP]   baseline at rest: pitch=%d roll=%d (x0.01 deg)\n", (INT)p0, (INT)r0);
    tm_printf((UB *)"[HUMP]   pitch noise RMS: raw %u, filtered %u (x0.1 deg)\n",
              (unsigned int)isqrt_w((nf > 0) ? (UW)(sq_raw / nf) : 0),
              (unsigned int)isqrt_w((nf > 0) ? (UW)(sq_f / nf) : 0));
    tm_printf((UB *)"[HUMP]   raw single-sample peak pitch %d at %d mm (vibration, ignore)\n",
              max_dp, max_dp_mm);
    tm_printf((UB *)"[HUMP]   az range %d..%d mg (bump jolt)\n", az_min, az_max);
    tm_printf((UB *)"[HUMP]   climbing:   nose UP peak %d (x0.01 deg) at %d mm;"
              " path rise ~ %d mm (ends at %d mm)\n", up, up_mm, (INT)rise, rise_mm);
    tm_printf((UB *)"[HUMP]   descending: nose DOWN peak %d (x0.01 deg) at %d mm"
              " -> wheel height ~ %d mm\n", down, down_mm,
              (INT)((float)HUMP_WHEELBASE_MM * f_sin_small(-down)));
    tm_printf((UB *)"[HUMP]   descending: %d mm plateau mean %d (x0.01 deg) from %d mm"
              " -> wheel height ~ %d mm  <- steadier\n", HUMP_PLAT_MM, plat, plat_mm,
              (INT)((float)HUMP_WHEELBASE_MM * f_sin_small(-plat)));
    tm_printf((UB *)"[HUMP]   filtered roll peak %d (x0.01 deg) at %d mm"
              " (one wheel higher than the other)\n", rmax, rmax_mm);
}

LOCAL void imu_test_hump_log(void)
{
    static const char *const passes[HUMP_PASSES] = {
        "Pass 1/4: FLAT floor, no ramp, 200 mm/s (baseline noise)",
        "Pass 2/4: SMALL ramps (~1.0 cm), 200 mm/s",
        "Pass 3/4: TALL ramps (~2.4 cm), 200 mm/s",
        "Pass 4/4: TALL ramps (~2.4 cm) again, SLOWER 100 mm/s",
    };
    static const INT speeds[HUMP_PASSES] = { 200, 200, 200, 100 };
    INT p, s;

    i2c1_init();
    if(E_OK != imu_init()) {
        tm_printf((UB *)"[HUMP] imu_init failed, check the IMU\n");
        return;
    }

    tm_printf((UB *)"\n[HUMP] Ramp logging: %d passes of %d cm.\n",
              HUMP_PASSES, HUMP_DIST_MM / 10);
    tm_printf((UB *)"[HUMP] Place one ramp in front of EACH drive wheel, about 20 cm\n");
    tm_printf((UB *)"[HUMP] ahead, side by side so both wheels reach them together.\n");
    tm_printf((UB *)"[HUMP] Check the castor passes between them on the floor.\n");
    tm_printf((UB *)"[HUMP] Keep the car still until it drives: it reads its rest tilt first.\n");

    for(p = 0; p < HUMP_PASSES; p++) {
        tm_printf((UB *)"\n[HUMP] %s\n", passes[p]);
        for(s = HUMP_SETUP_S; s > 0; s--) {
            tm_printf((UB *)"[HUMP]   driving in %d s\n", s);
            tk_dly_tsk(1000);
        }
        hump_pass(passes[p], speeds[p]);
    }

    tm_printf((UB *)"\n=== Hump logging complete ===\n");
}
#elif MOTION_TEST_MODE == IMU_TEST_TERRAIN
/* Buddy 4 step 7: the real-time detector (terrain.c) on the ramps.
   Events are kept in RAM and printed after each pass, because the
   motors are under PI while it runs. The braking at the end of each pass
   is fed in too, to show it does not register as a hump. */
LOCAL INT terr_dist_mm(void)
{
    enc_diag_t l, r;

    (void)motion_get_encoder_diag(&l, &r);
    return (INT)(((l.hw_count + r.hw_count) / 2) * SWEEP_UM_PER_EDGE / 1000);
}

LOCAL void terrain_pass(const char *name)
{
    static hump_event_t humps[TERR_MAX_EVENTS];
    static collision_event_t colls[TERR_MAX_EVENTS];
    imu_attitude_t att;
    imu_raw_t s;
    hump_event_t h;
    collision_event_t c;
    W p0 = 0;
    W r0 = 0;
    INT base_n = 0;
    INT nh = 0;
    INT nc = 0;
    INT i, dist;
    UW ev;
    SYSTIM tim;
    UW t_stop = 0;
    BOOL stopping = FALSE;

    for(i = 0; i < 25; i++) {
        if(E_OK == imu_read_attitude(&att, NULL)) {
            p0 += att.pitch_cdeg;
            r0 += att.roll_cdeg;
            base_n++;
        }
        tk_dly_tsk(20);
    }
    terrain_reset((int16_t)((base_n > 0) ? (p0 / base_n) : 0),
                  (int16_t)((base_n > 0) ? (r0 / base_n) : 0));

    motion_reset_odometry();
    motion_set_velocity(TERR_SPEED_MM_S, TERR_SPEED_MM_S);

    while(1) {
        if(E_OK == imu_read_attitude(&att, &s)) {
            dist = terr_dist_mm();
            ev = terrain_update(dist, &att, &s, motion_is_steady(), &h, &c);
            if((0u != (ev & TERRAIN_EVT_HUMP)) && (nh < TERR_MAX_EVENTS)) {
                humps[nh++] = h;
            }
            if((0u != (ev & TERRAIN_EVT_COLLISION)) && (nc < TERR_MAX_EVENTS)) {
                colls[nc++] = c;
            }

            tk_get_tim(&tim);
            if((!stopping) && (dist >= TERR_DIST_MM)) {
                motion_set_velocity(0, 0);
                stopping = TRUE;
                t_stop = tim.lo;
            }
            if(stopping && ((tim.lo - t_stop) > 800u)) {
                break;
            }
        }
        tk_dly_tsk(20);
    }
    tk_dly_tsk(300);

    tm_printf((UB *)"\n[TERR] %s: rest pitch %d (x0.01 deg)\n", name,
              (INT)((base_n > 0) ? (p0 / base_n) : 0));
    tm_printf((UB *)"[TERR]   %d hump(s):\n", nh);
    for(i = 0; i < nh; i++) {
        tm_printf((UB *)"[TERR]     HUMP %d..%d mm, height ~%d mm"
                  " (30 mm mean %d x0.01 deg from %d mm)\n",
                  (INT)humps[i].start_mm, (INT)humps[i].end_mm, humps[i].height_mm,
                  humps[i].plateau_cdeg, (INT)humps[i].plateau_mm);
    }
    tm_printf((UB *)"[TERR]   %d collision(s):\n", nc);
    for(i = 0; i < nc; i++) {
        tm_printf((UB *)"[TERR]     COLLISION at %d mm, jolt %d mg\n",
                  (INT)colls[i].dist_mm, colls[i].jolt_mg);
    }
    tm_printf((UB *)"[TERR]   largest jolt %d mg (collision threshold 500);"
              " most nose-down while steady %d (x0.01 deg)\n",
              terrain_max_jolt_mg(), terrain_min_pitch_cdeg());
}

LOCAL void imu_test_terrain(void)
{
    static const char *const passes[3] = {
        "Pass 1/3: FLAT floor, no ramp (expect: no hump)",
        "Pass 2/3: SMALL ramps ~1.0 cm (expect: one hump ~10 mm)",
        "Pass 3/3: TALL ramps ~2.4 cm (expect: one hump ~24 mm)",
    };
    INT p, s;

    i2c1_init();
    if(E_OK != imu_init()) {
        tm_printf((UB *)"[TERR] imu_init failed, check the IMU\n");
        return;
    }

    tm_printf((UB *)"\n[TERR] Hump/collision detector: 3 passes of %d cm at %d mm/s.\n",
              TERR_DIST_MM / 10, TERR_SPEED_MM_S);
    tm_printf((UB *)"[TERR] Same layout as before: one ramp per drive wheel, ~20 cm ahead,\n");
    tm_printf((UB *)"[TERR] castor between them. Keep the car still until it drives.\n");

    for(p = 0; p < 3; p++) {
        tm_printf((UB *)"\n[TERR] %s\n", passes[p]);
        for(s = HUMP_SETUP_S; s > 0; s--) {
            tm_printf((UB *)"[TERR]   driving in %d s\n", s);
            tk_dly_tsk(1000);
        }
        terrain_pass(passes[p]);
    }

    tm_printf((UB *)"\n=== Terrain detector test complete ===\n");
}
#elif MOTION_TEST_MODE == ULTRA_TEST_RANGE
/* Buddy 5 step 1: range calibration against a flat target at known
   distances. Uses the uncalibrated speed-of-sound conversion, then fits
   true = scale x measured + offset over all the distances. Motors off. */
LOCAL UW us_isqrt(UW v)
{
    UW r = 0;
    UW bit = 1UL << 30;

    while(bit > v) {
        bit >>= 2;
    }
    while(bit != 0) {
        if(v >= r + bit) {
            v -= r + bit;
            r = (r >> 1) + bit;
        } else {
            r >>= 1;
        }
        bit >>= 2;
    }
    return r;
}

LOCAL void ultra_test_range(void)
{
    /* Point 0 has no target: it shows what the sensor sees by itself
       (floor, table edge, clutter), which caps the useful range. */
    static const INT true_mm[US_POINTS] = { 0, 100, 200, 300, 500, 1000 };
    static W mean_mm[US_POINTS];
    static BOOL have[US_POINTS];
    UW t0, t1, echo_us;
    INT p, i, ok, miss, key;
    W mn, mx, mm;
    float sum, sum_sq, mean_f, var_f;
    float sx, sy, sxx, sxy, n_f, a, b;
    INT n_fit;
    UW last_live;

    ultrasonic_init();

    t0 = us_now();
    tk_dly_tsk(500);
    t1 = us_now();
    tm_printf((UB *)"\n[US] timer check: %u us over a 500 ms kernel delay"
              " (expect ~500000-502000)\n", (unsigned int)(t1 - t0));

    tm_printf((UB *)"[US] Range calibration, paced by you: for each point, place the\n");
    tm_printf((UB *)"[US] target, check the live reading, then press any key (e.g. ENTER).\n");
    tm_printf((UB *)"[US] Lay a tape on the floor from the sensor's FRONT FACE and slide a\n");
    tm_printf((UB *)"[US] FLAT, hard target (box side, book) along it, square to the sensor.\n");
    tm_printf((UB *)"[US] Keep the area around the beam clear.\n");

    for(p = 0; p < US_POINTS; p++) {
        have[p] = FALSE;
        if(0 == true_mm[p]) {
            tm_printf((UB *)"\n[US] Point %d/%d: NO target - clear everything in front,"
                      " then press a key\n", p + 1, US_POINTS);
        } else {
            tm_printf((UB *)"\n[US] Point %d/%d: target at %d cm, then press a key\n",
                      p + 1, US_POINTS, true_mm[p] / 10);
        }

        console_flush_input();
        last_live = 0;
        key = -1;
        while(key < 0) {
            if((us_now() - last_live) > 700000u) {
                last_live = us_now();
                if(E_OK == ultrasonic_read_us(&echo_us)) {
                    tm_printf((UB *)"[US]   live: %d mm\n", (INT)ultrasonic_us_to_mm(echo_us));
                } else {
                    tm_printf((UB *)"[US]   live: no echo\n");
                }
            }
            tk_dly_tsk(50);
            key = console_try_getc();
        }
        tm_printf((UB *)"[US]   measuring, hold still...\n");

        sum = 0.0f;
        sum_sq = 0.0f;
        mn = 0x7FFFFFFF;
        mx = 0;
        ok = 0;
        miss = 0;
        for(i = 0; i < US_SAMPLES; i++) {
            if(E_OK == ultrasonic_read_us(&echo_us)) {
                mm = ultrasonic_us_to_mm(echo_us);
                sum += (float)mm;
                sum_sq += (float)mm * (float)mm;
                if(mm < mn) mn = mm;
                if(mm > mx) mx = mm;
                ok++;
            } else {
                miss++;
            }
            tk_dly_tsk(US_PERIOD_MS);
        }

        if(0 == ok) {
            tm_printf((UB *)"[US]   no echoes at all (%d misses)%s\n", miss,
                      (0 == true_mm[p]) ? " - nothing in range: good" : " - point skipped");
            continue;
        }
        mean_f = sum / (float)ok;
        var_f = (sum_sq / (float)ok) - (mean_f * mean_f);
        mean_mm[p] = (W)(mean_f + 0.5f);
        have[p] = (0 != true_mm[p]);
        tm_printf((UB *)"[US]   %s%4d mm: mean %4d  min %4d  max %4d  sd %d mm  misses %d/%d\n",
                  (0 == true_mm[p]) ? "no target, sees something at " : "true ",
                  (0 == true_mm[p]) ? (INT)mean_mm[p] : true_mm[p],
                  (INT)mean_mm[p], (INT)mn, (INT)mx,
                  (INT)us_isqrt((var_f > 0.0f) ? (UW)var_f : 0u), miss, US_SAMPLES);
    }

    /* Least-squares fit true = a x measured + b over the target points */
    sx = sy = sxx = sxy = 0.0f;
    n_fit = 0;
    for(p = 0; p < US_POINTS; p++) {
        if(have[p]) {
            sx += (float)mean_mm[p];
            sy += (float)true_mm[p];
            sxx += (float)mean_mm[p] * (float)mean_mm[p];
            sxy += (float)mean_mm[p] * (float)true_mm[p];
            n_fit++;
        }
    }
    if(n_fit >= 2) {
        n_f = (float)n_fit;
        a = ((n_f * sxy) - (sx * sy)) / ((n_f * sxx) - (sx * sx));
        b = (sy - (a * sx)) / n_f;
        tm_printf((UB *)"\n[US] fit: true = %d/1000 x measured %c %d mm\n",
                  (INT)(a * 1000.0f), (b < 0.0f) ? '-' : '+',
                  (INT)((b < 0.0f) ? -b : b));
        for(p = 0; p < US_POINTS; p++) {
            if(have[p]) {
                tm_printf((UB *)"[US]   %4d mm: raw %4d (raw error %d)  corrected %4d"
                          " (error %d mm)\n",
                          true_mm[p], (INT)mean_mm[p], (INT)mean_mm[p] - true_mm[p],
                          (INT)((a * (float)mean_mm[p]) + b),
                          (INT)((a * (float)mean_mm[p]) + b) - true_mm[p]);
            }
        }
    } else {
        tm_printf((UB *)"\n[US] not enough points for a fit\n");
    }

    tm_printf((UB *)"\n=== Ultrasonic range test complete ===\n");
}
#elif MOTION_TEST_MODE == SERVO_TEST_JOG
/* Buddy 5 step 2: jog the scan servo from the serial monitor and mark
   centre and the two safe limits. Motors stay off. Each step also prints
   the ultrasonic reading, so you can see the beam move. */
LOCAL void servo_test_jog(void)
{
    INT key;
    INT us_now_pulse;
    INT centre = -1;
    INT lim_a = -1;
    INT lim_b = -1;
    BOOL relaxed = FALSE;
    UW echo_us;

    ultrasonic_init();
    servo_init(SERVO_JOG_START_US);

    tm_printf((UB *)"\n[SRV] Servo jog. Motors stay off. Keys (no ENTER needed):\n");
    tm_printf((UB *)"[SRV]   +  / -   step %d us      >  / <   step %d us\n",
              SERVO_JOG_SMALL_US, SERVO_JOG_BIG_US);
    tm_printf((UB *)"[SRV]   c  mark CENTRE (sensor pointing straight ahead)\n");
    tm_printf((UB *)"[SRV]   l  mark LEFT limit,  r  mark RIGHT limit\n");
    tm_printf((UB *)"[SRV]      (mark each limit a little BEFORE the mount touches)\n");
    tm_printf((UB *)"[SRV]   o  relax / re-enable the servo,  q  finish and summarise\n");
    tm_printf((UB *)"[SRV] Starting at %d us. Go in small steps near the plates!\n",
              SERVO_JOG_START_US);

    console_flush_input();
    while(1) {
        key = console_try_getc();
        if(key < 0) {
            tk_dly_tsk(30);
            continue;
        }

        us_now_pulse = (INT)servo_get_us();
        switch(key) {
        case '+': us_now_pulse += SERVO_JOG_SMALL_US; break;
        case '-': us_now_pulse -= SERVO_JOG_SMALL_US; break;
        case '>': us_now_pulse += SERVO_JOG_BIG_US;   break;
        case '<': us_now_pulse -= SERVO_JOG_BIG_US;   break;
        case 'c': centre = us_now_pulse;
                  tm_printf((UB *)"[SRV] CENTRE marked at %d us\n", centre);
                  continue;
        case 'l': lim_a = us_now_pulse;
                  tm_printf((UB *)"[SRV] LEFT limit marked at %d us\n", lim_a);
                  continue;
        case 'r': lim_b = us_now_pulse;
                  tm_printf((UB *)"[SRV] RIGHT limit marked at %d us\n", lim_b);
                  continue;
        case 'o': relaxed = !relaxed;
                  if(relaxed) {
                      servo_relax();
                      tm_printf((UB *)"[SRV] relaxed (no pulses)\n");
                  } else {
                      (void)servo_set_us((uint16_t)us_now_pulse);
                      tm_printf((UB *)"[SRV] holding %d us again\n", us_now_pulse);
                  }
                  continue;
        case 'q': break;
        default:  continue;
        }
        if('q' == key) {
            break;
        }

        relaxed = FALSE;
        us_now_pulse = (INT)servo_set_us((uint16_t)us_now_pulse);
        tk_dly_tsk(150);                         /* let it move and settle */
        if(E_OK == ultrasonic_read_us(&echo_us)) {
            tm_printf((UB *)"[SRV] pulse %d us   range %d mm\n", us_now_pulse,
                      (INT)ultrasonic_us_to_mm(echo_us));
        } else {
            tm_printf((UB *)"[SRV] pulse %d us   range: no echo\n", us_now_pulse);
        }
    }

    servo_relax();
    tm_printf((UB *)"\n[SRV] Summary: centre=%d us, left limit=%d us, right limit=%d us\n",
              centre, lim_a, lim_b);
    if((centre > 0) && (lim_a > 0) && (lim_b > 0)) {
        tm_printf((UB *)"[SRV]   left is %d us from centre, right is %d us from centre\n",
                  lim_a - centre, lim_b - centre);
        tm_printf((UB *)"[SRV]   at a typical ~11 us/deg that is about %d and %d deg"
                  " (angle check comes in step 3)\n",
                  (lim_a - centre) / 11, (lim_b - centre) / 11);
    }
    tm_printf((UB *)"\n=== Servo jog complete (servo relaxed) ===\n");
}
#elif MOTION_TEST_MODE == SERVO_TEST_MAP
/* Buddy 5 step 3: what the scanning sensor actually sees.
   Sweep 1: range at every 10 us across the safe window with a narrow
   target in front. The stretch where the target is seen gives the beam
   width (in us); its middle is where the target is.
   Then the car turns left on the spot by a measured angle (encoders,
   validated against the compass) and sweeps again: the target's shift in
   us divided by the angle turned is the servo's us per degree. */
LOCAL INT map_median3(void)
{
    UW e;
    INT v[3];
    INT n = 0;
    INT i, t;

    for(i = 0; i < 3; i++) {
        if(E_OK == ultrasonic_read_us(&e)) {
            v[n++] = (INT)ultrasonic_us_to_mm(e);
        }
        tk_dly_tsk(US_PERIOD_MS);
    }
    if(0 == n) {
        return -1;
    }
    if(n < 3) {
        return v[0];
    }
    if(v[0] > v[1]) { t = v[0]; v[0] = v[1]; v[1] = t; }
    if(v[1] > v[2]) { t = v[1]; v[1] = v[2]; v[2] = t; }
    if(v[0] > v[1]) { t = v[0]; v[0] = v[1]; v[1] = t; }
    return v[1];
}

/* Sweep right to left. Prints every step; returns the middle pulse of the
   stretch seen within near_mm, and its width, or -1 if never seen. */
LOCAL INT map_sweep(const char *name, INT near_mm, INT *p_width_us)
{
    INT pulse, mm;
    INT first = -1;
    INT last = -1;

    tm_printf((UB *)"\n[MAP] %s: pulse_us,range_mm (target counted if < %d mm)\n",
              name, near_mm);
    (void)servo_set_us(SERVO_RIGHT_US);
    tk_dly_tsk(400);
    for(pulse = SERVO_RIGHT_US; pulse <= (INT)SERVO_LEFT_US; pulse += MAP_STEP_US) {
        (void)servo_set_us((uint16_t)pulse);
        tk_dly_tsk(MAP_SETTLE_MS);
        mm = map_median3();
        tm_printf((UB *)"[MAP] %d,%d\n", pulse, mm);
        if((mm > 0) && (mm < near_mm)) {
            if(first < 0) {
                first = pulse;
            }
            last = pulse;
        }
    }
    (void)servo_set_us(SERVO_CENTRE_US);

    if(first < 0) {
        tm_printf((UB *)"[MAP] %s: target never seen\n", name);
        *p_width_us = 0;
        return -1;
    }
    *p_width_us = last - first;
    tm_printf((UB *)"[MAP] %s: target seen from %d to %d us (span %d us), middle %d us"
              " (%d us from centre)\n", name, first, last, last - first,
              (first + last) / 2, ((first + last) / 2) - (INT)SERVO_CENTRE_US);
    return (first + last) / 2;
}

LOCAL INT map_dist_avg_abs(void)
{
    enc_diag_t l, r;

    (void)motion_get_encoder_diag(&l, &r);
    return (INT)((((l.hw_count < 0) ? -l.hw_count : l.hw_count)
                + ((r.hw_count < 0) ? -r.hw_count : r.hw_count)) / 2);
}

LOCAL void servo_test_map(void)
{
    INT mid1, mid2, w1, w2, key, target_edges, edges, turned_cdeg, s;
    INT near_mm;
    INT per10;
    UW echo_us;

    ultrasonic_init();
    servo_init(SERVO_CENTRE_US);
    servo_set_limits(SERVO_RIGHT_US, SERVO_LEFT_US);

    tm_printf((UB *)"\n[MAP] Servo window %d..%d us, centre %d us (left = larger pulse).\n",
              SERVO_RIGHT_US, SERVO_LEFT_US, SERVO_CENTRE_US);
    tm_printf((UB *)"[MAP] Car ON THE FLOOR. Put a NARROW, tall target (bottle, can,\n");
    tm_printf((UB *)"[MAP] table leg) about %d cm straight ahead of the sensor, nothing\n",
              MAP_TARGET_CM);
    tm_printf((UB *)"[MAP] else within ~%d cm. Leave ~20 cm clear around the car: it\n",
              MAP_NEAR_CM);
    tm_printf((UB *)"[MAP] will turn on the spot by about %d deg. Press a key to start.\n",
              MAP_TURN_DEG);

    console_flush_input();
    key = -1;
    while(key < 0) {
        tk_dly_tsk(500);
        if(E_OK == ultrasonic_read_us(&echo_us)) {
            tm_printf((UB *)"[MAP]   centre reads %d mm\n", (INT)ultrasonic_us_to_mm(echo_us));
        }
        key = console_try_getc();
    }

    near_mm = MAP_NEAR_CM * 10;
    mid1 = map_sweep("Sweep 1 (car facing the target)", near_mm, &w1);

    /* Turn left on the spot: left wheel back, right wheel forward. One
       degree is pi x 114 mm / 360 of wheel travel = 0.995 mm. */
    target_edges = (INT)(((float)MAP_TURN_DEG * 3.14159265f * (float)HDG_TRACK_MM * 1000.0f)
                         / (360.0f * (float)SWEEP_UM_PER_EDGE));
    tm_printf((UB *)"\n[MAP] turning left %d deg (%d edges per wheel)...\n",
              MAP_TURN_DEG, target_edges);
    tk_dly_tsk(500);
    motion_reset_odometry();
    motion_set_velocity(-MAP_TURN_MM_S, MAP_TURN_MM_S);
    for(s = 0; s < 400; s++) {
        if(map_dist_avg_abs() >= (target_edges - MAP_TURN_BRAKE_EDGES)) {
            break;
        }
        tk_dly_tsk(5);
    }
    motion_set_velocity(0, 0);
    tk_dly_tsk(1000);

    edges = map_dist_avg_abs();
    turned_cdeg = (INT)(((float)edges * (float)SWEEP_UM_PER_EDGE * 36000.0f)
                        / (3.14159265f * (float)HDG_TRACK_MM * 1000.0f));
    tm_printf((UB *)"[MAP] turned %d.%02d deg by the encoders (%d edges)\n",
              turned_cdeg / 100, turned_cdeg % 100, edges);

    mid2 = map_sweep("Sweep 2 (car turned left)", near_mm, &w2);

    tm_printf((UB *)"\n[MAP] Result:\n");
    if((mid1 > 0) && (mid2 > 0) && (turned_cdeg > 0)) {
        per10 = ((mid1 - mid2) * 1000) / turned_cdeg;
        tm_printf((UB *)"[MAP]   target moved %d us for %d.%02d deg -> %d.%d us per degree\n",
                  mid2 - mid1, turned_cdeg / 100, turned_cdeg % 100,
                  per10 / 10, ((per10 < 0) ? -per10 : per10) % 10);
        if(mid1 != mid2) {
            tm_printf((UB *)"[MAP]   window: left %d deg, right %d deg from centre\n",
                      (((INT)SERVO_LEFT_US - (INT)SERVO_CENTRE_US) * turned_cdeg)
                          / ((mid1 - mid2) * 100),
                      (((INT)SERVO_RIGHT_US - (INT)SERVO_CENTRE_US) * turned_cdeg)
                          / ((mid1 - mid2) * 100));
            tm_printf((UB *)"[MAP]   beam: target seen over %d us and %d us -> about %d deg wide\n",
                      w1, w2, (((w1 + w2) / 2) * turned_cdeg) / ((mid1 - mid2) * 100));
        }
    } else {
        tm_printf((UB *)"[MAP]   not enough to compute (target missing in a sweep)\n");
    }

    servo_relax();
    tm_printf((UB *)"\n=== Servo map complete (servo relaxed) ===\n");
}
#elif MOTION_TEST_MODE == ULTRA_TEST_BEAM
/* Buddy 5 step 3b: the sensor's real beam width.
   The servo map showed a narrow target at 33 cm is seen across the whole
   servo window (+/-16-18 deg) and still 15 deg off to the side, so the
   beam is wider than the servo can sweep. Here the servo stays at centre
   and the CAR turns instead (encoder angle, checked against the compass
   to 0.2 %): from 45 deg right of the target to 45 deg left, pinging all
   the way. The angles over which the target is seen give the beam width,
   measured at 20, 35 and 50 cm. */
typedef struct {
    int16_t angle_cdeg;   /* car heading change, left positive */
    int16_t range_mm;     /* -1 = no echo */
} beam_sample_t;

LOCAL beam_sample_t beam_log[BEAM_LOG_LEN];

/* Signed turn since the last odometry reset, left positive, x0.01 deg.
   On the spot each wheel travels pi x track / 360 per degree. */
LOCAL INT beam_angle_cdeg(void)
{
    enc_diag_t l, r;

    (void)motion_get_encoder_diag(&l, &r);
    return (INT)(((float)(r.hw_count - l.hw_count) * 0.5f * (float)SWEEP_UM_PER_EDGE * 36000.0f)
                 / (3.14159265f * (float)HDG_TRACK_MM * 1000.0f));
}

/* Turn on the spot by deg (left positive) at speed mm/s, measured */
LOCAL void beam_turn(INT deg, INT speed)
{
    INT start, target, a;
    INT brake_cdeg = BEAM_BRAKE_CDEG;
    INT i;

    start = beam_angle_cdeg();
    target = start + deg * 100;
    if(deg > 0) {
        motion_set_velocity((int16_t)-speed, (int16_t)speed);
    } else {
        motion_set_velocity((int16_t)speed, (int16_t)-speed);
    }
    for(i = 0; i < 2000; i++) {
        a = beam_angle_cdeg();
        if(((deg > 0) && (a >= target - brake_cdeg)) || ((deg < 0) && (a <= target + brake_cdeg))) {
            break;
        }
        tk_dly_tsk(5);
    }
    motion_set_velocity(0, 0);
    tk_dly_tsk(700);
}

LOCAL void beam_pass(INT target_cm)
{
    INT n = 0;
    INT i, key, mm, a;
    INT near_mm;
    INT seen_lo = 32767;
    INT seen_hi = -32768;
    UW e;

    tm_printf((UB *)"\n[BEAM] Put the narrow target %d cm straight ahead of the sensor\n", target_cm);
    tm_printf((UB *)"[BEAM] (nothing else within ~%d cm), then press a key.\n", target_cm + 20);
    console_flush_input();
    key = -1;
    while(key < 0) {
        tk_dly_tsk(600);
        if(E_OK == ultrasonic_read_us(&e)) {
            tm_printf((UB *)"[BEAM]   ahead: %d mm\n", (INT)ultrasonic_us_to_mm(e));
        } else {
            tm_printf((UB *)"[BEAM]   ahead: no echo\n");
        }
        key = console_try_getc();
    }

    motion_reset_odometry();
    beam_turn(-BEAM_HALF_DEG, BEAM_TURN_MM_S);                /* face right of it */

    motion_set_velocity((int16_t)-BEAM_SCAN_MM_S, (int16_t)BEAM_SCAN_MM_S);
    while(n < BEAM_LOG_LEN) {
        a = beam_angle_cdeg();
        if(E_OK == ultrasonic_read_us(&e)) {
            mm = (INT)ultrasonic_us_to_mm(e);
        } else {
            mm = -1;
        }
        beam_log[n].angle_cdeg = (int16_t)a;
        beam_log[n].range_mm = (int16_t)mm;
        n++;
        if(a >= BEAM_HALF_DEG * 100) {
            break;
        }
        tk_dly_tsk(US_PERIOD_MS);
    }
    motion_set_velocity(0, 0);
    tk_dly_tsk(700);
    beam_turn(-BEAM_HALF_DEG, BEAM_TURN_MM_S);                /* back to facing it */

    /* The target counts if within 10 cm of where it was placed */
    near_mm = target_cm * 10 + 100;
    tm_printf((UB *)"\n[BEAM] %d cm: angle_cdeg,range_mm (left positive)\n", target_cm);
    for(i = 0; i < n; i++) {
        tm_printf((UB *)"[BEAM] %d,%d\n", beam_log[i].angle_cdeg, beam_log[i].range_mm);
        if((beam_log[i].range_mm > 0) && (beam_log[i].range_mm < near_mm)) {
            if(beam_log[i].angle_cdeg < seen_lo) seen_lo = beam_log[i].angle_cdeg;
            if(beam_log[i].angle_cdeg > seen_hi) seen_hi = beam_log[i].angle_cdeg;
        }
    }
    if(seen_hi >= seen_lo) {
        tm_printf((UB *)"[BEAM] %d cm summary: target seen from %d to %d (x0.01 deg)"
                  " -> beam about %d deg wide, centred %d deg\n",
                  target_cm, seen_lo, seen_hi, (seen_hi - seen_lo) / 100,
                  (seen_lo + seen_hi) / 200);
    } else {
        tm_printf((UB *)"[BEAM] %d cm summary: target never seen within %d mm\n",
                  target_cm, near_mm);
    }
}

LOCAL void ultra_test_beam(void)
{
    static const INT dists[3] = { 20, 35, 50 };
    INT p;

    ultrasonic_init();
    servo_init(SERVO_CENTRE_US);
    servo_set_limits(SERVO_RIGHT_US, SERVO_LEFT_US);

    tm_printf((UB *)"\n[BEAM] Beam width test. Servo held at centre; the CAR turns on the\n");
    tm_printf((UB *)"[BEAM] spot from %d deg right to %d deg left of the target and back.\n",
              BEAM_HALF_DEG, BEAM_HALF_DEG);
    tm_printf((UB *)"[BEAM] Car on the floor, ~20 cm clear around it, open space beyond.\n");

    for(p = 0; p < 3; p++) {
        beam_pass(dists[p]);
    }

    servo_relax();
    tm_printf((UB *)"\n=== Beam width test complete (servo relaxed) ===\n");
}
#elif MOTION_TEST_MODE == ULTRA_TEST_SCAN
/* Buddy 5 step 4: the obstacle scan routine (scan.c), key-paced so you
   can try different obstacles and positions. */
LOCAL void ultra_test_scan(void)
{
    obstacle_profile_t prof;
    INT run, key;
    INT shift_left, shift_right;
    UW e;

    ultrasonic_init();
    servo_init(SERVO_CENTRE_US);
    servo_set_limits(SERVO_RIGHT_US, SERVO_LEFT_US);

    tm_printf((UB *)"\n[SCAN] Obstacle scan: the car turns %d deg each way on the spot,\n",
              SCAN_HALF_DEG);
    tm_printf((UB *)"[SCAN] then looks closely at each edge. Car on the floor with ~20 cm\n");
    tm_printf((UB *)"[SCAN] clear around it. Obstacles within %d cm count.\n", SCAN_DETECT_MM / 10);

    for(run = 1; run <= SCAN_RUNS; run++) {
        tm_printf((UB *)"\n[SCAN] Run %d/%d: place an obstacle (box, hand...) in front,\n",
                  run, SCAN_RUNS);
        tm_printf((UB *)"[SCAN] e.g. 30-40 cm away, centred or off to one side; note where.\n");
        tm_printf((UB *)"[SCAN] Press a key to scan.\n");
        console_flush_input();
        key = -1;
        while(key < 0) {
            tk_dly_tsk(700);
            if(E_OK == ultrasonic_read_us(&e)) {
                tm_printf((UB *)"[SCAN]   ahead: %d mm\n", (INT)ultrasonic_us_to_mm(e));
            } else {
                tm_printf((UB *)"[SCAN]   ahead: no echo\n");
            }
            key = console_try_getc();
        }

        if(E_OK != scan_obstacle(SCAN_HALF_DEG, SCAN_DETECT_MM, TRUE, &prof)) {
            tm_printf((UB *)"[SCAN] scan failed\n");
            continue;
        }

        tm_printf((UB *)"\n[SCAN] Run %d profile:\n", run);
        if(!prof.found) {
            tm_printf((UB *)"[SCAN]   nothing within %d mm: path clear\n", SCAN_DETECT_MM);
            continue;
        }
        tm_printf((UB *)"[SCAN]   nearest %d mm at %d.%d deg (left +)\n", prof.nearest_mm,
                  prof.nearest_cdeg / 100, ((prof.nearest_cdeg < 0) ? -prof.nearest_cdeg
                                            : prof.nearest_cdeg) % 100 / 10);
        tm_printf((UB *)"[SCAN]   seen from %d to %d deg (span %d deg, beam here %d deg)\n",
                  prof.right_cdeg / 100, prof.left_cdeg / 100,
                  (prof.left_cdeg - prof.right_cdeg) / 100, prof.beam_cdeg / 100);
        if(prof.width_mm > 0) {
            tm_printf((UB *)"[SCAN]   estimated width %d mm\n", prof.width_mm);
        } else {
            tm_printf((UB *)"[SCAN]   width below resolution (narrower than the beam spread)\n");
        }
        tm_printf((UB *)"[SCAN]   sides: %d mm (left side) .. %d mm (right side),"
                  " from the car centreline, left +\n", prof.left_side_mm, prof.right_side_mm);

        /* To pass on the left, the car's right flank (-half width) must
           clear the obstacle's left side, and vice versa */
        shift_left = prof.left_side_mm + SCAN_CAR_HALF_MM;
        shift_right = SCAN_CAR_HALF_MM - prof.right_side_mm;
        if((prof.right_side_mm > SCAN_CAR_HALF_MM) || (prof.left_side_mm < -SCAN_CAR_HALF_MM)) {
            tm_printf((UB *)"[SCAN]   straight ahead is CLEAR of it\n");
        } else if(shift_left <= shift_right) {
            tm_printf((UB *)"[SCAN]   blocked: pass LEFT (shift %d mm), right would need %d mm\n",
                      shift_left, shift_right);
        } else {
            tm_printf((UB *)"[SCAN]   blocked: pass RIGHT (shift %d mm), left would need %d mm\n",
                      shift_right, shift_left);
        }
    }

    servo_relax();
    tm_printf((UB *)"\n=== Obstacle scan test complete (servo relaxed) ===\n");
}
#elif MOTION_TEST_MODE == DEMO_ROAM
/* Integrated demo, Buddies 2 + 4 + 5: a "Roomba without the bumping".
   Buddy 5: HC-SR04 at servo centre looks ahead while driving.
   Buddy 2: PI drives at ROAM_SPEED_MM_S, and the encoders measure each
            45 deg left turn on the spot.
   Buddy 4: the compass reports how far each turn went (independent of the
            wheels); the terrain detector flags a jolt, and it counts as a
            bump only if the wheels slow down too (a stall alone also counts);
            it also counts humps driven over.
   Nothing is printed while the car drives (usermain outranks the motion
   task), only while it is stopped.

   ROAM_STOP_MM is the raw sensor reading. The range test put the raw
   reading ~58 mm above the distance from the car front, so the car stops
   roughly 14-20 cm short of the obstacle's face. */
LOCAL int32_t roam_l0;
LOCAL int32_t roam_r0;
LOCAL int32_t roam_d0;

/* Car rotation since roam_l0/roam_r0, left positive, x0.01 deg */
LOCAL INT roam_angle_cdeg(void)
{
    enc_diag_t l, r;
    float half_diff;

    (void)motion_get_encoder_diag(&l, &r);
    half_diff = ((float)((r.hw_count - roam_r0) - (l.hw_count - roam_l0))) * 0.5f;
    return (INT)((half_diff * (float)SWEEP_UM_PER_EDGE * 36000.0f)
                 / (3.14159265f * (float)HDG_TRACK_MM * 1000.0f));
}

/* Forward travel since the demo started (turns cancel out), mm */
LOCAL INT roam_dist_mm(void)
{
    enc_diag_t l, r;

    (void)motion_get_encoder_diag(&l, &r);
    return (INT)((((l.hw_count + r.hw_count) / 2) - roam_d0) * SWEEP_UM_PER_EDGE / 1000);
}

LOCAL INT roam_ping_mm(void)
{
    UW e;

    if(E_OK == ultrasonic_read_us_max(&e, ROAM_ECHO_MAX_US)) {
        return (INT)ultrasonic_us_to_mm(e);
    }
    return -1;
}

LOCAL BOOL roam_near(INT mm)
{
    return (mm > 0) && (mm <= ROAM_STOP_MM);
}

/* Stopped: three pings, blocked if at least two are near. p_mm gets the
   nearest near reading, or the last reading when clear. */
LOCAL BOOL roam_blocked(INT *p_mm)
{
    INT i, mm;
    INT near_n = 0;
    INT nearest = -1;

    for(i = 0; i < 3; i++) {
        mm = roam_ping_mm();
        if(roam_near(mm)) {
            near_n++;
            if((nearest < 0) || (mm < nearest)) {
                nearest = mm;
            }
        }
        *p_mm = mm;
        tk_dly_tsk(US_PERIOD_MS);
    }
    if(near_n >= 2) {
        *p_mm = nearest;
        return TRUE;
    }
    return FALSE;
}

/* Compass heading averaged over 0.2 s (unwrapped around the first
   sample), x0.01 deg clockwise from magnetic north, or -1 on failure */
LOCAL INT roam_heading_cdeg(void)
{
    imu_attitude_t att;
    INT first = -1;
    INT n = 0;
    INT i, d;
    W sum = 0;

    for(i = 0; i < 10; i++) {
        if(E_OK == imu_read_attitude(&att, NULL)) {
            if(first < 0) {
                first = (INT)att.heading_cdeg;
            }
            d = (INT)att.heading_cdeg - first;
            if(d > 18000) {
                d -= 36000;
            } else if(d < -18000) {
                d += 36000;
            }
            sum += d;
            n++;
        }
        tk_dly_tsk(20);
    }
    if(n == 0) {
        return -1;
    }
    d = first + (INT)(sum / n);
    if(d < 0) {
        d += 36000;
    } else if(d >= 36000) {
        d -= 36000;
    }
    return d;
}

/* 45 deg left on the spot by the encoders; returns the angle reached */
LOCAL INT roam_turn_left(void)
{
    enc_diag_t l, r;
    UW i;

    (void)motion_get_encoder_diag(&l, &r);
    roam_l0 = l.hw_count;
    roam_r0 = r.hw_count;

    motion_set_velocity(-ROAM_TURN_MM_S, ROAM_TURN_MM_S);
    for(i = 0; i < 1000u; i++) {
        if(roam_angle_cdeg() >= (ROAM_TURN_CDEG - ROAM_TURN_BRAKE_CDEG)) {
            break;
        }
        tk_dly_tsk(5);
    }
    motion_set_velocity(0, 0);
    tk_dly_tsk(400);
    return roam_angle_cdeg();
}

/* Print x0.01 deg as d.d, sign kept */
LOCAL void roam_print_deg(INT cdeg)
{
    INT a = (cdeg < 0) ? -cdeg : cdeg;

    tm_printf((UB *)"%s%d.%d", (cdeg < 0) ? "-" : "", a / 100, (a % 100) / 10);
}

/* Milliseconds from the kernel clock */
LOCAL UW roam_now_ms(void)
{
    SYSTIM tim;

    tk_get_tim(&tim);
    return tim.lo;
}

LOCAL void demo_roam(void)
{
    static imu_attitude_t att;
    static imu_raw_t cal;
    static hump_event_t hump;
    static collision_event_t coll;
    enc_diag_t l, r;
    BOOL imu_ok;
    BOOL bumped, paused, force_turn;
    INT s, tick, near_n, mm, enc, h0, h1, d;
    INT legs = 0;
    INT turns = 0;
    INT humps = 0;
    INT leg_start;
    static UW ring_t[ROAM_SPD_RING];
    static INT ring_d[ROAM_SPD_RING];
    static INT ign_mg[ROAM_IGNORED_MAX];
    static INT ign_pct[ROAM_IGNORED_MAX];
    INT ring_n, ring_i, k, j, v_pct, min_pct, n_ign, dist;
    UW t, leg_t0, jolt_t;
    BOOL jolt_pending, stalled;
    W p0 = 0;
    W r0 = 0;
    INT base_n = 0;
    UW ev;

    ultrasonic_init();
    servo_init(SERVO_CENTRE_US);
    servo_set_limits(SERVO_RIGHT_US, SERVO_LEFT_US);
    i2c1_init();
    imu_ok = (E_OK == imu_init());

    tm_printf((UB *)"\n[ROAM] Roaming demo (Buddies 2 + 4 + 5).\n");
    tm_printf((UB *)"[ROAM] Drives at %d mm/s; an obstacle within %d mm (sensor reading)\n",
              ROAM_SPEED_MM_S, ROAM_STOP_MM);
    tm_printf((UB *)"[ROAM] stops it, then it turns 45 deg left until the way is clear.\n");
    tm_printf((UB *)"[ROAM] Any key pauses / resumes.\n");
    if(!imu_ok) {
        tm_printf((UB *)"[ROAM] imu_init failed: running without compass and bump stop\n");
    }
    for(s = ROAM_START_S; s > 0; s--) {
        tm_printf((UB *)"[ROAM] starting in %d s (keep the car still)\n", s);
        tk_dly_tsk(1000);
    }

    if(imu_ok) {
        for(s = 0; s < 25; s++) {
            if(E_OK == imu_read_attitude(&att, NULL)) {
                p0 += att.pitch_cdeg;
                r0 += att.roll_cdeg;
                base_n++;
            }
            tk_dly_tsk(20);
        }
        terrain_reset((int16_t)((base_n > 0) ? (p0 / base_n) : 0),
                      (int16_t)((base_n > 0) ? (r0 / base_n) : 0));
    }
    (void)motion_get_encoder_diag(&l, &r);
    roam_d0 = (l.hw_count + r.hw_count) / 2;
    console_flush_input();
    force_turn = FALSE;

    while(1) {
        /* ---- Stopped: turn 45 deg left until nothing is within range ---- */
        while(force_turn || roam_blocked(&mm)) {
            if(force_turn) {
                tm_printf((UB *)"[ROAM] turning away from the bump\n");
            } else {
                tm_printf((UB *)"[ROAM] blocked at %d mm: turning 45 deg left\n", mm);
            }
            force_turn = FALSE;
            h0 = imu_ok ? roam_heading_cdeg() : -1;
            enc = roam_turn_left();
            h1 = imu_ok ? roam_heading_cdeg() : -1;
            turns++;

            tm_printf((UB *)"[ROAM]   turn %d: encoders ", turns);
            roam_print_deg(enc);
            if((h0 >= 0) && (h1 >= 0)) {
                /* Heading is clockwise, so a left turn lowers it */
                d = h0 - h1;
                if(d > 18000) {
                    d -= 36000;
                } else if(d < -18000) {
                    d += 36000;
                }
                tm_printf((UB *)" deg, compass ");
                roam_print_deg(d);
                tm_printf((UB *)" deg (heading %d -> %d)\n", h0 / 100, h1 / 100);
            } else {
                tm_printf((UB *)" deg\n");
            }
        }

        /* ---- Drive until something is near, a bump, or a key ---- */
        legs++;
        tm_printf((UB *)"[ROAM] clear (ahead %d mm): leg %d, driving\n", mm, legs);
        leg_start = roam_dist_mm();
        bumped = FALSE;
        stalled = FALSE;
        paused = FALSE;
        jolt_pending = FALSE;
        jolt_t = 0u;
        min_pct = 100;
        v_pct = 100;
        n_ign = 0;
        ring_n = 0;
        ring_i = 0;
        near_n = 0;
        tick = 0;
        leg_t0 = roam_now_ms();
        motion_set_velocity(ROAM_SPEED_MM_S, ROAM_SPEED_MM_S);

        while(1) {
            /* Forward speed over the last ROAM_SPD_WIN_MS, % of the command
               (100 until a full window has passed after the start) */
            t = roam_now_ms();
            dist = roam_dist_mm();
            ring_t[ring_i] = t;
            ring_d[ring_i] = dist;
            ring_i = (ring_i + 1) % ROAM_SPD_RING;
            if(ring_n < ROAM_SPD_RING) {
                ring_n++;
            }
            v_pct = 100;
            if((t - leg_t0) >= ROAM_SPD_SETTLE_MS) {
                for(j = 1; j < ring_n; j++) {
                    k = (ring_i - 1 - j + ROAM_SPD_RING) % ROAM_SPD_RING;
                    if((t - ring_t[k]) >= ROAM_SPD_WIN_MS) {
                        v_pct = (INT)(((dist - ring_d[k]) * 100000)
                                      / ((INT)(t - ring_t[k]) * ROAM_SPEED_MM_S));
                        break;
                    }
                }
            }

            /* A real hit slows the wheels too; a floor seam or the castor
               flicking round only jolts the IMU */
            if(imu_ok && (E_OK == imu_read_attitude(&att, &cal))) {
                ev = terrain_update(dist, &att, &cal, motion_is_steady(),
                                    &hump, &coll);
                if(0u != (ev & (TERRAIN_EVT_HUMP | TERRAIN_EVT_SIDE_HUMP))) {
                    humps++;
                }
                if((0u != (ev & TERRAIN_EVT_COLLISION)) && (!jolt_pending)) {
                    jolt_pending = TRUE;
                    jolt_t = t;
                    min_pct = v_pct;
                }
            }
            if(jolt_pending) {
                if(v_pct < min_pct) {
                    min_pct = v_pct;
                }
                if(min_pct < ROAM_JOLT_SLOW_PCT) {
                    bumped = TRUE;
                    break;
                }
                if((t - jolt_t) > ROAM_JOLT_CONFIRM_MS) {
                    if(n_ign < ROAM_IGNORED_MAX) {
                        ign_mg[n_ign] = coll.jolt_mg;
                        ign_pct[n_ign] = min_pct;
                        n_ign++;
                    }
                    jolt_pending = FALSE;
                }
            }
            /* Pushing against something the sonar missed, jolt or not:
               the motion task's stall/encoder checks raise a fault and
               stop the wheels. They are masked while on a hump. */
            motion_set_hump_mask(terrain_on_hump());
            if(0u != (fault_get() & FAULT_MOTION_MASK)) {
                stalled = TRUE;
                break;
            }

            if(++tick >= ROAM_PING_EVERY) {
                tick = 0;
                mm = roam_ping_mm();
                if(roam_near(mm)) {
                    if(++near_n >= ROAM_CONFIRM) {
                        break;
                    }
                } else {
                    near_n = 0;
                }
            }
            if(console_try_getc() >= 0) {
                paused = TRUE;
                break;
            }
            tk_dly_tsk(20);
        }
        motion_stop();
        motion_set_hump_mask(FALSE);
        tk_dly_tsk(500);

        tm_printf((UB *)"[ROAM]   motion loop %d passes/s, faults 0x%x\n",
                  (INT)motion_loop_rate(), (INT)fault_get());
        for(j = 0; j < n_ign; j++) {
            tm_printf((UB *)"[ROAM]   ignored jolt %d mg: wheels kept %d%% speed\n",
                      ign_mg[j], ign_pct[j]);
        }
        tm_printf((UB *)"[ROAM] stopped after %d mm", roam_dist_mm() - leg_start);
        if(bumped || stalled) {
            if(bumped) {
                tm_printf((UB *)": BUMP (jolt %d mg, wheels down to %d%%), backing off\n",
                          coll.jolt_mg, min_pct);
            } else {
                tm_printf((UB *)": %s (wheels at %d%%), backing off\n",
                          (0u != (fault_get() & FAULT_MOTOR_STALL)) ? "STALLED"
                          : "ENCODER FAULT", v_pct);
                /* Cleared by the controller once it has handled it */
                fault_clear(FAULT_MOTION_MASK);
            }
            motion_set_velocity(-ROAM_BACK_MM_S, -ROAM_BACK_MM_S);
            tk_dly_tsk(ROAM_BACK_MS);
            motion_set_velocity(0, 0);
            tk_dly_tsk(500);
            force_turn = TRUE;
        } else if(paused) {
            tm_printf((UB *)": paused. Press a key to resume.\n");
        } else {
            tm_printf((UB *)": obstacle at %d mm\n", mm);
        }
        if(humps > 0) {
            tm_printf((UB *)"[ROAM]   humps so far: %d (last ~%d mm high)\n",
                      humps, hump.height_mm);
        }

        if(paused) {
            tk_dly_tsk(300);
            console_flush_input();
            while(console_try_getc() < 0) {
                tk_dly_tsk(100);
            }
            tm_printf((UB *)"[ROAM] resumed (%d legs, %d turns so far)\n", legs, turns);
        }
    }
}
#elif MOTION_TEST_MODE == IR_TEST_LOG
/* Buddy 3 step 1: IR bring-up. Live readout to check polarity, then
   hand-pushed passes (motors off) that record every sensor change with
   the encoder distance, printed afterwards as black/white run widths. */
typedef struct {
    INT mm10;                    /* distance, x0.1 mm */
    UB  bits;                    /* ir_read() after the change */
} ir_evt_t;

LOCAL UW ir_e0;

/* Distance pushed since the pass started, x0.1 mm. hw_edges counts either
   way round, so push in one direction only. */
LOCAL INT ir_pos_mm10(void)
{
    enc_diag_t l, r;

    (void)motion_get_encoder_diag(&l, &r);
    return (INT)(((l.hw_edges + r.hw_edges - ir_e0) * SWEEP_UM_PER_EDGE) / 200u);
}

LOCAL void ir_print_mm10(INT v)
{
    tm_printf((UB *)"%d.%d", v / 10, v % 10);
}

/* One sensor's runs between its first and last change in the pass */
LOCAL void ir_print_runs(const ir_evt_t *ev, INT n, UB bit, const char *name)
{
    INT i, k = 0;
    INT last_mm10 = -1;
    INT short_n = 0;
    BOOL was = FALSE;

    tm_printf((UB *)"[IR] %s runs, mm (B black, W white):\n[IR]  ", name);
    for(i = 0; i < n; i++) {
        BOOL now = (0u != (ev[i].bits & bit));

        if((i > 0) && (now == was)) {
            continue;                    /* another sensor changed */
        }
        if(last_mm10 >= 0) {
            tm_printf((UB *)" %s", was ? "B" : "W");
            ir_print_mm10(ev[i].mm10 - last_mm10);
            if((ev[i].mm10 - last_mm10) < IR_SHORT_MM10) {
                short_n++;
            }
            if((++k % 10) == 0) {
                tm_printf((UB *)"\n[IR]  ");
            }
        }
        last_mm10 = ev[i].mm10;
        was = now;
    }
    if(k == 0) {
        tm_printf((UB *)" (no change)");
    }
    tm_printf((UB *)"\n[IR]   %d runs, %d shorter than 1 mm (edge chatter)\n", k, short_n);
}

LOCAL void ir_test_log(void)
{
    static ir_evt_t ev[IR_LOG_LEN];
    enc_diag_t l, r;
    INT pass, n, key;
    UB bits, prev;

    ir_init();
    tm_printf((UB *)"\n[IR] IR bring-up: L=GP%d C=GP%d R=GP%d, 1 = black expected.\n",
              IR_LEFT_DOUT, IR_CENTER_DOUT, IR_RIGHT_DOUT);
    tm_printf((UB *)"[IR] Live readout: hold the car with each sensor over white, then\n");
    tm_printf((UB *)"[IR] black (the line or tape). Press a key to go on to the passes.\n");
    console_flush_input();
    key = -1;
    while(key < 0) {
        bits = (UB)ir_read();
        tm_printf((UB *)"[IR]   L=%d C=%d R=%d\n", (0u != (bits & IR_BIT_LEFT)) ? 1 : 0,
                  (0u != (bits & IR_BIT_CENTRE)) ? 1 : 0,
                  (0u != (bits & IR_BIT_RIGHT)) ? 1 : 0);
        tk_dly_tsk(400);
        key = console_try_getc();
    }

    for(pass = 1; ; pass++) {
        tm_printf((UB *)"\n[IR] Pass %d: put the car ~3 cm before the line or barcode,\n", pass);
        tm_printf((UB *)"[IR] press a key, push it slowly and steadily forwards only across,\n");
        tm_printf((UB *)"[IR] then press a key to stop recording.\n");
        console_flush_input();
        while(console_try_getc() < 0) {
            tk_dly_tsk(50);
        }
        tm_printf((UB *)"[IR] recording...\n");
        tk_dly_tsk(100);
        console_flush_input();

        (void)motion_get_encoder_diag(&l, &r);
        ir_e0 = l.hw_edges + r.hw_edges;
        n = 0;
        prev = (UB)ir_read();
        ev[n].mm10 = 0;
        ev[n].bits = prev;
        n++;
        /* No printing here: a print would stretch the poll and blur edges */
        while((n < IR_LOG_LEN) && (console_try_getc() < 0)) {
            bits = (UB)ir_read();
            if(bits != prev) {
                ev[n].mm10 = ir_pos_mm10();
                ev[n].bits = bits;
                n++;
                prev = bits;
            }
            tk_dly_tsk(1);
        }

        tm_printf((UB *)"\n[IR] Pass %d: pushed ", pass);
        ir_print_mm10(ir_pos_mm10());
        tm_printf((UB *)" mm, %d changes%s\n", n - 1,
                  (n >= IR_LOG_LEN) ? " (log FULL, stopped early)" : "");
        ir_print_runs(ev, n, IR_BIT_LEFT, "LEFT  (barcode)");
        ir_print_runs(ev, n, IR_BIT_CENTRE, "CENTRE");
        ir_print_runs(ev, n, IR_BIT_RIGHT, "RIGHT ");
    }
}
#elif MOTION_TEST_MODE == IR_TEST_AO
/* Buddy 3 step 1b: left IR analog out (GP27) against encoder distance,
   hand-pushed with the motors off. Each pass stores one AO reading per
   encoder step (0.33 mm), then prints the white/black levels, the bar and
   space widths found with a mid-level threshold, and the raw profile. */
LOCAL UW ao_e0;

/* Encoder steps (0.328 mm) pushed since the pass started. hw_edges
   counts either way round, so push in one direction only. */
LOCAL INT ao_step(void)
{
    enc_diag_t l, r;

    (void)motion_get_encoder_diag(&l, &r);
    return (INT)((l.hw_edges + r.hw_edges - ao_e0) / 2u);
}

LOCAL void ao_print_mm(INT steps)
{
    INT mm10 = (steps * SWEEP_UM_PER_EDGE) / 100;

    tm_printf((UB *)"%d.%d", mm10 / 10, mm10 % 10);
}

LOCAL void ao_analyse(const UH *prof, INT n)
{
    INT i, lo = 3300, hi = 0, thr, hys, start = 0, ext;
    INT runs = 0;
    BOOL black;

    for(i = 0; i < n; i++) {
        if(prof[i] < lo) {
            lo = prof[i];
        }
        if(prof[i] > hi) {
            hi = prof[i];
        }
    }
    thr = (lo + hi) / 2;
    hys = (hi - lo) / 8;
    tm_printf((UB *)"[AO] white %d mV, darkest %d mV -> threshold %d +/- %d mV\n",
              lo, hi, thr, hys);
    if((hi - lo) < 150) {
        tm_printf((UB *)"[AO] contrast under 150 mV: no bars to split\n");
        return;
    }

    /* Runs with hysteresis; for each, the extreme reading inside it (peak
       for black, dip for white) shows how close it came to the threshold */
    tm_printf((UB *)"[AO] runs, mm (peak or dip mV), B black, W white:\n[AO]  ");
    black = (prof[0] > thr);
    ext = prof[0];
    for(i = 1; i <= n; i++) {
        BOOL flip = FALSE;

        if(i < n) {
            if(black && (prof[i] < (thr - hys))) {
                flip = TRUE;
            } else if((!black) && (prof[i] > (thr + hys))) {
                flip = TRUE;
            }
        }
        if(flip || (i == n)) {
            tm_printf((UB *)" %s", black ? "B" : "W");
            ao_print_mm(i - start);
            tm_printf((UB *)"(%d)", ext);
            if((++runs % 6) == 0) {
                tm_printf((UB *)"\n[AO]  ");
            }
            if(i < n) {
                black = !black;
                start = i;
                ext = prof[i];
            }
        } else if(black ? (prof[i] > ext) : (prof[i] < ext)) {
            ext = prof[i];
        }
    }
    tm_printf((UB *)"\n[AO]   %d runs (first and last are the lead-in/out)\n", runs);
}

LOCAL void ir_test_ao(void)
{
    static UH prof[IR_AO_LEN];
    enc_diag_t l, r;
    INT pass, n, i, k, s, last;
    UH mv;

    ir_init();
    ir_ao_init();
    tm_printf((UB *)"\n[AO] Left IR analog test: AOUT on GP%d (ADC%d), DOUT on GP%d.\n",
              IR_LEFT_AOUT, IR_LEFT_AOUT - 26, IR_LEFT_DOUT);
    tm_printf((UB *)"[AO] Live readout: hold the left IR over white, the line, then a\n");
    tm_printf((UB *)"[AO] thin and a wide barcode bar. Press a key to go on.\n");
    console_flush_input();
    while(console_try_getc() < 0) {
        tm_printf((UB *)"[AO]   AO=%d mV  DOUT=%d\n", (INT)ir_ao_left_mv(),
                  (0u != (ir_read() & IR_BIT_LEFT)) ? 1 : 0);
        tk_dly_tsk(400);
    }

    for(pass = 1; ; pass++) {
        tm_printf((UB *)"\n[AO] Pass %d: left IR ~2 cm before the barcode, press a key,\n", pass);
        tm_printf((UB *)"[AO] push slowly forwards only until ~2 cm past it, press a key.\n");
        console_flush_input();
        while(console_try_getc() < 0) {
            tk_dly_tsk(50);
        }
        tm_printf((UB *)"[AO] recording...\n");
        tk_dly_tsk(100);
        console_flush_input();

        (void)motion_get_encoder_diag(&l, &r);
        ao_e0 = l.hw_edges + r.hw_edges;
        last = -1;
        /* No printing here: it would stretch the poll */
        while(console_try_getc() < 0) {
            s = ao_step();
            if(s >= IR_AO_LEN) {
                break;
            }
            mv = (UH)ir_ao_left_mv();
            /* Fill any steps passed since the last poll with this reading */
            for(k = last + 1; k <= s; k++) {
                prof[k] = mv;
            }
            if(s > last) {
                last = s;
            }
            tk_dly_tsk(1);
        }
        n = last + 1;

        tm_printf((UB *)"\n[AO] Pass %d: ", pass);
        ao_print_mm(n);
        tm_printf((UB *)" mm recorded%s\n", (n >= IR_AO_LEN) ? " (buffer FULL)" : "");
        if(n < 10) {
            tm_printf((UB *)"[AO] too short, push further\n");
            continue;
        }
        ao_analyse(prof, n);

        tm_printf((UB *)"[AO] profile, mV every 1 mm (3 steps):\n");
        for(i = 0; i < n; i += 3 * IR_AO_ROW) {
            tm_printf((UB *)"[AO] %d mm:", (i * SWEEP_UM_PER_EDGE) / 1000);
            for(k = i; (k < (i + (3 * IR_AO_ROW))) && (k < n); k += 3) {
                tm_printf((UB *)" %d", prof[k]);
            }
            tm_printf((UB *)"\n");
        }
    }
}
#elif MOTION_TEST_MODE == MOTION_TEST_SAFETY
/* Phase 1 check (Tribunal 1): car LIFTED, wheels free. Key-paced parts:
   A timing and PI error, B STOP with commands queued, C stall (hold both
   wheels), D encoder plausibility (hold one wheel). Nothing is printed
   while the wheels turn. */
LOCAL UW safe_now_ms(void)
{
    SYSTIM tim;

    tk_get_tim(&tim);
    return tim.lo;
}

LOCAL void safe_wait_key(const char *what)
{
    tm_printf((UB *)"[SAFE] %s Press a key.\n", what);
    console_flush_input();
    while(console_try_getc() < 0) {
        tk_dly_tsk(50);
    }
}

/* Wait until neither wheel has moved for 100 ms (or 3 s); returns ms */
LOCAL UW safe_wait_still(UW t0)
{
    enc_diag_t l, r;
    W last = -1, now;
    UW still = 0;

    while((still < 100u) && ((safe_now_ms() - t0) < 3000u)) {
        (void)motion_get_encoder_diag(&l, &r);
        now = (W)(l.hw_edges + r.hw_edges);
        still = (now == last) ? (still + 5u) : 0u;
        last = now;
        tk_dly_tsk(5);
    }
    return safe_now_ms() - t0 - still;
}

/* Drive at v until a motion fault or limit_ms; returns ms to the fault */
LOCAL UW safe_drive_until_fault(INT v, UW limit_ms)
{
    UW t0 = safe_now_ms();

    motion_set_velocity((int16_t)v, (int16_t)v);
    while(((safe_now_ms() - t0) < limit_ms) && (0u == (fault_get() & FAULT_MOTION_MASK))) {
        tk_dly_tsk(5);
    }
    return safe_now_ms() - t0;
}

LOCAL void safe_report_fault(UW ms)
{
    UB f = fault_get();

    tm_printf((UB *)"[SAFE]   faults 0x%x after %d ms:%s%s%s%s\n", (INT)f, (INT)ms,
              (0u != (f & FAULT_MOTOR_STALL)) ? " MOTOR_STALL" : "",
              (0u != (f & FAULT_ENC_LEFT)) ? " ENC_LEFT" : "",
              (0u != (f & FAULT_ENC_RIGHT)) ? " ENC_RIGHT" : "",
              (0u == (f & FAULT_MOTION_MASK)) ? " none (no trip)" : "");
}

LOCAL void motion_test_safety(void)
{
    static const INT speeds[3] = { 150, 300, 500 };
    enc_diag_t l0, r0, l1, r1;
    INT i, l_mm_s, r_mm_s;
    UW t0, ms;

    tm_printf((UB *)"\n[SAFE] Motion safety and timing test. Car LIFTED, wheels free.\n");

    /* A: loop rate and speed error at three speeds */
    safe_wait_key("Part A: timing and PI error at 150/300/500 mm/s.");
    for(i = 0; i < 3; i++) {
        motion_set_velocity((int16_t)speeds[i], (int16_t)speeds[i]);
        tk_dly_tsk(1500);
        (void)motion_get_encoder_diag(&l0, &r0);
        tk_dly_tsk(1000);
        (void)motion_get_encoder_diag(&l1, &r1);
        motion_set_velocity(0, 0);
        tk_dly_tsk(800);
        l_mm_s = (INT)(((l1.hw_count - l0.hw_count) * SWEEP_UM_PER_EDGE) / 1000);
        r_mm_s = (INT)(((r1.hw_count - r0.hw_count) * SWEEP_UM_PER_EDGE) / 1000);
        tm_printf((UB *)"[SAFE]   %d mm/s: L %d R %d mm/s (error L %d R %d %%), loop %d passes/s\n",
                  speeds[i], l_mm_s, r_mm_s,
                  ((l_mm_s - speeds[i]) * 100) / speeds[i],
                  ((r_mm_s - speeds[i]) * 100) / speeds[i],
                  (INT)motion_loop_rate());
    }

    /* B: STOP overtakes queued commands */
    safe_wait_key("Part B: STOP at 300 mm/s with 3 commands queued behind it.");
    motion_set_velocity(300, 300);
    tk_dly_tsk(1500);
    motion_set_velocity(300, 300);
    motion_set_velocity(300, 300);
    motion_set_velocity(300, 300);
    t0 = safe_now_ms();
    motion_stop();
    ms = safe_wait_still(t0);
    tk_dly_tsk(500);
    (void)motion_get_encoder_diag(&l0, &r0);
    tk_dly_tsk(500);
    (void)motion_get_encoder_diag(&l1, &r1);
    tm_printf((UB *)"[SAFE]   wheels still %d ms after STOP; %s\n", (INT)ms,
              ((l1.hw_edges == l0.hw_edges) && (r1.hw_edges == r0.hw_edges))
              ? "stayed stopped (queued commands discarded)" : "MOVED AGAIN");

    /* C: stall, both wheels held */
    safe_wait_key("Part C: wheels spin at 200 mm/s; after ~1 s grip BOTH wheels"
                  " firmly until they stop.");
    ms = safe_drive_until_fault(200, 10000u);
    motion_set_velocity(0, 0);
    safe_report_fault(ms);
    tk_dly_tsk(800);
    fault_clear(FAULT_MOTION_MASK);

    /* D: one wheel held */
    safe_wait_key("Part D: wheels spin at 200 mm/s; after ~1 s grip only the"
                  " LEFT wheel until it stops.");
    ms = safe_drive_until_fault(200, 10000u);
    motion_set_velocity(0, 0);
    safe_report_fault(ms);
    tk_dly_tsk(800);
    fault_clear(FAULT_MOTION_MASK);

    /* E: no false trips while running free */
    safe_wait_key("Part E: 10 s free running at 300 mm/s, hands off (expect no trip).");
    ms = safe_drive_until_fault(300, 10000u);
    motion_set_velocity(0, 0);
    safe_report_fault(ms);
    fault_clear(FAULT_MOTION_MASK);

    tm_printf((UB *)"\n=== Motion safety test complete ===\n");
}
#elif MOTION_TEST_MODE == MOTION_TEST_CPU
/* CPU and RAM use of the motion task, car LIFTED. Each step holds a state
   for 3 s, then reads the last full second: loop passes, busy time and the
   longest pass, plus the stack high-water mark. Prints only between steps. */
LOCAL void cpu_report(const char *name)
{
    uint32_t busy, max_pass;

    motion_get_cpu(&busy, &max_pass);
    tm_printf((UB *)"[CPU] %s: %d passes/s, busy %d us/s = %d.%d%d %% CPU, "
              "longest pass %d us, %d us per pass on average\n", name,
              (INT)motion_loop_rate(), (INT)busy, (INT)(busy / 10000u),
              (INT)((busy / 1000u) % 10u), (INT)((busy / 100u) % 10u), (INT)max_pass,
              (motion_loop_rate() > 0u) ? (INT)(busy / motion_loop_rate()) : 0);
}

LOCAL void motion_test_cpu(void)
{
    static const INT speeds[3] = { 150, 300, 500 };
    INT i;

    ultrasonic_init();              /* starts the 1 MHz timer the counters use */
    tm_printf((UB *)"\n[CPU] Motion task CPU and stack. Car LIFTED. Starting in 3 s.\n");
    tk_dly_tsk(3000);

    cpu_report("idle (stopped)");
    for(i = 0; i < 3; i++) {
        motion_set_velocity((int16_t)speeds[i], (int16_t)speeds[i]);
        tk_dly_tsk(3000);
        tm_printf((UB *)"[CPU] ");
        tm_printf((UB *)"%d mm/s:\n", speeds[i]);
        cpu_report("  driving");
    }
    motion_set_velocity(0, 0);
    tk_dly_tsk(1500);
    tm_printf((UB *)"[CPU] stack: %d of %d bytes used (deepest so far)\n",
              (INT)motion_stack_used(), (INT)motion_stack_size());
    tm_printf((UB *)"\n=== CPU test complete ===\n");
}
#elif MOTION_TEST_MODE == IMU_TEST_RAMP_ROLL
/* Phase 2 step 1: one-sided hump log. The car drives RAMP_DIST_MM straight
   at RAMP_SPEED_MM_S under the normal PI (no heading hold) with only the
   RIGHT wheel going over the ramp. Every 20 ms it stores roll, pitch,
   heading, both wheels' distance and the horizontal jolt in RAM; the log
   is printed once the car has stopped. The stall/encoder checks stay on
   (no hump mask), to see whether the ramp or the IR bracket trips them. */
typedef struct {
    int16_t d_mm;        /* mean forward distance */
    int16_t l_mm;        /* left wheel distance */
    int16_t r_mm;        /* right wheel distance */
    int16_t roll;        /* x0.01 deg, right side down + */
    int16_t pitch;       /* x0.01 deg, nose up + */
    uint16_t hdg;        /* x0.01 deg clockwise from magnetic north */
    int16_t jolt_mg;     /* larger of |d ax|, |d ay| since last sample */
} ramp_pt_t;

LOCAL INT ramp_wrap_cdeg(INT d)
{
    while(d > 18000) {
        d -= 36000;
    }
    while(d < -18000) {
        d += 36000;
    }
    return d;
}

LOCAL INT ramp_sin_cdeg_x1000(INT cdeg)
{
    /* sin by Taylor to x^7 (no libm), x1000 */
    float x = (float)cdeg * (3.14159265f / 18000.0f);
    float x2 = x * x;

    return (INT)(1000.0f * x * (1.0f - (x2 / 6.0f) * (1.0f - (x2 / 20.0f)
                 * (1.0f - (x2 / 42.0f)))));
}

LOCAL void ramp_pass(const char *name)
{
    static ramp_pt_t log[RAMP_LOG_LEN];
    imu_attitude_t att;
    imu_raw_t s;
    enc_diag_t l, r;
    INT n = 0, i, k, base_n = 0;
    W roll0 = 0, pitch0 = 0, hdg_unwrap = 0;
    INT hdg_first = -1, d, j;
    INT prev_ax = 0, prev_ay = 0;
    BOOL have_prev = FALSE, stopping = FALSE;
    UW t_stop = 0;
    SYSTIM tim;
    INT roll_min = 0, roll_max = 0, i_min = 0, i_max = 0, jolt_max = 0, i_jolt = 0;
    INT h0, h1, enc_yaw;

    /* Rest reference: 0.5 s still */
    for(i = 0; i < 25; i++) {
        if(E_OK == imu_read_attitude(&att, NULL)) {
            roll0 += att.roll_cdeg;
            pitch0 += att.pitch_cdeg;
            if(hdg_first < 0) {
                hdg_first = att.heading_cdeg;
            }
            hdg_unwrap += ramp_wrap_cdeg((INT)att.heading_cdeg - hdg_first);
            base_n++;
        }
        tk_dly_tsk(20);
    }
    if(base_n == 0) {
        tm_printf((UB *)"[RAMP] IMU read failed\n");
        return;
    }
    roll0 /= base_n;
    pitch0 /= base_n;
    h0 = hdg_first + (INT)(hdg_unwrap / base_n);

    fault_clear(FAULT_MOTION_MASK);
    motion_reset_odometry();
    motion_set_velocity(RAMP_SPEED_MM_S, RAMP_SPEED_MM_S);

    while(n < RAMP_LOG_LEN) {
        if(E_OK == imu_read_attitude(&att, &s)) {
            (void)motion_get_encoder_diag(&l, &r);
            log[n].l_mm = (int16_t)((l.hw_count * SWEEP_UM_PER_EDGE) / 1000);
            log[n].r_mm = (int16_t)((r.hw_count * SWEEP_UM_PER_EDGE) / 1000);
            log[n].d_mm = (int16_t)((log[n].l_mm + log[n].r_mm) / 2);
            log[n].roll = (int16_t)(att.roll_cdeg - roll0);
            log[n].pitch = (int16_t)(att.pitch_cdeg - pitch0);
            log[n].hdg = att.heading_cdeg;
            j = 0;
            if(have_prev) {
                j = (s.ax > prev_ax) ? (s.ax - prev_ax) : (prev_ax - s.ax);
                k = (s.ay > prev_ay) ? (s.ay - prev_ay) : (prev_ay - s.ay);
                if(k > j) {
                    j = k;
                }
            }
            prev_ax = s.ax;
            prev_ay = s.ay;
            have_prev = TRUE;
            log[n].jolt_mg = (int16_t)j;
            n++;

            tk_get_tim(&tim);
            if((!stopping) && ((log[n - 1].d_mm >= RAMP_DIST_MM)
                               || (0u != (fault_get() & FAULT_MOTION_MASK)))) {
                motion_set_velocity(0, 0);
                stopping = TRUE;
                t_stop = tim.lo;
            }
            if(stopping && ((tim.lo - t_stop) > 800u)) {
                break;
            }
        }
        tk_dly_tsk(20);
    }
    motion_set_velocity(0, 0);
    tk_dly_tsk(300);

    /* Heading after, averaged like the rest reference */
    hdg_first = -1;
    hdg_unwrap = 0;
    base_n = 0;
    for(i = 0; i < 25; i++) {
        if(E_OK == imu_read_attitude(&att, NULL)) {
            if(hdg_first < 0) {
                hdg_first = att.heading_cdeg;
            }
            hdg_unwrap += ramp_wrap_cdeg((INT)att.heading_cdeg - hdg_first);
            base_n++;
        }
        tk_dly_tsk(20);
    }
    h1 = (base_n > 0) ? (hdg_first + (INT)(hdg_unwrap / base_n)) : h0;

    tm_printf((UB *)"\n[RAMP] %s: %d samples, rest roll %d pitch %d (x0.01 deg), faults 0x%x\n",
              name, n, (INT)roll0, (INT)pitch0, (INT)fault_get());
    tm_printf((UB *)"[RAMP] i,d_mm,l_mm,r_mm,roll,pitch,hdg,jolt_mg  (roll/pitch from rest)\n");
    for(i = 0; i < n; i++) {
        tm_printf((UB *)"[RAMP] %d,%d,%d,%d,%d,%d,%d,%d\n", i, log[i].d_mm, log[i].l_mm,
                  log[i].r_mm, log[i].roll, log[i].pitch, (INT)log[i].hdg, log[i].jolt_mg);
        if(log[i].roll < roll_min) {
            roll_min = log[i].roll;
            i_min = i;
        }
        if(log[i].roll > roll_max) {
            roll_max = log[i].roll;
            i_max = i;
        }
        if(log[i].jolt_mg > jolt_max) {
            jolt_max = log[i].jolt_mg;
            i_jolt = i;
        }
    }

    d = ramp_wrap_cdeg(h1 - h0);
    enc_yaw = (n > 0) ? (INT)(((log[n - 1].l_mm - log[n - 1].r_mm) * 573L * 100L)
                              / (10L * HDG_TRACK_MM)) : 0;
    tm_printf((UB *)"[RAMP] summary %s:\n", name);
    tm_printf((UB *)"[RAMP]   roll range %d .. %d (x0.01 deg) at d=%d / %d mm\n",
              roll_min, roll_max, log[i_min].d_mm, log[i_max].d_mm);
    tm_printf((UB *)"[RAMP]   if roll = asin(h / %d mm): right-up h ~ %d mm\n",
              HDG_TRACK_MM, (HDG_TRACK_MM * ramp_sin_cdeg_x1000(-roll_min)) / 1000);
    tm_printf((UB *)"[RAMP]   compass turned %d (x0.01 deg, + = right), encoders %d"
              " (L-R over track, + = right)\n", d, enc_yaw);
    tm_printf((UB *)"[RAMP]   wheels L %d R %d mm, largest jolt %d mg at d=%d mm\n",
              (n > 0) ? log[n - 1].l_mm : 0, (n > 0) ? log[n - 1].r_mm : 0,
              jolt_max, log[i_jolt].d_mm);
    tm_printf((UB *)"[RAMP]   Now measure how far sideways the car ended up from its"
              " start line (mm, left or right).\n");
}

LOCAL void imu_test_ramp_roll(void)
{
    static const char *const passes[3] = {
        "Pass 1/3: FLAT floor, no ramp",
        "Pass 2/3: SMALL ramp (1.0 cm) under the RIGHT wheel only",
        "Pass 3/3: TALL ramp (2.4 cm) under the RIGHT wheel only",
    };
    INT p;

    i2c1_init();
    if(E_OK != imu_init()) {
        tm_printf((UB *)"[RAMP] imu_init failed, check the IMU\n");
        return;
    }
    tm_printf((UB *)"\n[RAMP] One-sided hump log: %d mm straight at %d mm/s, no heading hold.\n",
              RAMP_DIST_MM, RAMP_SPEED_MM_S);
    tm_printf((UB *)"[RAMP] Mark the start line beside the car. Ramp start ~15 cm ahead of\n");
    tm_printf((UB *)"[RAMP] the RIGHT wheel, lined up so only that wheel climbs it.\n");

    for(p = 0; p < 3; p++) {
        tm_printf((UB *)"\n[RAMP] %s.\n", passes[p]);
        tm_printf((UB *)"[RAMP] Set up, keep the car still, then press a key to drive.\n");
        console_flush_input();
        while(console_try_getc() < 0) {
            tk_dly_tsk(50);
        }
        tk_dly_tsk(1000);           /* hands off */
        ramp_pass(passes[p]);
    }
    tm_printf((UB *)"\n=== One-sided hump log complete ===\n");
}
#elif MOTION_TEST_MODE == MOTION_TEST_STRAIGHT
/* Straight-line drift check: STRAIGHT_RUNS key-paced runs of
   STRAIGHT_DIST_CM at STRAIGHT_SPEED_MM_S. After each run, measure how far
   sideways the car ended up from its start line. The encoders only show
   that both wheels turned the same amount; ground drift is the ruler's job. */
LOCAL void motion_test_straight(void)
{
    enc_diag_t l, r;
    INT run;

    tm_printf((UB *)"\n[STRAIGHT] %d runs of %d cm at %d mm/s on the floor.\n",
              STRAIGHT_RUNS, STRAIGHT_DIST_CM, STRAIGHT_SPEED_MM_S);
    tm_printf((UB *)"[STRAIGHT] Put a straight tape line on the floor beside the LEFT\n");
    tm_printf((UB *)"[STRAIGHT] wheel, car parallel to it. After each run measure the gap\n");
    tm_printf((UB *)"[STRAIGHT] between the tape and the left wheel at the start and end.\n");

    for(run = 1; run <= STRAIGHT_RUNS; run++) {
        tm_printf((UB *)"\n[STRAIGHT] Run %d/%d: line the car up, press a key.\n",
                  run, STRAIGHT_RUNS);
        console_flush_input();
        while(console_try_getc() < 0) {
            tk_dly_tsk(50);
        }
        tk_dly_tsk(1000);           /* hands off */
        (void)motion_move_forward_cm(STRAIGHT_DIST_CM, STRAIGHT_SPEED_MM_S);
        (void)motion_get_encoder_diag(&l, &r);
        tm_printf((UB *)"[STRAIGHT]   wheels L %d R %d mm (L-R %d mm)\n",
                  (INT)((l.hw_count * SWEEP_UM_PER_EDGE) / 1000),
                  (INT)((r.hw_count * SWEEP_UM_PER_EDGE) / 1000),
                  (INT)(((l.hw_count - r.hw_count) * SWEEP_UM_PER_EDGE) / 1000));
        tm_printf((UB *)"[STRAIGHT]   now measure the sideways offset (mm, left or right)\n");
    }
    tm_printf((UB *)"\n=== Straight-line test complete ===\n");
}
#elif MOTION_TEST_MODE == IMU_TEST_SIDE_HUMP
/* Phase 2: the roll-based one-sided hump detector (terrain.c) on the course
   test, SIDE_RUNS runs over the TALL ramp under the RIGHT wheel only. The
   detector runs live and sets the motion task's hump mask on the ramp.
   Each run also checks the IR bracket strike: for every jolt of at least
   STRIKE_MG it finds the lowest wheel speed in the next 250 ms, the same
   test the roam/controller code uses to accept a jolt as an IMPACT
   (wheels below 60 % of the command). Printed after the car stops. */
#define STRIKE_MG                300
#define STRIKE_WIN_MS            250
#define SPD_WIN_MS               100

typedef struct {
    uint16_t t_ms;       /* since the start command */
    int16_t  d_mm;
    int16_t  roll;       /* raw, from rest, x0.01 deg */
    int16_t  jolt_mg;
    uint8_t  on_hump;
} side_pt_t;

/* Mean wheel speed over the SPD_WIN_MS before sample i, % of the command */
LOCAL INT side_v_pct(const side_pt_t *lg, INT i)
{
    INT k = i;

    while((k > 0) && ((INT)(lg[i].t_ms - lg[k].t_ms) < SPD_WIN_MS)) {
        k--;
    }
    if(lg[i].t_ms == lg[k].t_ms) {
        return 100;
    }
    return ((lg[i].d_mm - lg[k].d_mm) * 100000)
           / ((INT)(lg[i].t_ms - lg[k].t_ms) * RAMP_SPEED_MM_S);
}

LOCAL void side_pass(const char *name)
{
    static side_pt_t lg[RAMP_LOG_LEN];
    static side_hump_event_t sides[TERR_MAX_EVENTS];
    static hump_event_t humps[TERR_MAX_EVENTS];
    imu_attitude_t att;
    imu_raw_t s;
    hump_event_t h;
    collision_event_t c;
    enc_diag_t l, r;
    W p0 = 0, r0 = 0;
    INT base_n = 0, n = 0, i, d, ns = 0, nh = 0, nc = 0;
    INT prev_ax = 0, prev_ay = 0, j, k, vmin, v;
    BOOL have_prev = FALSE, stopping = FALSE;
    UW ev, t_stop = 0, t0;
    SYSTIM tim;

    for(i = 0; i < 25; i++) {
        if(E_OK == imu_read_attitude(&att, NULL)) {
            p0 += att.pitch_cdeg;
            r0 += att.roll_cdeg;
            base_n++;
        }
        tk_dly_tsk(20);
    }
    if(base_n == 0) {
        tm_printf((UB *)"[SIDE] IMU read failed\n");
        return;
    }
    p0 /= base_n;
    r0 /= base_n;
    terrain_reset((int16_t)p0, (int16_t)r0);

    fault_clear(FAULT_MOTION_MASK);
    motion_reset_odometry();
    tk_get_tim(&tim);
    t0 = tim.lo;
    motion_set_velocity(RAMP_SPEED_MM_S, RAMP_SPEED_MM_S);

    while(n < RAMP_LOG_LEN) {
        if(E_OK == imu_read_attitude(&att, &s)) {
            (void)motion_get_encoder_diag(&l, &r);
            tk_get_tim(&tim);
            d = (INT)((((l.hw_count + r.hw_count) / 2) * SWEEP_UM_PER_EDGE) / 1000);
            ev = terrain_update(d, &att, &s, motion_is_steady(), &h, &c);
            motion_set_hump_mask(terrain_on_hump());
            if((0u != (ev & TERRAIN_EVT_SIDE_HUMP)) && (ns < TERR_MAX_EVENTS)) {
                terrain_last_side_hump(&sides[ns++]);
            }
            if((0u != (ev & TERRAIN_EVT_HUMP)) && (nh < TERR_MAX_EVENTS)) {
                humps[nh++] = h;
            }
            if(0u != (ev & TERRAIN_EVT_COLLISION)) {
                nc++;
            }

            j = 0;
            if(have_prev) {
                j = (s.ax > prev_ax) ? (s.ax - prev_ax) : (prev_ax - s.ax);
                k = (s.ay > prev_ay) ? (s.ay - prev_ay) : (prev_ay - s.ay);
                if(k > j) {
                    j = k;
                }
            }
            prev_ax = s.ax;
            prev_ay = s.ay;
            have_prev = TRUE;
            lg[n].t_ms = (uint16_t)(tim.lo - t0);
            lg[n].d_mm = (int16_t)d;
            lg[n].roll = (int16_t)(att.roll_cdeg - r0);
            lg[n].jolt_mg = (int16_t)j;
            lg[n].on_hump = terrain_on_hump() ? 1u : 0u;
            n++;

            if((!stopping) && ((d >= RAMP_DIST_MM)
                               || (0u != (fault_get() & FAULT_MOTION_MASK)))) {
                motion_set_velocity(0, 0);
                stopping = TRUE;
                t_stop = tim.lo;
            }
            if(stopping && ((tim.lo - t_stop) > 800u)) {
                break;
            }
        }
        tk_dly_tsk(20);
    }
    motion_set_velocity(0, 0);
    motion_set_hump_mask(FALSE);
    tk_dly_tsk(300);

    tm_printf((UB *)"\n[SIDE] %s: %d samples, rest roll %d (x0.01 deg), faults 0x%x\n",
              name, n, (INT)r0, (INT)fault_get());
    tm_printf((UB *)"[SIDE] i,t_ms,d_mm,roll,jolt_mg,speed_pct,on_hump\n");
    for(i = 0; i < n; i++) {
        tm_printf((UB *)"[SIDE] %d,%d,%d,%d,%d,%d,%d\n", i, (INT)lg[i].t_ms, lg[i].d_mm,
                  lg[i].roll, lg[i].jolt_mg, side_v_pct(lg, i), (INT)lg[i].on_hump);
    }
    tm_printf((UB *)"[SIDE] summary %s:\n", name);
    tm_printf((UB *)"[SIDE]   %d one-sided hump(s):\n", ns);
    for(i = 0; i < ns; i++) {
        tm_printf((UB *)"[SIDE]     %s wheel, %d..%d mm, height ~%d mm"
                  " (30 mm mean roll %d x0.01 deg from %d mm)\n",
                  (0u != sides[i].right_side) ? "RIGHT" : "LEFT",
                  (INT)sides[i].start_mm, (INT)sides[i].end_mm, sides[i].height_mm,
                  sides[i].plateau_cdeg, (INT)sides[i].plateau_mm);
    }
    tm_printf((UB *)"[SIDE]   %d pitch hump(s) reported, %d dropped as one-sided\n",
              nh, (INT)terrain_pitch_humps_dropped());
    for(i = 0; i < nh; i++) {
        tm_printf((UB *)"[SIDE]     pitch hump %d..%d mm, height ~%d mm\n",
                  (INT)humps[i].start_mm, (INT)humps[i].end_mm, humps[i].height_mm);
    }
    tm_printf((UB *)"[SIDE]   largest filtered roll %d, largest jolt %d mg, %d collision(s)\n",
              terrain_max_roll_cdeg(), terrain_max_jolt_mg(), nc);

    /* Bracket strike: would any jolt pass the IMPACT confirmation? */
    for(i = 1; i < n; i++) {
        if(lg[i].jolt_mg < STRIKE_MG) {
            continue;
        }
        vmin = 1000;
        for(k = i; (k < n) && ((INT)(lg[k].t_ms - lg[i].t_ms) <= STRIKE_WIN_MS); k++) {
            v = side_v_pct(lg, k);
            if(v < vmin) {
                vmin = v;
            }
        }
        tm_printf((UB *)"[SIDE]   jolt %d mg at %d mm: wheels down to %d %% within"
                  " %d ms -> %s\n", lg[i].jolt_mg, lg[i].d_mm, vmin, STRIKE_WIN_MS,
                  (vmin < ROAM_JOLT_SLOW_PCT) ? "WOULD count as IMPACT" : "ignored");
    }
    tm_printf((UB *)"[SIDE]   Now measure how far sideways the car ended up"
              " (mm, left or right).\n");
}

LOCAL void imu_test_side_hump(void)
{
    INT run;

    i2c1_init();
    if(E_OK != imu_init()) {
        tm_printf((UB *)"[SIDE] imu_init failed, check the IMU\n");
        return;
    }
    tm_printf((UB *)"\n[SIDE] One-sided hump: %d runs over the TALL ramp (2.4 cm), RIGHT"
              " wheel only, %d mm at %d mm/s.\n", SIDE_RUNS, RAMP_DIST_MM, RAMP_SPEED_MM_S);
    tm_printf((UB *)"[SIDE] Ramp start ~15 cm ahead of the right wheel. Roll the car\n");
    tm_printf((UB *)"[SIDE] forward a few cm by hand first so the castor trails.\n");

    for(run = 1; run <= SIDE_RUNS; run++) {
        tm_printf((UB *)"\n[SIDE] Run %d/%d: set up, keep the car still, press a key.\n",
                  run, SIDE_RUNS);
        console_flush_input();
        while(console_try_getc() < 0) {
            tk_dly_tsk(50);
        }
        tk_dly_tsk(1000);
        side_pass("TALL ramp, right wheel");
    }
    tm_printf((UB *)"\n=== One-sided hump detector test complete ===\n");
}
#elif MOTION_TEST_MODE == OBST_TEST_STOP
/* Phase 3 step 1: obstacle watch task (obstacle.c). Key-paced runs at
   OBS_WATCH_MAX_MM_S toward a target on the path, which must stop the car
   with the nose (IR tip) >= 50 mm from it: runs 1..OBST_BOX_RUNS a box,
   flat face to the car; the rest the round bottle (a weak, narrow echo).
   Every ping is traced with the distance travelled and printed after the
   stop. Nothing is printed while driving. */
LOCAL INT obst_dist_mm(void)
{
    enc_diag_t l, r;

    (void)motion_get_encoder_diag(&l, &r);
    return (INT)((((l.hw_count + r.hw_count) / 2) * SWEEP_UM_PER_EDGE) / 1000);
}

LOCAL void obst_run(INT run, BOOL box)
{
    static obstacle_trace_t tr[OBS_TRACE_LEN];
    INT d_trig = -1, mm_trig = -1, d_end, i, n = 0, nt;
    W sum = 0;
    BOOL stopped = FALSE;

    obstacle_watch(TRUE);
    tk_dly_tsk(300);                 /* a few pings of history first */
    motion_reset_odometry();
    obstacle_trace_start();
    motion_set_velocity(OBS_WATCH_MAX_MM_S, OBS_WATCH_MAX_MM_S);

    while(obst_dist_mm() < OBST_MAX_DIST_MM) {
        if(obstacle_blocked()) {
            motion_stop();
            d_trig = obst_dist_mm();
            mm_trig = obstacle_last_mm();
            stopped = TRUE;
            break;
        }
        if(0u != (fault_get() & (FAULT_MOTION_MASK | FAULT_NO_ECHO))) {
            motion_stop();
            break;
        }
        tk_dly_tsk(5);
    }
    if(!stopped) {
        motion_stop();
    }
    tk_dly_tsk(800);
    d_end = obst_dist_mm();

    /* Reading at rest: mean of 5 good pings */
    for(i = 0; (i < 12) && (n < 5); i++) {
        tk_dly_tsk(70);
        if(obstacle_last_mm() > 0) {
            sum += obstacle_last_mm();
            n++;
        }
    }
    obstacle_watch(FALSE);
    nt = (INT)obstacle_trace_get(tr);

    tm_printf((UB *)"[OBST] run %d pings (d_mm,reading_mm,near_bits; -1 no echo,"
              " -2 dead):", run);
    for(i = 0; i < nt; i++) {
        if((i % 6) == 0) {
            tm_printf((UB *)"\n[OBST]  ");
        }
        tm_printf((UB *)" %d,%d,%d%d%d", tr[i].d_mm, tr[i].mm,
                  (tr[i].hist >> 2) & 1, (tr[i].hist >> 1) & 1, tr[i].hist & 1);
    }
    tm_printf((UB *)"\n");
    tm_printf((UB *)"[OBST] run %d (%s): ", run, box ? "box" : "bottle");
    if(stopped) {
        tm_printf((UB *)"STOPPED, triggered at %d mm travelled with reading %d mm;"
                  " at rest after %d mm, reading %d mm (braking %d mm)\n",
                  d_trig, mm_trig, d_end, (n > 0) ? (INT)(sum / n) : -1, d_end - d_trig);
    } else {
        tm_printf((UB *)"no stop, drove %d mm%s\n", d_end,
                  (0u != fault_get()) ? " (FAULT)" : "");
    }
    tm_printf((UB *)"[OBST]   faults 0x%x, pings %d. %s\n", (INT)fault_get(),
              (INT)obstacle_ping_count(),
              "Measure the gap from the nose (IR tip) to the target (mm).");
}

LOCAL void obst_test_stop(void)
{
    INT run;
    ER er;

    er = obstacle_task_create();
    if(E_OK != er) {
        tm_printf((UB *)"[OBST] obstacle_task_create failed: %d\n", er);
        return;
    }
    tm_printf((UB *)"\n[OBST] Obstacle watch: stop when 2 of the last 3 pings <= %d mm"
              " (raw), driving at %d mm/s.\n", OBS_STOP_MM, OBS_WATCH_MAX_MM_S);
    tm_printf((UB *)"[OBST] Runs 1-%d: a BOX, flat face to the car, ~60-100 cm ahead"
              " on the path. Runs %d-%d: the round BOTTLE instead.\n", OBST_BOX_RUNS,
              OBST_BOX_RUNS + 1, OBST_RUNS);

    for(run = 1; run <= OBST_RUNS; run++) {
        tm_printf((UB *)"\n[OBST] Run %d/%d: set up (%s), press a key.\n", run, OBST_RUNS,
                  (run <= OBST_BOX_RUNS) ? "box, flat face to the car" : "round bottle");
        console_flush_input();
        while(console_try_getc() < 0) {
            tk_dly_tsk(50);
        }
        tk_dly_tsk(1000);
        fault_clear(FAULT_MOTION_MASK);
        obst_run(run, (run <= OBST_BOX_RUNS) ? TRUE : FALSE);
    }
    tm_printf((UB *)"\n=== Obstacle watch test complete ===\n");
}
#elif MOTION_TEST_MODE == DEMO_STOP
/* Obstacle demo version 1 (course): drive forward at DEMOSTOP_MM_S with
   the obstacle watch on, stop at the block (>= 5 cm nose gap; measured
   12.7-15.0 cm on a flat face at 300 mm/s), stay stopped. A key starts
   each run; a key while driving stops it too. Faults stop the car. */
LOCAL INT demostop_dist_mm(void)
{
    enc_diag_t l, r;

    (void)motion_get_encoder_diag(&l, &r);
    return (INT)((((l.hw_count + r.hw_count) / 2) * SWEEP_UM_PER_EDGE) / 1000);
}

LOCAL void demo_stop(void)
{
    INT run, d_trig, mm_trig, d_end;
    UB why;
    ER er;

    er = obstacle_task_create();
    if(E_OK != er) {
        tm_printf((UB *)"[STOP] obstacle_task_create failed: %d\n", er);
        return;
    }
    tm_printf((UB *)"\n[STOP] Obstacle demo v1: drive at %d mm/s, stop at the block,"
              " stay stopped.\n", DEMOSTOP_MM_S);

    for(run = 1; ; run++) {
        tm_printf((UB *)"\n[STOP] Run %d: point the car at the block, press a key to go"
                  " (any key while driving stops it).\n", run);
        console_flush_input();
        while(console_try_getc() < 0) {
            tk_dly_tsk(50);
        }
        tk_dly_tsk(1000);
        console_flush_input();
        fault_clear(FAULT_MOTION_MASK);

        obstacle_watch(TRUE);
        tk_dly_tsk(300);
        motion_reset_odometry();
        motion_set_velocity(DEMOSTOP_MM_S, DEMOSTOP_MM_S);

        why = 0u;
        d_trig = 0;
        mm_trig = -1;
        while(0u == why) {
            if(obstacle_blocked()) {
                why = 1u;
            } else if(0u != (fault_get() & (FAULT_MOTION_MASK | FAULT_NO_ECHO))) {
                why = 2u;
            } else if(console_try_getc() >= 0) {
                why = 3u;
            } else if(demostop_dist_mm() >= DEMOSTOP_MAX_MM) {
                why = 4u;
            } else {
                tk_dly_tsk(5);
            }
        }
        motion_stop();
        d_trig = demostop_dist_mm();
        mm_trig = obstacle_last_mm();
        tk_dly_tsk(800);
        d_end = demostop_dist_mm();
        obstacle_watch(FALSE);

        tm_printf((UB *)"[STOP] %s after %d mm (sensor %d mm), at rest after %d mm,"
                  " faults 0x%x\n",
                  (1u == why) ? "OBSTACLE: stopped" : (2u == why) ? "FAULT: stopped"
                  : (3u == why) ? "KEY: stopped" : "NOTHING FOUND: stopped at the limit",
                  d_trig, mm_trig, d_end, (INT)fault_get());
    }
}
#elif MOTION_TEST_MODE == SCAN_TEST_ABORT
/* Phase 3 step 2: the scan runs in the obstacle task and can be stopped.
   Car on the floor with ~20 cm clear around it, a box ~30-40 cm ahead.
   A: a full scan (time and profile). B: abort mid-scan with STOP, time
   until the wheels are still and until the scan hands back. C: a
   too-short timeout, which must raise FAULT_SCAN_TIMEOUT and stop. */
LOCAL UW scanab_now_ms(void)
{
    SYSTIM tim;

    tk_get_tim(&tim);
    return tim.lo;
}

/* ms from t0 until neither wheel has moved for 20 ms (5 ms steps) */
LOCAL UW scanab_still_ms(UW t0)
{
    enc_diag_t l, r;
    W last = -1, now;
    UW still = 0;

    while((still < 20u) && ((scanab_now_ms() - t0) < 3000u)) {
        (void)motion_get_encoder_diag(&l, &r);
        now = (W)(l.hw_edges + r.hw_edges);
        still = (now == last) ? (still + 5u) : 0u;
        last = now;
        tk_dly_tsk(5);
    }
    return scanab_now_ms() - t0 - still;
}

LOCAL void scanab_key(const char *what)
{
    tm_printf((UB *)"\n[SCANAB] %s Press a key.\n", what);
    console_flush_input();
    while(console_try_getc() < 0) {
        tk_dly_tsk(50);
    }
    tk_dly_tsk(1000);
}

LOCAL void scan_test_abort(void)
{
    obstacle_profile_t prof;
    scan_timing_t tmg;
    UW t0, t_still, t_done;
    INT arun;
    ER er;

    er = obstacle_task_create();
    if(E_OK != er) {
        tm_printf((UB *)"[SCANAB] obstacle_task_create failed: %d\n", er);
        return;
    }
    tm_printf((UB *)"\n[SCANAB] Scan safety test. Car on the floor, ~20 cm clear around"
              " it, a box ~30-40 cm ahead.\n");

    /* A: normal scans, timed by phase */
    for(arun = 1; arun <= SCANAB_A_RUNS; arun++) {
        tm_printf((UB *)"\n[SCANAB] Part A run %d/%d (move the box between runs).", arun,
                  SCANAB_A_RUNS);
        scanab_key("Part A: one full scan (+/-45 deg), up to 90 s, timed by phase.");
        t0 = scanab_now_ms();
        (void)obstacle_scan_start(SCAN_HALF_DEG, SCAN_DETECT_MM);
        er = obstacle_scan_wait(SCANAB_LONG_MS, &prof);
        tm_printf((UB *)"[SCANAB]   result %d after %d ms\n", er, (INT)(scanab_now_ms() - t0));
        if((E_OK == er) && prof.found) {
            tm_printf((UB *)"[SCANAB]   nearest %d mm at %d cdeg, sides %d .. %d mm\n",
                      prof.nearest_mm, prof.nearest_cdeg, prof.right_side_mm,
                      prof.left_side_mm);
        }
        scan_get_timing(&tmg);
        tm_printf((UB *)"[SCANAB]   time: turn to start %d ms, sweep %d ms (%d pings),"
                  " return %d ms\n", (INT)tmg.start_ms, (INT)tmg.coarse_ms,
                  (INT)tmg.coarse_samples, (INT)tmg.return_ms);
        tm_printf((UB *)"[SCANAB]   fine right edge %d ms, %d steps covering %d cdeg;"
                  " left edge %d ms, %d steps covering %d cdeg\n",
                  (INT)tmg.fine_ms[0], (INT)tmg.fine_steps[0], tmg.fine_span_cdeg[0],
                  (INT)tmg.fine_ms[1], (INT)tmg.fine_steps[1], tmg.fine_span_cdeg[1]);
        fault_clear(FAULT_SCAN_TIMEOUT);
    }

    /* B: STOP mid-scan */
    scanab_key("Part B: a scan starts; after 3 s it is stopped (abort + STOP).");
    (void)obstacle_scan_start(SCAN_HALF_DEG, SCAN_DETECT_MM);
    tk_dly_tsk(3000);
    t0 = scanab_now_ms();
    obstacle_scan_abort();
    motion_stop();
    t_still = scanab_still_ms(t0);
    er = obstacle_scan_wait(OBS_ABORT_WAIT_MS, NULL);
    t_done = scanab_now_ms() - t0;
    tm_printf((UB *)"[SCANAB]   wheels still %d ms after STOP; scan handed back after"
              " %d ms with %d (%s)\n", (INT)t_still, (INT)t_done, er,
              (E_RLWAI == er) ? "aborted, as expected" : "UNEXPECTED");

    /* C: timeout */
    scanab_key("Part C: a scan with a 4 s timeout (too short on purpose).");
    fault_clear(FAULT_SCAN_TIMEOUT);
    (void)obstacle_scan_start(SCAN_HALF_DEG, SCAN_DETECT_MM);
    t0 = scanab_now_ms();
    er = obstacle_scan_wait(4000u, NULL);
    tm_printf((UB *)"[SCANAB]   wait returned %d after %d ms, faults 0x%x (%s)\n", er,
              (INT)(scanab_now_ms() - t0), (INT)fault_get(),
              ((E_TMOUT == er) && (0u != (fault_get() & FAULT_SCAN_TIMEOUT)))
              ? "timeout fault raised, wheels stopped" : "UNEXPECTED");
    fault_clear(FAULT_SCAN_TIMEOUT);

    tm_printf((UB *)"\n=== Scan safety test complete ===\n");
}
#elif MOTION_TEST_MODE == DEMO_AVOID
/* Obstacle demo version 2 (course): drive with the watch on, stop at the
   block, scan +/-45 deg, then go round it on the side that needs the
   smaller sideways shift, as tight as the scan allows, and come back onto
   the original path with 90 deg turns on the spot:

        turn 90 toward the side    -> sideways: shift
        turn back 90 (parallel)    -> straight past the block's back
        turn 90 back toward path   -> sideways: shift
        turn back 90 (original heading) -> carry on driving

   90 deg legs keep the car's side parallel to the block, so the gap can be
   small: shift = block side + the car's half width on that side +
   AVOID_GAP_MM. Each turn
   is checked on the encoders and trimmed if it missed. A 45 deg diagonal would
   swing the car's front corner across the block's near corner at these
   distances. The block's depth is not measured (the sonar only sees its
   front), so AVOID_DEPTH_MM is assumed. The sideways legs watch in the
   direction of travel; the straight beside the block does not (the block
   is ~50 mm off the car's side and would trip it). Until Buddy 3 can find
   the line, "back on the path" means the original line by encoders. */
LOCAL INT avoid_dist_mm(void)
{
    enc_diag_t l, r;

    (void)motion_get_encoder_diag(&l, &r);
    return (INT)((((l.hw_count + r.hw_count) / 2) * SWEEP_UM_PER_EDGE) / 1000);
}

/* Turn by cdeg on the spot, then one small correcting turn if the
   encoders say it missed by more than AVOID_TRIM_CDEG. Returns the angle
   actually turned. */
LOCAL INT avoid_turn(INT cdeg)
{
    INT got = (INT)motion_turn_cdeg(cdeg, AVOID_TURN_MM_S);

    if(((cdeg - got) > AVOID_TRIM_CDEG) || ((got - cdeg) > AVOID_TRIM_CDEG)) {
        got += (INT)motion_turn_cdeg(cdeg - got, AVOID_TRIM_MM_S);
    }
    return got;
}

/* Drive mm forward with the watch on. Returns TRUE if it got there,
   FALSE if blocked, faulted or a key was pressed (car stopped). */
LOCAL BOOL avoid_leg(INT mm, BOOL watch)
{
    INT d0 = avoid_dist_mm();
    BOOL ok = TRUE;

    if(watch) {
        obstacle_watch(TRUE);
        tk_dly_tsk(200);
    }
    motion_set_velocity(AVOID_LEG_MM_S, AVOID_LEG_MM_S);
    while((avoid_dist_mm() - d0) < (mm - AVOID_LEG_BRAKE_MM)) {
        if((watch && obstacle_blocked())
           || (0u != (fault_get() & FAULT_MOTION_MASK))
           || (console_try_getc() >= 0)) {
            ok = FALSE;
            break;
        }
        tk_dly_tsk(5);
    }
    motion_stop();
    tk_dly_tsk(400);
    if(watch) {
        obstacle_watch(FALSE);
    }
    return ok;
}

LOCAL void demo_avoid(void)
{
    obstacle_profile_t prof;
    INT run, shift_l, shift_r, shift, straight, d_stop, side, t1;
    UB why;
    ER er;

    er = obstacle_task_create();
    if(E_OK != er) {
        tm_printf((UB *)"[AVOID] obstacle_task_create failed: %d\n", er);
        return;
    }
    tm_printf((UB *)"\n[AVOID] Obstacle demo v2: drive, stop at the block, scan, go round"
              " it, back onto the original path.\n");
    tm_printf((UB *)"[AVOID] Needs ~60 cm clear on each side of the block. Assumed block"
              " depth %d mm.\n", AVOID_DEPTH_MM);

    for(run = 1; ; run++) {
        tm_printf((UB *)"\n[AVOID] Run %d: point the car at the block, press a key"
                  " (any key while moving stops it).\n", run);
        console_flush_input();
        while(console_try_getc() < 0) {
            tk_dly_tsk(50);
        }
        tk_dly_tsk(1000);
        console_flush_input();
        fault_clear(FAULT_MOTION_MASK | FAULT_SCAN_TIMEOUT);

        /* 1: approach and stop (as demo v1) */
        obstacle_watch(TRUE);
        tk_dly_tsk(300);
        motion_reset_odometry();
        motion_set_velocity(DEMOSTOP_MM_S, DEMOSTOP_MM_S);
        why = 0u;
        while(0u == why) {
            if(obstacle_blocked()) {
                why = 1u;
            } else if(0u != (fault_get() & (FAULT_MOTION_MASK | FAULT_NO_ECHO))) {
                why = 2u;
            } else if(console_try_getc() >= 0) {
                why = 3u;
            } else if(avoid_dist_mm() >= DEMOSTOP_MAX_MM) {
                why = 4u;
            } else {
                tk_dly_tsk(5);
            }
        }
        motion_stop();
        tk_dly_tsk(800);
        obstacle_watch(FALSE);
        d_stop = avoid_dist_mm();
        if(1u != why) {
            tm_printf((UB *)"[AVOID] stopped without a block (%s) after %d mm\n",
                      (2u == why) ? "fault" : (3u == why) ? "key" : "nothing found",
                      d_stop);
            continue;
        }

        /* 2: scan */
        (void)obstacle_scan_start(SCAN_HALF_DEG, SCAN_DETECT_MM);
        er = obstacle_scan_wait(OBS_SCAN_TIMEOUT_MS, &prof);
        if((E_OK != er) || (!prof.found)) {
            tm_printf((UB *)"[AVOID] scan %s (%d): staying stopped\n",
                      (E_OK == er) ? "found nothing" : "failed", er);
            continue;
        }

        /* 3: pick the side needing the smaller shift (left positive) */
        /* Passing left puts the block on the car's right side, and the
           other way round */
        shift_l = prof.left_side_mm + AVOID_HALF_R_MM + AVOID_GAP_MM;
        shift_r = AVOID_HALF_L_MM + AVOID_GAP_MM - prof.right_side_mm;
        if(shift_l < 0) {
            shift_l = 0;
        }
        if(shift_r < 0) {
            shift_r = 0;
        }
        side = (shift_l <= shift_r) ? 1 : -1;
        shift = (side > 0) ? shift_l : shift_r;
        /* Sideways legs do not move the car forward, so the straight leg
           runs from here (axle) past the block's back by AVOID_PAST_MM */
        straight = prof.nearest_mm + AVOID_SENSOR_AHEAD_MM + AVOID_DEPTH_MM
                 + AVOID_PAST_MM;
        tm_printf((UB *)"[AVOID] block %d mm ahead, sides %d .. %d mm: passing %s,"
                  " sideways %d mm, straight %d mm\n", prof.nearest_mm,
                  prof.right_side_mm, prof.left_side_mm,
                  (side > 0) ? "LEFT" : "RIGHT", shift, straight);

        /* 4: go round; prints only once the car has stopped */
        t1 = avoid_turn(side * 9000);
        if(!avoid_leg(shift, TRUE)) {
            tm_printf((UB *)"[AVOID] stopped on the sideways leg\n");
            continue;
        }
        t1 += avoid_turn(-side * 9000);
        if(!avoid_leg(straight, FALSE)) {
            tm_printf((UB *)"[AVOID] stopped on the straight\n");
            continue;
        }
        t1 += avoid_turn(-side * 9000);
        if(!avoid_leg(shift, TRUE)) {
            tm_printf((UB *)"[AVOID] stopped on the way back to the path\n");
            continue;
        }
        t1 += avoid_turn(side * 9000);
        tm_printf((UB *)"[AVOID] back on the original path; net heading error %s%d.%d deg"
                  " (encoders, left +)\n", (t1 < 0) ? "-" : "",
                  ((t1 < 0) ? -t1 : t1) / 100, ((t1 < 0) ? -t1 : t1) % 100 / 10);

        /* 5: carry on along the path */
        if(avoid_leg(AVOID_AFTER_MM, TRUE)) {
            tm_printf((UB *)"[AVOID] drove %d mm past: run complete\n", AVOID_AFTER_MM);
        } else {
            tm_printf((UB *)"[AVOID] stopped after the manoeuvre (blocked or key)\n");
        }
    }
}
#elif MOTION_TEST_MODE == LINE_TEST_FOLLOW
/* Buddy 3 line following, step 1: key-paced runs on the joined straight +
   cross mats. Each run follows the line at LINE_TEST_MM_S and stops after
   LINE_TEST_MAX_MM of travel (wheels stay on the raised mats), or when the
   line is lost and the on-spot search fails, or on a key. The 20 ms trace
   is printed afterwards: d_mm, bits L C R, steering corr and held trim
   (mm/s, + = left), s = searching. */
LOCAL void line_test_follow(void)
{
    static line_trace_t tr[LINE_TRACE_LEN];
    static const char *const end_txt[4] = {
        "running", "distance limit reached", "LINE LOST (search failed)", "stopped"
    };
    INT run, n, i, offc = 0;
    ER er;

    er = line_task_create();
    if(E_OK != er) {
        tm_printf((UB *)"[LINE] line_task_create failed: %d\n", er);
        return;
    }
    tm_printf((UB *)"\n[LINE] Line following on the straight + cross mats: %d mm/s,"
              " stops after %d mm.\n", LINE_TEST_MM_S, LINE_TEST_MAX_MM);
    tm_printf((UB *)"[LINE] Suggested runs: centred; ~5 mm off to the left; ~5 mm off to"
              " the right; ~10 deg crooked each way; just off the line.\n");

    for(run = 1; ; run++) {
        tm_printf((UB *)"\n[LINE] Run %d: place the car at the start of the straight mat,"
                  " press a key (any key while running stops it).\n", run);
        console_flush_input();
        while(console_try_getc() < 0) {
            tk_dly_tsk(50);
        }
        tk_dly_tsk(1000);
        console_flush_input();
        fault_clear(FAULT_MOTION_MASK);

        (void)line_follow_start(LINE_TEST_MM_S, LINE_TEST_MAX_MM);
        tk_dly_tsk(50);
        while(LINE_END_NONE == line_follow_end()) {
            if(console_try_getc() >= 0) {
                line_follow_stop();
            }
            tk_dly_tsk(20);
        }
        tk_dly_tsk(500);

        n = (INT)line_trace_get(tr);
        offc = 0;
        tm_printf((UB *)"[LINE] run %d trace (d_mm,LCR,corr,periods[,s]):", run);
        for(i = 0; i < n; i++) {
            if((i % 5) == 0) {
                tm_printf((UB *)"\n[LINE]  ");
            }
            tm_printf((UB *)" %d,%d%d%d,%d,%d%s", tr[i].d_mm,
                      (0u != (tr[i].bits & IR_BIT_LEFT)) ? 1 : 0,
                      (0u != (tr[i].bits & IR_BIT_CENTRE)) ? 1 : 0,
                      (0u != (tr[i].bits & IR_BIT_RIGHT)) ? 1 : 0,
                      tr[i].corr, tr[i].trim, (0u != tr[i].searching) ? ",s" : "");
            /* Edge following toggles the centre on every crossing; count
               the periods spent with the right spot on the line instead */
            if((0u == tr[i].searching) && (0u == (tr[i].bits & IR_BIT_CENTRE))
               && (0u != (tr[i].bits & IR_BIT_RIGHT))) {
                offc++;
            }
        }
        tm_printf((UB *)"\n[LINE] run %d: %s after %d periods; right spot only on the line in %d;"
                  " recoveries so far %d; faults 0x%x\n", run,
                  end_txt[line_follow_end() & 3u], n, offc, (INT)line_recoveries(),
                  (INT)fault_get());
        tm_printf((UB *)"[LINE]   Note where the car ended up relative to the line.\n");
    }
}
#elif MOTION_TEST_MODE == LINE_TEST_RAMP
/* Course scenario 1: the one-sided ramp while line following. The line
   task follows the line on the straight mat at LINE_TEST_MM_S with the
   TALL ramp (2.4 cm) under the RIGHT wheel only. This task samples the
   IMU every 20 ms, feeds the roll-based hump detector (terrain.c) and
   sets the motion task's hump mask while the car is on the ramp, so the
   stall/encoder checks do not trip on the climb. Run 1 is a flat control
   (no ramp). While on the ramp the line task drives straight on the
   encoders (line_set_hold) and picks the line up 50 mm after it; driving
   with the hump mask on for > 4 s is a stall fault (stuck on the ramp).
   Pass (user 2026-10-24): rides up and down the ramp straight, then finds
   and follows the line; the run ends on the distance limit with no faults;
   the ramp is reported once as a RIGHT one-sided hump of ~24 mm. */
#define RL_LOG_LEN               ((INT)LINE_TRACE_LEN)
#define RL_AFTER_MM              100   /* line check window past the hump */

typedef struct {
    int16_t d_mm;
    int16_t roll;        /* raw, from rest, x0.01 deg */
    uint8_t on_hump;
} rl_pt_t;

LOCAL void line_test_ramp(void)
{
    static line_trace_t tr[LINE_TRACE_LEN];
    static rl_pt_t lg[RL_LOG_LEN];
    static side_hump_event_t sides[TERR_MAX_EVENTS];
    static const char *const end_txt[4] = {
        "running", "distance limit reached", "LINE LOST (search failed)", "stopped"
    };
    imu_attitude_t att;
    imu_raw_t s;
    hump_event_t h;
    collision_event_t c;
    enc_diag_t l, r;
    W p0, r0;
    INT run, n, nt, i, d, ns, nh, nc, base_n, white0, white_max, srch, lo, hi;
    UW ev;
    ER er;

    i2c1_init();
    if(E_OK != imu_init()) {
        tm_printf((UB *)"[RLINE] imu_init failed, check the IMU\n");
        return;
    }
    er = line_task_create();
    if(E_OK != er) {
        tm_printf((UB *)"[RLINE] line_task_create failed: %d\n", er);
        return;
    }
    tm_printf((UB *)"\n[RLINE] One-sided ramp while line following: %d mm/s, stops after"
              " %d mm.\n", LINE_TEST_MM_S, LINE_TEST_MAX_MM);
    tm_printf((UB *)"[RLINE] Run 1: flat (no ramp), as a control. Runs 2-6: TALL ramp under"
              " the RIGHT wheel only, its start ~10 cm ahead of the wheel.\n");
    tm_printf((UB *)"[RLINE] Start with the wheels at the mat edge, centre IR on the line's"
              " right edge.\n");

    for(run = 1; ; run++) {
        tm_printf((UB *)"\n[RLINE] Run %d (%s): set up, keep the car still, press a key"
                  " (any key while running stops it).\n", run,
                  (run == 1) ? "flat" : "TALL ramp, right wheel");
        console_flush_input();
        while(console_try_getc() < 0) {
            tk_dly_tsk(50);
        }
        tk_dly_tsk(1000);
        console_flush_input();
        fault_clear(FAULT_MOTION_MASK);

        /* Rest attitude: the zero for roll and pitch */
        p0 = 0;
        r0 = 0;
        base_n = 0;
        for(i = 0; i < 25; i++) {
            if(E_OK == imu_read_attitude(&att, NULL)) {
                p0 += att.pitch_cdeg;
                r0 += att.roll_cdeg;
                base_n++;
            }
            tk_dly_tsk(20);
        }
        if(base_n == 0) {
            tm_printf((UB *)"[RLINE] IMU read failed\n");
            continue;
        }
        p0 /= base_n;
        r0 /= base_n;
        terrain_reset((int16_t)p0, (int16_t)r0);

        n = 0;
        ns = 0;
        nh = 0;
        nc = 0;
        (void)line_follow_start(LINE_TEST_MM_S, LINE_TEST_MAX_MM);
        tk_dly_tsk(20);
        while(LINE_END_NONE == line_follow_end()) {
            if(console_try_getc() >= 0) {
                line_follow_stop();
            }
            if(E_OK == imu_read_attitude(&att, &s)) {
                (void)motion_get_encoder_diag(&l, &r);
                d = (INT)((((l.hw_count + r.hw_count) / 2) * SWEEP_UM_PER_EDGE) / 1000);
                ev = terrain_update(d, &att, &s, motion_is_steady(), &h, &c);
                motion_set_hump_mask(terrain_on_hump());
                line_set_hold(terrain_on_hump());
                if((0u != (ev & TERRAIN_EVT_SIDE_HUMP)) && (ns < TERR_MAX_EVENTS)) {
                    terrain_last_side_hump(&sides[ns++]);
                }
                if(0u != (ev & TERRAIN_EVT_HUMP)) {
                    nh++;
                }
                if(0u != (ev & TERRAIN_EVT_COLLISION)) {
                    nc++;
                }
                /* Log only when the car has moved (a search or a stop
                   would otherwise fill the log) */
                if((n < RL_LOG_LEN)
                   && ((n == 0) || (d >= lg[n - 1].d_mm + 2) || (d <= lg[n - 1].d_mm - 2)
                       || ((terrain_on_hump() ? 1u : 0u) != lg[n - 1].on_hump))) {
                    lg[n].d_mm = (int16_t)d;
                    lg[n].roll = (int16_t)(att.roll_cdeg - r0);
                    lg[n].on_hump = terrain_on_hump() ? 1u : 0u;
                    n++;
                }
            }
            tk_dly_tsk(20);
        }
        motion_set_hump_mask(FALSE);
        line_set_hold(FALSE);
        tk_dly_tsk(500);

        /* IMU log: d_mm,roll[,H while on the hump] */
        tm_printf((UB *)"[RLINE] run %d IMU (d_mm,roll x0.01 deg[,H = on hump]):", run);
        for(i = 0; i < n; i++) {
            if((i % 8) == 0) {
                tm_printf((UB *)"\n[RLINE]  ");
            }
            tm_printf((UB *)" %d,%d%s", lg[i].d_mm, lg[i].roll,
                      (0u != lg[i].on_hump) ? ",H" : "");
        }
        nt = (INT)line_trace_get(tr);
        tm_printf((UB *)"\n[RLINE] run %d line trace (d_mm,LCR,corr,run[,s = search"
                  " every 200 ms, h = ramp hold]):", run);
        for(i = 0; i < nt; i++) {
            if((i % 5) == 0) {
                tm_printf((UB *)"\n[RLINE]  ");
            }
            tm_printf((UB *)" %d,%d%d%d,%d,%d%s", tr[i].d_mm,
                      (0u != (tr[i].bits & IR_BIT_LEFT)) ? 1 : 0,
                      (0u != (tr[i].bits & IR_BIT_CENTRE)) ? 1 : 0,
                      (0u != (tr[i].bits & IR_BIT_RIGHT)) ? 1 : 0,
                      tr[i].corr, tr[i].trim,
                      (LINE_TR_SEARCH == tr[i].searching) ? ",s"
                      : ((LINE_TR_HOLD == tr[i].searching) ? ",h" : ""));
        }

        tm_printf((UB *)"\n[RLINE] run %d summary: %s, faults 0x%x, recoveries so far %d\n",
                  run, end_txt[line_follow_end() & 3u], (INT)fault_get(),
                  (INT)line_recoveries());
        tm_printf((UB *)"[RLINE]   %d one-sided hump(s):\n", ns);
        for(i = 0; i < ns; i++) {
            tm_printf((UB *)"[RLINE]     %s wheel, %d..%d mm, height ~%d mm\n",
                      (0u != sides[i].right_side) ? "RIGHT" : "LEFT",
                      (INT)sides[i].start_mm, (INT)sides[i].end_mm, sides[i].height_mm);
        }
        tm_printf((UB *)"[RLINE]   %d pitch hump(s), %d dropped as one-sided; largest roll"
                  " %d x0.01 deg; largest jolt %d mg; %d collision event(s)\n",
                  nh, (INT)terrain_pitch_humps_dropped(), terrain_max_roll_cdeg(),
                  terrain_max_jolt_mg(), nc);

        /* Line held over the ramp? Longest stretch with the centre on white
           (normal edge following crosses white in short bits) and any
           on-spot search, from the hump start to RL_AFTER_MM past its end */
        for(i = 0; i < ns; i++) {
            lo = (INT)sides[i].start_mm;
            hi = (INT)sides[i].end_mm + RL_AFTER_MM;
            white0 = -1;
            white_max = 0;
            srch = 0;
            for(d = 0; d < nt; d++) {
                if((tr[d].d_mm < lo) || (tr[d].d_mm > hi)) {
                    continue;
                }
                if(LINE_TR_SEARCH == tr[d].searching) {
                    srch++;
                }
                if(LINE_TR_HOLD == tr[d].searching) {
                    white0 = -1;
                    continue;
                }
                if(0u == (tr[d].bits & (IR_BIT_CENTRE | IR_BIT_RIGHT))) {
                    if(white0 < 0) {
                        white0 = tr[d].d_mm;
                    }
                    if((tr[d].d_mm - white0) > white_max) {
                        white_max = tr[d].d_mm - white0;
                    }
                } else {
                    white0 = -1;
                }
            }
            tm_printf((UB *)"[RLINE]   hump %d (%d..%d mm, held straight on it): longest"
                      " centre-white after it %d mm (lost at > 40), %d search sample(s)\n",
                      i + 1, lo, hi, white_max, srch);
        }
        tm_printf((UB *)"[RLINE]   Note where the car ended up relative to the line, and"
                  " whether the bracket touched the ramp.\n");
    }
}
#elif MOTION_TEST_MODE == CTRL_TEST_COURSE
/* Vehicle controller (controller/vehicle_ctrl.c), key-paced course runs:
     l  line following + one-sided ramp (no obstacle watch)
     o  line following + obstacle stop (block on the line, >= 5 cm gap)
     b  line following + barcode on the straight, command at the cross
        (turns stop the run: the test mats have nothing past the cross)
   The controller runs everything; this task only starts runs, samples
   robot_status every 500 ms like Buddy 1's telemetry will (kept, printed
   after the run: no printing while the wheels turn), and prints the
   controller's event log. Impacts are logged, not acted on, until the
   bracket's ramp strike has been checked against the rule. */
#define CTRLT_MM_S               100
#define CTRLT_MAX_MM             450
#define CTRLT_SNAP_MS            500
#define CTRLT_SNAPS              24

LOCAL const char *ctrlt_state(UB s)
{
    static const char *const txt[9] = {
        "idle", "starting", "following", "on ramp", "DONE (distance or stop)",
        "OBSTACLE: stopped", "IMPACT: stopped", "FAULT / line lost", "turning"
    };
    return (s < 9u) ? txt[s] : "?";
}

LOCAL const char *ctrlt_evt(UB e)
{
    static const char *const txt[12] = {
        "-", "FAULT", "OBSTACLE", "LINE_END", "RAMP_ON", "RAMP_OFF", "IMPACT",
        "jolt ignored", "STOP_REQ", "BARCODE", "JUNCTION", "COMMAND"
    };
    return (e < 12u) ? txt[e] : "?";
}

LOCAL const char *ctrlt_cmd(UB c)
{
    static const char *const txt[5] = {"none", "LEFT", "RIGHT", "STRAIGHT", "U-TURN"};
    return (c < 5u) ? txt[c] : "?";
}

LOCAL void ctrl_test_course(void)
{
    static robot_status_t snaps[CTRLT_SNAPS];
    static ctrl_log_t lg[CTRL_LOG_LEN];
    static bc_elem_t bel[BC_MAX_ELEMS];
    robot_status_t st;
    INT run, c, n, i, k, ns;
    UB opts;
    ER er;

    er = ctrl_task_create();
    if(E_OK != er) {
        tm_printf((UB *)"[CTRL] ctrl_task_create failed: %d\n", er);
        return;
    }
    ctrl_set_after_turn_mm(0);
    tm_printf((UB *)"\n[CTRL] Vehicle controller: %d mm/s, stops after %d mm. IMU %s.\n",
              CTRLT_MM_S, CTRLT_MAX_MM,
              (0u != (fault_get() & FAULT_IMU)) ? "NOT answering (no ramp handling)" : "ok");

    for(run = 1; ; run++) {
        tm_printf((UB *)"\n[CTRL] Run %d: set the car at the mat edge (centre IR on the line's"
                  " right edge), keep it still, then press\n[CTRL]   l = line + ramp,"
                  "  o = line + obstacle stop,  b = line + barcode command at the cross"
                  "\n[CTRL]   (any key while running stops it)\n", run);
        console_flush_input();
        do {
            c = console_try_getc();
            tk_dly_tsk(50);
        } while((c != 'l') && (c != 'o') && (c != 'b'));
        opts = (c == 'o') ? CTRL_OPT_OBSTACLE : ((c == 'b') ? CTRL_OPT_BARCODE : 0u);
        tk_dly_tsk(500);
        console_flush_input();

        er = ctrl_run(CTRLT_MM_S, CTRLT_MAX_MM, opts);
        if(E_OK != er) {
            tm_printf((UB *)"[CTRL] ctrl_run failed: %d\n", er);
            continue;
        }
        ns = 0;
        while(ctrl_running()) {
            for(i = 0; (i < CTRLT_SNAP_MS / 20) && ctrl_running(); i++) {
                if(console_try_getc() >= 0) {
                    ctrl_stop();
                }
                tk_dly_tsk(20);
            }
            if(ns < CTRLT_SNAPS) {
                ctrl_get_status(&snaps[ns++]);
            }
        }
        ctrl_get_status(&st);

        tm_printf((UB *)"[CTRL] run %d (%s): %s at %d mm, heading %d x0.01 deg,"
                  " faults 0x%x, events dropped %d\n", run,
                  (0u != (opts & CTRL_OPT_OBSTACLE)) ? "line + obstacle"
                  : ((0u != (opts & CTRL_OPT_BARCODE)) ? "line + barcode" : "line + ramp"),
                  ctrlt_state(st.state),
                  st.dist_mm, st.heading_cdeg, (INT)st.faults, (INT)ctrl_evt_dropped());
        n = (INT)ctrl_log_get(lg);
        tm_printf((UB *)"[CTRL]   events (t_ms, d_mm, event, arg, val):\n");
        for(i = 0; i < n; i++) {
            tm_printf((UB *)"[CTRL]     %d, %d, %s, %d, %d", (INT)lg[i].t_ms, lg[i].d_mm,
                      ctrlt_evt(lg[i].type), (INT)lg[i].arg, lg[i].val);
            if(EVT_BARCODE == lg[i].type) {
                if(0u != lg[i].arg) {
                    tm_printf((UB *)"   (read '%c' %s)", (INT)lg[i].arg,
                              (0 != lg[i].val) ? "REVERSED" : "forward");
                } else {
                    tm_printf((UB *)"   (no read, %d bars/spaces)", lg[i].val);
                }
            } else if(EVT_COMMAND == lg[i].type) {
                tm_printf((UB *)"   (%s done, turned %d x0.01 deg)", ctrlt_cmd(lg[i].arg),
                          lg[i].val);
            }
            tm_printf((UB *)"\n");
        }
        if(0u != (opts & CTRL_OPT_BARCODE)) {
            n = (INT)barcode_scan_last(bel, BC_MAX_ELEMS);
            tm_printf((UB *)"[CTRL]   last barcode read, %d bars/spaces (B mm/peak mV,"
                      " w mm):\n[CTRL]    ", n);
            for(i = 0; i < n; i++) {
                k = (INT)((bel[i].w * SWEEP_UM_PER_EDGE) / 100);
                if(0u != bel[i].bar) {
                    tm_printf((UB *)" B%d.%d/%d", k / 10, k % 10, bel[i].peak);
                } else {
                    tm_printf((UB *)" w%d.%d", k / 10, k % 10);
                }
                if((i % 10) == 9) {
                    tm_printf((UB *)"\n[CTRL]    ");
                }
            }
            tm_printf((UB *)"\n");
        }
        tm_printf((UB *)"[CTRL]   robot_status every %d ms (state, d_mm, heading, roll,"
                  " sonar_mm, flags R/I/W, pending command, last event):\n", CTRLT_SNAP_MS);
        for(i = 0; i < ns; i++) {
            tm_printf((UB *)"[CTRL]     %s, %d, %d, %d, %d, %c%c%c, %s, %s\n",
                      ctrlt_state(snaps[i].state), snaps[i].dist_mm, snaps[i].heading_cdeg,
                      snaps[i].roll_cdeg, snaps[i].obstacle_mm,
                      (0u != (snaps[i].flags & CTRL_FLAG_ON_RAMP)) ? 'R' : '-',
                      (0u != (snaps[i].flags & CTRL_FLAG_IMU_OK)) ? 'I' : '-',
                      (0u != (snaps[i].flags & CTRL_FLAG_WATCHING)) ? 'W' : '-',
                      ctrlt_cmd((UB)((snaps[i].flags >> CTRL_FLAG_CMD_SHIFT) & 7u)),
                      ctrlt_evt(snaps[i].last_evt));
        }
        tm_printf((UB *)"[CTRL]   Note where the car ended up (and the nose gap for o).\n");
    }
}
#elif MOTION_TEST_MODE == BARCODE_TEST_DRIVE
/* Buddy 3 barcode step 2: driven scan with the slit-taped left IR. The car
   drives itself straight at BCD_MM_S for BCD_DIST_MM with the barcode
   under the left IR, storing the left AO once per encoder step (0.33 mm):
   constant speed and height, unlike the hand-pushed passes. Decoder v2
   (barcode.c, 24 Oct): threshold just above white, bars and spaces split
   into narrow/wide separately, "*...*" searched both ways. Printed after
   the car stops: levels, every bar (width, peak) and space, the decode,
   and the profile every 1 mm. */
#define BCD_MM_S                 50
#define BCD_DIST_MM              250
#define BCD_LEN                  800    /* steps: 262 mm */
#define BCD_ROW                  20     /* profile values per line (1 mm each) */

LOCAL INT bcd_steps(void)
{
    enc_diag_t l, r;

    (void)motion_get_encoder_diag(&l, &r);
    return (INT)((l.hw_count + r.hw_count) / 2);
}

LOCAL void bcd_print_mm(INT steps)
{
    INT mm10 = (steps * SWEEP_UM_PER_EDGE) / 100;

    tm_printf((UB *)"%d.%d", mm10 / 10, mm10 % 10);
}

LOCAL void barcode_test_drive(void)
{
    static UH prof[BCD_LEN];
    static bc_elem_t el[BC_MAX_ELEMS];
    bc_result_t res;
    INT pass, n, i, k, s, last, ne, ok_n = 0, tries = 0;
    uint32_t une;
    BOOL dec_ok;
    UH white;

    ir_init();
    ir_ao_init();
    tm_printf((UB *)"\n[BCD] Driven barcode scan v2: left IR AO, %d mm/s for %d mm.\n",
              BCD_MM_S, BCD_DIST_MM);
    tm_printf((UB *)"[BCD] Live readout first: check white, line, wide bar, thin bar."
              " Press a key to go on.\n");
    console_flush_input();
    while(console_try_getc() < 0) {
        tm_printf((UB *)"[BCD]   AO=%d mV\n", (INT)ir_ao_left_mv());
        tk_dly_tsk(400);
    }

    for(pass = 1; ; pass++) {
        tm_printf((UB *)"\n[BCD] Pass %d: put the car so the left IR is ~3 cm before the"
                  " barcode, square to it, press a key (any key while driving stops it).\n",
                  pass);
        console_flush_input();
        while(console_try_getc() < 0) {
            tk_dly_tsk(50);
        }
        tk_dly_tsk(1000);
        console_flush_input();
        fault_clear(FAULT_MOTION_MASK);
        motion_reset_odometry();
        motion_set_velocity(BCD_MM_S, BCD_MM_S);

        /* No printing while driving */
        last = -1;
        while(console_try_getc() < 0) {
            s = bcd_steps();
            if((s >= BCD_LEN) || (((s * SWEEP_UM_PER_EDGE) / 1000) >= BCD_DIST_MM)
               || (0u != (fault_get() & FAULT_MOTION_MASK))) {
                break;
            }
            if(s >= 0) {
                UH mv = (UH)ir_ao_left_mv();

                for(k = last + 1; k <= s; k++) {
                    prof[k] = mv;
                }
                if(s > last) {
                    last = s;
                }
            }
            tk_dly_tsk(1);
        }
        motion_stop();
        tk_dly_tsk(500);
        n = last + 1;

        tm_printf((UB *)"\n[BCD] Pass %d: ", pass);
        bcd_print_mm(n);
        tm_printf((UB *)" mm recorded, faults 0x%x\n", (INT)fault_get());
        if(n < 30) {
            continue;
        }

        dec_ok = barcode_read(prof, (uint32_t)n, el, BC_MAX_ELEMS, &une, &white, &res);
        ne = (INT)une;
        tm_printf((UB *)"[BCD]   white %d mV (fixed threshold %d mV); %d elements"
                  " (B width mm/peak mV, w width mm):\n[BCD]  ",
                  white, white + BC_ON_MV, ne);
        for(i = 0; i < ne; i++) {
            if(0u != el[i].bar) {
                tm_printf((UB *)" B");
                bcd_print_mm(el[i].w);
                tm_printf((UB *)"/%d", el[i].peak);
            } else {
                tm_printf((UB *)" w");
                bcd_print_mm(el[i].w);
            }
            if((i % 10) == 9) {
                tm_printf((UB *)"\n[BCD]  ");
            }
        }
        tm_printf((UB *)"\n");
        tries++;
        if(dec_ok) {
            ok_n++;
            tm_printf((UB *)"[BCD]   DECODED \"%s\" %s, from element %d, %d star element(s)"
                      " corrected\n", res.text, (0u != res.reversed) ? "REVERSED" : "forward",
                      (INT)res.first, (INT)res.star_errs);
        } else {
            tm_printf((UB *)"[BCD]   no Code 39 match\n");
        }
        tm_printf((UB *)"[BCD]   decoded %d of %d passes so far\n", ok_n, tries);

        tm_printf((UB *)"[BCD]   profile, mV every 1 mm:\n");
        for(i = 0; i < n; i += 3 * BCD_ROW) {
            tm_printf((UB *)"[BCD] %d mm:", (i * SWEEP_UM_PER_EDGE) / 1000);
            for(k = i; (k < (i + (3 * BCD_ROW))) && (k < n); k += 3) {
                tm_printf((UB *)" %d", prof[k]);
            }
            tm_printf((UB *)"\n");
        }
    }
}
#elif MOTION_TEST_MODE == OBST_TEST_WIDTH
/* Buddy 5: obstacle width (course: a 10-20 cm block, detect it, measure or
   estimate its width and report it). Calibration runs: the car drives at
   WIDTH_MM_S toward a flat-faced object until the obstacle task's stop
   rule triggers, stops, then scans +/-WIDTH_HALF_DEG in the obstacle task.
   Printed afterwards: the stop, the present width estimate (seen span
   minus the narrow-target beam width) and every coarse and fine reading,
   so the estimator can be fitted to objects of known width. */
#define WIDTH_MM_S               200
#define WIDTH_MAX_MM             1200   /* stop anyway after this far */
#define WIDTH_HALF_DEG           45
#define WIDTH_DETECT_MM          600
#define WIDTH_ROW                8

LOCAL INT width_dist_mm(void)
{
    enc_diag_t l, r;

    (void)motion_get_encoder_diag(&l, &r);
    return (INT)((((l.hw_count + r.hw_count) / 2) * SWEEP_UM_PER_EDGE) / 1000);
}

LOCAL void width_print_pts(const char *name, const scan_pt_t *p, INT n)
{
    INT i;

    tm_printf((UB *)"[WIDTH]   %s (%d readings, cdeg,mm):", name, n);
    for(i = 0; i < n; i++) {
        if((i % WIDTH_ROW) == 0) {
            tm_printf((UB *)"\n[WIDTH]    ");
        }
        tm_printf((UB *)" %d,%d", p[i].cdeg, p[i].mm);
    }
    tm_printf((UB *)"\n");
}

LOCAL void obst_test_width(void)
{
    static scan_pt_t pts[SCAN_LOG_MAX];
    obstacle_profile_t prof;
    INT run, i, n, d_trig, mm_trig, rest, nr;
    W sum;
    BOOL stopped;
    ER er;

    er = obstacle_task_create();
    if(E_OK != er) {
        tm_printf((UB *)"[WIDTH] obstacle_task_create failed: %d\n", er);
        return;
    }
    tm_printf((UB *)"\n[WIDTH] Obstacle width calibration: drive at %d mm/s, stop at the"
              " object, scan +/-%d deg.\n", WIDTH_MM_S, WIDTH_HALF_DEG);
    tm_printf((UB *)"[WIDTH] Use flat-faced objects ~10, ~15 and ~20 cm wide, face square"
              " to the car, about 50 cm ahead. Keep ~25 cm clear on each side.\n");

    for(run = 1; ; run++) {
        tm_printf((UB *)"\n[WIDTH] Run %d: aim the car at the object (centred, or offset if"
                  " you like), press a key (any key while driving stops it).\n", run);
        console_flush_input();
        while(console_try_getc() < 0) {
            tk_dly_tsk(50);
        }
        tk_dly_tsk(1000);
        console_flush_input();
        fault_clear(FAULT_MOTION_MASK | FAULT_SCAN_TIMEOUT);

        obstacle_watch(TRUE);
        tk_dly_tsk(300);
        motion_reset_odometry();
        motion_set_velocity(WIDTH_MM_S, WIDTH_MM_S);
        stopped = FALSE;
        d_trig = -1;
        mm_trig = -1;
        while(width_dist_mm() < WIDTH_MAX_MM) {
            if(obstacle_blocked()) {
                motion_stop();
                d_trig = width_dist_mm();
                mm_trig = obstacle_last_mm();
                stopped = TRUE;
                break;
            }
            if((console_try_getc() >= 0)
               || (0u != (fault_get() & (FAULT_MOTION_MASK | FAULT_NO_ECHO)))) {
                break;
            }
            tk_dly_tsk(5);
        }
        motion_stop();
        tk_dly_tsk(800);

        /* Range at rest: mean of up to 5 good pings */
        sum = 0;
        nr = 0;
        for(i = 0; (i < 12) && (nr < 5); i++) {
            tk_dly_tsk(70);
            if(obstacle_last_mm() > 0) {
                sum += obstacle_last_mm();
                nr++;
            }
        }
        rest = (nr > 0) ? (INT)(sum / nr) : -1;
        obstacle_watch(FALSE);

        if(!stopped) {
            tm_printf((UB *)"[WIDTH] run %d: no obstacle stop (faults 0x%x), skipped\n",
                      run, (INT)fault_get());
            continue;
        }

        er = obstacle_scan_start(WIDTH_HALF_DEG, WIDTH_DETECT_MM);
        if(E_OK == er) {
            er = obstacle_scan_wait(OBS_SCAN_TIMEOUT_MS, &prof);
        }

        tm_printf((UB *)"[WIDTH] run %d: stopped (triggered at %d mm travelled, sensor %d mm),"
                  " at rest sensor %d mm; scan %s, faults 0x%x\n", run, d_trig, mm_trig, rest,
                  (E_OK == er) ? "ok" : "FAILED", (INT)fault_get());
        if((E_OK == er) && prof.found) {
            tm_printf((UB *)"[WIDTH]   nearest %d mm at %d cdeg; seen from %d to %d cdeg"
                      " (span %d), beam %d -> width estimate %d mm (old rule %d mm),"
                      " sides %d / %d mm (left of centreline +)\n", prof.nearest_mm,
                      prof.nearest_cdeg, prof.right_cdeg, prof.left_cdeg,
                      prof.left_cdeg - prof.right_cdeg, prof.beam_cdeg, prof.width_mm,
                      prof.width_old_mm, prof.left_side_mm, prof.right_side_mm);
        } else if(E_OK == er) {
            tm_printf((UB *)"[WIDTH]   nothing within %d mm\n", WIDTH_DETECT_MM);
        }
        n = (INT)scan_get_coarse(pts, SCAN_LOG_MAX);
        width_print_pts("coarse sweep right to left", pts, n);
        n = (INT)scan_get_fine(0u, pts, SCAN_FINE_MAX);
        width_print_pts("fine, right edge", pts, n);
        n = (INT)scan_get_fine(1u, pts, SCAN_FINE_MAX);
        width_print_pts("fine, left edge", pts, n);
        tm_printf((UB *)"[WIDTH]   Note the object's real width, which face, and the nose"
                  " gap.\n");
    }
}
#elif MOTION_TEST_MODE == MOTION_TEST_PI_STEP
/* Run a list of speed steps under PI control. Nothing is printed while the
   motors run (usermain outranks the motion task, so a blocking print would
   stall the loop); edge counts over the last PI_MEASURE_MS of each hold are
   saved and printed with the trace once the car has stopped. */
LOCAL void motion_pi_steps(const char *name, const int16_t *steps, INT n)
{
    INT i;
    /* static: usermain's stack is only 1 KB */
    static enc_diag_t l0[PI_MAX_STEPS], r0[PI_MAX_STEPS];
    static enc_diag_t l1[PI_MAX_STEPS], r1[PI_MAX_STEPS];
    INT l_mm_s, r_mm_s, diff_pct;

    if(n > PI_MAX_STEPS) {
        n = PI_MAX_STEPS;
    }

    motion_reset_odometry();
    motion_trace_start();

    for(i = 0; i < n; i++) {
        motion_set_velocity(steps[i], steps[i]);
        tk_dly_tsk(PI_HOLD_MS - PI_MEASURE_MS);
        motion_get_encoder_diag(&l0[i], &r0[i]);
        tk_dly_tsk(PI_MEASURE_MS);
        motion_get_encoder_diag(&l1[i], &r1[i]);
    }

    motion_set_velocity(0, 0);
    tk_dly_tsk(PI_STOP_MS);

    tm_printf((UB *)"\n[PI] %s trace:\n", name);
    motion_trace_dump();

    for(i = 0; i < n; i++) {
        /* edges over PI_MEASURE_MS -> mm/s at 322 um/edge */
        l_mm_s = (INT)(((l1[i].hw_count - l0[i].hw_count) * 1000 / PI_MEASURE_MS)
                       * SWEEP_UM_PER_EDGE / 1000);
        r_mm_s = (INT)(((r1[i].hw_count - r0[i].hw_count) * 1000 / PI_MEASURE_MS)
                       * SWEEP_UM_PER_EDGE / 1000);
        diff_pct = (l_mm_s != 0) ? ((r_mm_s - l_mm_s) * 100 / l_mm_s) : 0;
        tm_printf((UB *)"[PI] %s target=%4d mm/s  L=%4d  R=%4d mm/s  R-L=%d%%\n",
                  name, (INT)steps[i], l_mm_s, r_mm_s, diff_pct);
    }
}

LOCAL void motion_test_pi_step(void)
{
    static const int16_t fwd_steps[] = { 300, 500, 150 };
    static const int16_t rev_steps[] = { -300 };

    tm_printf((UB *)"\n[TEST] PI step response, lifted or on the floor\n");
    tm_printf((UB *)"[TEST] On the floor: ~1.6 m forward then ~0.5 m back\n");
    tm_printf((UB *)"[TEST] Starting in 10 s\n");
    tk_dly_tsk(10000);

    motion_pi_steps("fwd", fwd_steps, 3);
    motion_pi_steps("rev", rev_steps, 1);

    tm_printf((UB *)"\n=== PI step test complete ===\n");
}
#elif MOTION_TEST_MODE == MOTION_TEST_HAND
/* Motors are never commanded, so the only edges are from hand turning. */
LOCAL void motion_test_hand(void)
{
    enc_check_t chk;

    tm_printf((UB *)"\n[TEST] Hand-turn test: motors OFF\n");
    tm_printf((UB *)"[TEST] Mark each wheel. Turn it 10 full turns FORWARD,"
              " note cnt, then 10 turns BACK. Reset to zero the counters.\n");
    tm_printf((UB *)"[TEST] Right wheel: A = GP15 as counted, B = GP17, fixed = taken off"
              " A beyond B + 1 (expected ~6340 for 10 turns).\n");

    motion_reset_odometry();
    while(1) {
        tk_dly_tsk(HAND_PRINT_MS);
        motion_print_encoder_diag("HAND");
        motion_get_enc_check(&chk);
        tm_printf((UB *)"[HAND] right A raw=%d  B=%d  fixed=%d in %d passes\n",
                  (INT)chk.right_a_raw, (INT)chk.right_b, (INT)chk.fixed_edges,
                  (INT)chk.bad_windows);
    }
}
#else
LOCAL void motion_test_distance(void)
{
    static const struct {
        uint16_t cm;
        uint16_t mm_s;
    } moves[] = {
        { 50, 300 },
        { 30, 300 },
        { 50, 500 },
    };
    INT i;
    INT s;
    ER err;

    tm_printf((UB *)"\n[TEST] Distance test on the floor (PI control)\n");
    tm_printf((UB *)"[TEST] Mark the car's front before each move, then"
              " tape-measure how far it went\n");

    for(i = 0; i < (INT)(sizeof(moves) / sizeof(moves[0])); i++) {
        for(s = DIST_PAUSE_S; s > 0; s--) {
            tm_printf((UB *)"[TEST] Move %d (%u cm at %u mm/s) in %d s\n",
                      i + 1, moves[i].cm, moves[i].mm_s, s);
            tk_dly_tsk(1000);
        }

        err = motion_move_forward_cm(moves[i].cm, moves[i].mm_s);
        if(E_OK != err) {
            tm_printf((UB *)"[TEST] Move %d failed: %d\n", i + 1, err);
        }
    }

    tm_printf((UB *)"\n=== Distance test complete ===\n");
}
#endif

/* ------------------------------------------------------------------ *
 * Entry point
 * ------------------------------------------------------------------ */

/* Create an object, or say which one failed and stop. A silent object
   failure here would surface much later as a mysterious hang. */
LOCAL BOOL made(const char *what, ID id)
{
    if(id <= E_OK) {
        tm_printf((UB *)"[init] tk_cre_%s failed, er=%d\n", what, id);
        return FALSE;
    }
    return TRUE;
}

EXPORT INT usermain(void)
{
    ID tid;
    ER err;

#if APP_DEMO_PIPELINE
    mpfid = tk_cre_mpf(&cmpf); if(!made("mpf", mpfid)) return 1;
    mtxid = tk_cre_mtx(&cmtx); if(!made("mtx", mtxid)) return 1;
    mbfid = tk_cre_mbf(&cmbf); if(!made("mbf", mbfid)) return 1;
    semid = tk_cre_sem(&csem); if(!made("sem", semid)) return 1;
    flgid = tk_cre_flg(&cflg); if(!made("flg", flgid)) return 1;

#endif

    tid = tk_cre_tsk(&ctsk_blink); if(made("tsk(blink)", tid)) tk_sta_tsk(tid, 0);
#if APP_DEMO_PIPELINE
    tid = tk_cre_tsk(&ctsk_monitor); if(made("tsk(monitor)", tid)) tk_sta_tsk(tid, 0);
    tid = tk_cre_tsk(&ctsk_consumer); if(made("tsk(consumer)", tid)) tk_sta_tsk(tid, 0);
    tid = tk_cre_tsk(&ctsk_producer); if(made("tsk(producer)", tid)) tk_sta_tsk(tid, 0);
#endif
#if TM_WIFI_CYW43
    tid = tk_cre_tsk(&ctsk_wifi); if(made("tsk(wifi)", tid)) tk_sta_tsk(tid, 0);
#endif

    /* ------------------------------------------------------------------ *
     * ROBOT MOTION SUBSYSTEM TEST
     * ------------------------------------------------------------------ */

    tm_printf((UB *)"\n=== Starting Motion Subsystem Test ===\n");

    err = motion_task_create();
    if (E_OK != err)
    {
        tm_printf((UB *)"[MAIN] Motion task creation failed: %d\n", err);
        return 1;
    }

    // Wait for motion task to initialize
    tk_dly_tsk(2000);

#if MOTION_TEST_MODE == MOTION_TEST_FIXED_TIME
    motion_test_fixed_time();
#elif MOTION_TEST_MODE == MOTION_TEST_DUTY_SWEEP
    motion_test_duty_sweep();
#elif MOTION_TEST_MODE == MOTION_TEST_PI_STEP
    motion_test_pi_step();
#elif MOTION_TEST_MODE == IMU_TEST_BRINGUP
    imu_test_bringup();
#elif MOTION_TEST_MODE == IMU_TEST_ACCEL_6FACE
    imu_test_accel_6face();
#elif MOTION_TEST_MODE == IMU_TEST_MAG_CAL
    imu_test_mag_cal();
#elif MOTION_TEST_MODE == IMU_TEST_HEADING
    imu_test_heading();
#elif MOTION_TEST_MODE == IMU_TEST_HUMP_LOG
    imu_test_hump_log();
#elif MOTION_TEST_MODE == IMU_TEST_TERRAIN
    imu_test_terrain();
#elif MOTION_TEST_MODE == ULTRA_TEST_RANGE
    ultra_test_range();
#elif MOTION_TEST_MODE == SERVO_TEST_JOG
    servo_test_jog();
#elif MOTION_TEST_MODE == SERVO_TEST_MAP
    servo_test_map();
#elif MOTION_TEST_MODE == ULTRA_TEST_BEAM
    ultra_test_beam();
#elif MOTION_TEST_MODE == ULTRA_TEST_SCAN
    ultra_test_scan();
#elif MOTION_TEST_MODE == DEMO_ROAM
    demo_roam();
#elif MOTION_TEST_MODE == IR_TEST_LOG
    ir_test_log();
#elif MOTION_TEST_MODE == IR_TEST_AO
    ir_test_ao();
#elif MOTION_TEST_MODE == MOTION_TEST_SAFETY
    motion_test_safety();
#elif MOTION_TEST_MODE == MOTION_TEST_CPU
    motion_test_cpu();
#elif MOTION_TEST_MODE == IMU_TEST_RAMP_ROLL
    imu_test_ramp_roll();
#elif MOTION_TEST_MODE == MOTION_TEST_STRAIGHT
    motion_test_straight();
#elif MOTION_TEST_MODE == IMU_TEST_SIDE_HUMP
    imu_test_side_hump();
#elif MOTION_TEST_MODE == OBST_TEST_STOP
    obst_test_stop();
#elif MOTION_TEST_MODE == DEMO_STOP
    demo_stop();
#elif MOTION_TEST_MODE == SCAN_TEST_ABORT
    scan_test_abort();
#elif MOTION_TEST_MODE == DEMO_AVOID
    demo_avoid();
#elif MOTION_TEST_MODE == LINE_TEST_FOLLOW
    line_test_follow();
#elif MOTION_TEST_MODE == LINE_TEST_RAMP
    line_test_ramp();
#elif MOTION_TEST_MODE == CTRL_TEST_COURSE
    ctrl_test_course();
#elif MOTION_TEST_MODE == BARCODE_TEST_DRIVE
    barcode_test_drive();
#elif MOTION_TEST_MODE == OBST_TEST_WIDTH
    obst_test_width();
#elif MOTION_TEST_MODE == MOTION_TEST_HAND
    motion_test_hand();
#else
    motion_test_distance();
#endif

    /* The initial task has nothing left to do. It must not return: on
       return the kernel shuts the system down. */
    tk_slp_tsk(TMO_FEVR);
    return 0;
}