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
#include "buddy2_motion/motion_task.h"
#include "buddy4_imu/imu.h"

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
 * To switch, change the MOTION_TEST_MODE default below and rebuild.
 * ------------------------------------------------------------------ */
#define MOTION_TEST_FIXED_TIME   1
#define MOTION_TEST_DISTANCE     2
#define MOTION_TEST_HAND         3
#define MOTION_TEST_DUTY_SWEEP   4
#define MOTION_TEST_PI_STEP      5
#define IMU_TEST_BRINGUP         6
#define IMU_TEST_ACCEL_6FACE     7

#ifndef MOTION_TEST_MODE
#define MOTION_TEST_MODE         IMU_TEST_ACCEL_6FACE
#endif

#define IMU_SAMPLE_COUNT         20
#define IMU_SAMPLE_MS            250

#define FACE_COUNT               6
#define FACE_MOVE_S              10    /* time to reposition the car */
#define FACE_SAMPLES             100   /* averaged per face */
#define FACE_SAMPLE_MS           20    /* 2 s per face */
#define FACE_DOMINANT_MG         700   /* one axis must read > 0.7 g */

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
    tm_printf((UB *)"\n[TEST] Hand-turn test: motors OFF\n");
    tm_printf((UB *)"[TEST] Mark each wheel. Turn it 10 full turns FORWARD,"
              " note cnt, then 10 turns BACK. Reset to zero the counters.\n");

    motion_reset_odometry();
    while(1) {
        tk_dly_tsk(HAND_PRINT_MS);
        motion_print_encoder_diag("HAND");
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