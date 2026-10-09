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
#include "buddy4_imu/terrain.h"
#include "buddy5_ultrasonic/ultrasonic.h"

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

#ifndef MOTION_TEST_MODE
#define MOTION_TEST_MODE         ULTRA_TEST_RANGE
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

#define US_POINTS                5
#define US_MOVE_S                10    /* time to place the target */
#define US_SAMPLES               30
#define US_PERIOD_MS             70    /* datasheet: >= 60 ms between pings */

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
            base_n++;
        }
        tk_dly_tsk(20);
    }
    terrain_reset((int16_t)((base_n > 0) ? (p0 / base_n) : 0));

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
    static const INT true_mm[US_POINTS] = { 100, 200, 300, 500, 1000 };
    static W mean_mm[US_POINTS];
    static BOOL have[US_POINTS];
    UW t0, t1, echo_us;
    INT p, s, i, ok, miss;
    W sum, sum_sq, mn, mx, mm, var;
    float sx, sy, sxx, sxy, n_f, a, b;
    INT n_fit;

    ultrasonic_init();

    /* Check the timer really runs at 1 MHz against the kernel clock */
    t0 = us_now();
    tk_dly_tsk(500);
    t1 = us_now();
    tm_printf((UB *)"\n[US] timer check: %u us over a 500 ms kernel delay"
              " (expect ~500000-502000)\n", (unsigned int)(t1 - t0));

    tm_printf((UB *)"[US] Range calibration. Use a FLAT, hard target (box side, book,\n");
    tm_printf((UB *)"[US] wall) square to the sensor, measured from the sensor's front face.\n");

    for(p = 0; p < US_POINTS; p++) {
        have[p] = FALSE;
        tm_printf((UB *)"\n[US] Point %d/%d: put the target at %d cm\n", p + 1, US_POINTS,
                  true_mm[p] / 10);
        for(s = US_MOVE_S; s > 0; s--) {
            if(E_OK == ultrasonic_read_us(&echo_us)) {
                tm_printf((UB *)"[US]   measuring in %d s   (now reads %d mm)\n", s,
                          (INT)ultrasonic_us_to_mm(echo_us));
            } else {
                tm_printf((UB *)"[US]   measuring in %d s   (now: no echo)\n", s);
            }
            tk_dly_tsk(1000);
        }

        sum = 0;
        sum_sq = 0;
        mn = 0x7FFFFFFF;
        mx = 0;
        ok = 0;
        miss = 0;
        for(i = 0; i < US_SAMPLES; i++) {
            if(E_OK == ultrasonic_read_us(&echo_us)) {
                mm = ultrasonic_us_to_mm(echo_us);
                sum += mm;
                sum_sq += mm * mm;
                if(mm < mn) mn = mm;
                if(mm > mx) mx = mm;
                ok++;
            } else {
                miss++;
            }
            tk_dly_tsk(US_PERIOD_MS);
        }

        if(0 == ok) {
            tm_printf((UB *)"[US]   no echoes at all (%d misses) - point skipped\n", miss);
            continue;
        }
        mean_mm[p] = sum / ok;
        var = (sum_sq / ok) - (mean_mm[p] * mean_mm[p]);
        have[p] = TRUE;
        tm_printf((UB *)"[US]   true %4d mm: mean %4d  min %4d  max %4d  sd %d mm"
                  "  misses %d/%d\n",
                  true_mm[p], (INT)mean_mm[p], (INT)mn, (INT)mx,
                  (INT)us_isqrt((var > 0) ? (UW)var : 0), miss, US_SAMPLES);
    }

    /* Least-squares fit true = a x measured + b */
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
                tm_printf((UB *)"[US]   %4d mm: corrected %4d mm (error %d mm)\n",
                          true_mm[p], (INT)((a * (float)mean_mm[p]) + b),
                          (INT)((a * (float)mean_mm[p]) + b) - true_mm[p]);
            }
        }
    } else {
        tm_printf((UB *)"\n[US] not enough points for a fit\n");
    }

    tm_printf((UB *)"\n=== Ultrasonic range test complete ===\n");
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