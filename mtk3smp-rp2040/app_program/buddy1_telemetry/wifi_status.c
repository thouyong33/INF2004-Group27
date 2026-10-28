/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * wifi_status.c
 * The port's Wi-Fi status task, moved out of app_main.c (28 Oct 2026) as
 * the starting point for Buddy 1 (MQTT telemetry). Built only with
 * WIFI=cyw43; the robot builds use WIFI=none, so this compiles to nothing.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "common/usb_console_compat.h"
#include "wifi_status.h"

#if TM_WIFI_CYW43
/* Plain C types on purpose - see the note in cyw43_utk.h. */
#include "cyw43_utk.h"

#define STACK_SZ 4096

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

ER
wifi_status_create(void)
{
    ID tid;

    tid = tk_cre_tsk(&ctsk_wifi);
    if (tid < E_OK)
    {
        return (ER)tid;
    }
    return tk_sta_tsk(tid, 0);
}
#endif /* TM_WIFI_CYW43 */
