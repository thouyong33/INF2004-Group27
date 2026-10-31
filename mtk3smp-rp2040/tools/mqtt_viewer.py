#!/usr/bin/env python3
"""
Buddy 1 laptop end: a small MQTT 3.1.1 broker that also prints the car's
telemetry. Plain Python 3.8+, nothing to install.

The car (WIFI_MQTT=1 build, Buddy 1 client lib/libnet/lwip/
lwip_utk_mqtt.c) connects to this laptop on port 1883 and publishes:

    inf2004/group27/telemetry  11-byte bit-packed Telemetry V2, 2 Hz
                               (lib/libnet/lwip/telemetry_codec.c)
    inf2004/group27/event      14-byte controller event, one per event
    inf2004/group27/sys        14-byte timing figures, once a second
    inf2004/group27/online     "1" on connect, "0" as its last will

and listens on inf2004/group27/command for one command byte: 00 START
(the continuous course), 01 STOP, 10-15 = runs l o b a r and p (print
the continuous run's log). Send them from the
control page this script opens on http://localhost:8080 (live status,
buttons, events and run timing; tools/mqtt_web.html), or type a key here
and press Enter.

The event and timing layouts are in lib/libnet/lwip/buddy1_hooks.h. Other
MQTT clients (buddy1_mqtt_decoder.py, MQTT Explorer, mosquitto_sub) can
connect too and subscribe to inf2004/group27/#: this is a real broker,
with QoS 0/1 publishes, retained messages, wildcards, keep-alive and
last will. No QoS 2, no auth.

Usage:
    python tools/mqtt_viewer.py                 # status once a second
    python tools/mqtt_viewer.py --every 0       # every status (5 Hz)
    python tools/mqtt_viewer.py --csv run1.csv  # also log to a file

Put this laptop's hotspot address in lib/libnet/lwip/include/
mqtt_config.h (MQTT_BROKER_HOST) before building.
Windows treats a phone hotspot as a Public network, so allow port 1883 once
(admin PowerShell):
    New-NetFirewallRule -DisplayName "MQTT 1883" -Direction Inbound
        -Protocol TCP -LocalPort 1883 -Action Allow -Profile Any
"""

import argparse
import asyncio
import collections
import csv
import json
import os
import socket
import struct
import sys
import threading
import time
import webbrowser

STATES = ["IDLE", "START", "FOLLOW", "RAMP", "DONE", "OBSTACLE", "IMPACT",
          "FAULT", "TURN"]
EVENTS = ["NONE", "FAULT", "OBSTACLE", "LINE_END", "RAMP_ON", "RAMP_OFF",
          "IMPACT", "JOLT_IGNORED", "STOP_REQ", "BARCODE", "JUNCTION",
          "COMMAND", "OBST_WIDTH", "OBST_CREEP", "OBST_EDGE", "OBST_AVOID",
          "OBST_DONE", "OBST_HDG"]
FAULTS = ["STALL", "ENC_L", "ENC_R", "IMU", "NO_ECHO", "WIFI", "SCAN",
          "LINE_LOST"]
COMMANDS = ["none", "left", "right", "straight", "U-turn"]

TOPIC = "inf2004/group27/"
TELEMETRY_LEN = 11
EVENT_FMT = "<IiBBhH"             # 14 bytes
SYS_FMT = "<HIHHHH"               # 14 bytes
RUN_STATES = (1, 2, 3, 5, 8)      # START FOLLOW RAMP OBSTACLE TURN
KEYS = "lobacrsp"
# BARCODE event val >> 1 (barcode.h BC_CUT_*)
CUT_TEXT = {1: ", last bar missing", 2: ", first bar missing"}
KEY_BYTE = {"c": 0x00, "s": 0x01, "l": 0x10, "o": 0x11, "b": 0x12,
            "a": 0x13, "r": 0x14, "p": 0x15}


def decode_telemetry(payload):
    """Telemetry V2 (version 2, 88 bits, LSB first), as in
    telemetry_codec.c and buddy1_mqtt_decoder.py."""
    bits = int.from_bytes(payload, "little")
    out = []
    for width in (2, 4, 11, 11, 16, 3, 1, 8, 1, 12, 8, 8):
        out.append(bits & ((1 << width) - 1))
        bits >>= width
    (version, state, left, right, dist, cmd, valid, hump, obst, obst_mm,
     faults, heartbeat) = out
    if version != 2 or left > 1100 or right > 1100:
        raise ValueError("not a version 2 packet")
    return {"state": state, "left": left - 550, "right": right - 550,
            "dist_mm": dist, "cmd": cmd, "valid": valid, "hump": hump,
            "obstacle": obst, "obst_mm": obst_mm, "faults": faults,
            "heartbeat": heartbeat}


def name(table, i):
    return table[i] if 0 <= i < len(table) else "?%d" % i


def fault_text(bits):
    names = [FAULTS[b] for b in range(8) if bits & (1 << b)]
    return "+".join(names) if names else "none"


def stamp():
    return time.strftime("%H:%M:%S")


class Hub:
    """Fans what the viewer prints, and the live status, out to the web
    page's open feeds. A page opened later gets the recent lines first."""

    def __init__(self):
        self.queues = set()
        self.backlog = collections.deque(maxlen=400)
        self.latest = {}              # last status/sys/car message by type

    def send(self, msg):
        if msg["t"] == "line":
            self.backlog.append(msg)
        else:
            self.latest[msg["t"]] = msg
        for q in list(self.queues):
            if q.qsize() < 1000:
                q.put_nowait(msg)


HUB = Hub()


def say(text, kind="info"):
    """Print a line on the terminal and show it on the web page."""
    line = "%s  %s" % (stamp(), text)
    print(line)
    HUB.send({"t": "line", "kind": kind, "text": line})


def local_addresses():
    found = []
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))
        found.append(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None,
                                       socket.AF_INET):
            if info[4][0] not in found and not info[4][0].startswith("127."):
                found.append(info[4][0])
    except OSError:
        pass
    return found or ["(unknown)"]


class Viewer:
    """Decodes and prints the inf2004/group27/* topics."""

    def __init__(self, every, csv_path):
        self.every = every
        self.last_print = 0.0
        self.last_state = None
        self.status_count = 0
        self.first_status = None
        self.run = None               # timing seen during the current run
        self.writer = None
        if csv_path:
            self.file = open(csv_path, "w", newline="")
            self.writer = csv.writer(self.file)
            self.writer.writerow(["time", "topic", "a", "b", "c", "d", "e",
                                  "f", "g", "h", "i", "j", "k"])

    def log(self, topic, values):
        if self.writer:
            self.writer.writerow([round(time.time(), 3), topic] + values)
            self.file.flush()

    def message(self, topic, payload):
        if topic == TOPIC + "online":
            text = payload.decode(errors="replace")
            say("car %s" % ("ONLINE" if text == "1"
                            else "OFFLINE (last will)"), "online")
            HUB.send({"t": "car", "up": text == "1"})
            self.log(topic, [text])
        elif topic == TOPIC + "telemetry" and len(payload) == TELEMETRY_LEN:
            try:
                self.status(decode_telemetry(payload))
            except ValueError as error:
                say("bad telemetry %s: %s" % (payload.hex(), error), "warn")
        elif topic == TOPIC + "event" and len(payload) == 14:
            self.event(struct.unpack(EVENT_FMT, payload))
        elif topic == TOPIC + "sys" and len(payload) == 14:
            self.sys(struct.unpack(SYS_FMT, payload))
        elif topic == TOPIC + "command":
            pass                                  # our own keys
        elif topic.startswith(TOPIC):
            say("%s: %d bytes %s" % (topic, len(payload), payload.hex()))

    def status(self, t):
        state = t["state"]
        faults = t["faults"]
        obst = t["obst_mm"] if t["obstacle"] else -1
        now = time.time()
        self.status_count += 1
        if self.first_status is None:
            self.first_status = now
        self.log("telemetry", [t[k] for k in ("state", "left", "right",
                 "dist_mm", "cmd", "hump", "obst_mm", "faults",
                 "heartbeat")])
        if state in RUN_STATES and self.run is None:
            self.run = {"rate": None, "pass": 0, "busy": 0, "gap": 0,
                        "samples": None}
        elif state not in RUN_STATES and self.run is not None:
            self.run_summary()
        rate_now = 0.0
        if now > self.first_status + 1:
            rate_now = self.status_count / (now - self.first_status)
        HUB.send({"t": "status", "state": name(STATES, state),
                  "running": state in RUN_STATES,
                  "total_m": t["dist_mm"] / 1000.0,
                  "left": t["left"], "right": t["right"],
                  "sonar": obst, "ramp": t["hump"],
                  "cmd": name(COMMANDS, t["cmd"]),
                  "heartbeat": t["heartbeat"],
                  "faults": fault_text(faults), "fault_bits": faults,
                  "rate": round(rate_now, 1)})
        changed = state != self.last_state
        if not changed and self.every > 0 and now - self.last_print \
                < self.every:
            return
        self.last_state = state
        self.last_print = now
        rate = ""
        if now > self.first_status + 1:
            rate = "  %.1f/s" % (self.status_count / (now - self.first_status))
        cmd = t["cmd"]
        say("%-8s L=%4d R=%4d mm/s dist=%6.2f m sonar=%5s ramp=%2d%s "
            "faults=%s hb=%3d%s"
            % (name(STATES, state), t["left"], t["right"],
               t["dist_mm"] / 1000.0, "-" if obst < 0 else obst,
               t["hump"],
               " cmd=" + name(COMMANDS, cmd) if cmd else "",
               fault_text(faults), t["heartbeat"], rate), "status")

    def sys(self, v):
        rate, busy, pass_us, samples, gap, drops = v
        self.log("sys", list(v))
        HUB.send({"t": "sys", "rate": rate, "busy_pct": busy / 10000.0,
                  "pass": pass_us, "samples": samples, "gap": gap,
                  "drops": drops})
        r = self.run
        if r is None:
            return
        r["rate"] = rate if r["rate"] is None else min(r["rate"], rate)
        r["pass"] = max(r["pass"], pass_us)
        r["busy"] = max(r["busy"], busy)
        r["gap"] = max(r["gap"], gap)
        if samples:                       # 0 in the second the run starts
            r["samples"] = samples if r["samples"] is None \
                else min(r["samples"], samples)
        r["drops"] = drops

    def run_summary(self):
        r = self.run
        self.run = None
        if r["rate"] is None:
            return
        say("run timing (Wi-Fi on): motion loop at least %d/s "
            "(500 = on time, 2 ms loop), longest pass %d us, busy up to %.1f%%; "
            "controller at least %s samples/s (50 = every 20 ms), longest "
            "gap %d ms; status drops so far %d"
            % (r["rate"], r["pass"], r["busy"] / 10000.0,
               r["samples"] if r["samples"] is not None else "-",
               r["gap"], r.get("drops", 0)), "timing")

    def event(self, v):
        t_ms, d_mm, etype, arg, val, idx = v
        self.log("event", list(v))
        extra = ""
        if etype == 9:                                   # BARCODE
            extra = "  no read"
            if arg:
                extra = "  read '%s' %s%s" % (
                    chr(arg), "REVERSED" if val & 1 else "forward",
                    CUT_TEXT.get(val >> 1, ""))
        elif etype == 11:                                # COMMAND
            extra = "  %s done" % name(COMMANDS, arg)
        elif etype == 1:                                 # FAULT
            extra = "  " + fault_text(arg)
        say("event #%d at %.1f s, %d mm: %s arg=%d val=%d%s"
            % (idx, t_ms / 1000.0, d_mm, name(EVENTS, etype), arg, val,
               extra), "fault" if etype == 1 else "event")


class Broker:
    def __init__(self, viewer):
        self.viewer = viewer
        self.sessions = set()
        self.retained = {}

    @staticmethod
    def matches(filt, topic):
        f = filt.split("/")
        t = topic.split("/")
        for i, part in enumerate(f):
            if part == "#":
                return True
            if i >= len(t) or (part != "+" and part != t[i]):
                return False
        return len(f) == len(t)

    def car_connected(self):
        return any(s.is_car() for s in self.sessions)

    def key(self, ch):
        """Send one menu key to the car; returns whether it went."""
        if not self.car_connected():
            say("car not connected: '%s' not sent" % ch, "warn")
            return False
        say("-> sent '%s' to the car" % ch, "key")
        self.route(TOPIC + "command", bytes([KEY_BYTE[ch]]), False)
        return True

    def route(self, topic, payload, retain, sender=None):
        if retain:
            if payload:
                self.retained[topic] = payload
            else:
                self.retained.pop(topic, None)
        self.viewer.message(topic, payload)
        for s in list(self.sessions):
            if s is not sender and any(self.matches(f, topic)
                                       for f in s.filters):
                s.send_publish(topic, payload)

    async def handle(self, reader, writer):
        s = Session(self, reader, writer)
        await s.run()


class Session:
    def __init__(self, broker, reader, writer):
        self.broker = broker
        self.reader = reader
        self.writer = writer
        self.filters = set()
        self.client_id = "?"
        self.will = None
        self.keepalive = 0
        self.peer = writer.get_extra_info("peername")

    def is_car(self):
        """The car's MQTT_CLIENT_ID (mqtt_config.h) ends in "car"."""
        return self.client_id.endswith("car")

    def send(self, ptype, body):
        n = len(body)
        length = bytearray()
        while True:
            b = n % 128
            n //= 128
            length.append(b | (0x80 if n else 0))
            if not n:
                break
        self.writer.write(bytes([ptype]) + bytes(length) + body)

    def send_publish(self, topic, payload):
        t = topic.encode()
        self.send(0x30, struct.pack(">H", len(t)) + t + payload)

    async def read_packet(self):
        timeout = self.keepalive * 1.5 if self.keepalive else None
        head = await asyncio.wait_for(self.reader.readexactly(1), timeout)
        mult, n = 1, 0
        while True:
            b = (await self.reader.readexactly(1))[0]
            n += (b & 0x7F) * mult
            mult *= 128
            if not b & 0x80:
                break
        body = await self.reader.readexactly(n) if n else b""
        return head[0], body

    @staticmethod
    def string(body, i):
        n = struct.unpack_from(">H", body, i)[0]
        return body[i + 2:i + 2 + n], i + 2 + n

    def connect(self, body):
        _proto, i = self.string(body, 0)
        level, flags, self.keepalive = struct.unpack_from(">BBH", body, i)
        i += 4
        cid, i = self.string(body, i)
        self.client_id = cid.decode(errors="replace") or "?"
        if flags & 0x04:
            wt, i = self.string(body, i)
            wm, i = self.string(body, i)
            self.will = (wt.decode(), wm, bool(flags & 0x20))
        if self.is_car():
            HUB.send({"t": "car", "up": True})
        say("[broker] %s connected from %s (MQTT level %d, keep-alive %d s)"
            % (self.client_id, self.peer[0], level, self.keepalive),
            "broker")
        self.send(0x20, b"\x00\x00")

    def publish(self, head, body):
        qos = (head >> 1) & 3
        topic, i = self.string(body, 0)
        if qos:
            pid = struct.unpack_from(">H", body, i)[0]
            i += 2
            if qos == 1:
                self.send(0x40, struct.pack(">H", pid))
        self.broker.route(topic.decode(errors="replace"), body[i:],
                          bool(head & 1), self)

    def subscribe(self, body):
        pid = struct.unpack_from(">H", body, 0)[0]
        i, granted, new = 2, bytearray(), []
        while i < len(body):
            f, i = self.string(body, i)
            i += 1                                        # requested QoS
            f = f.decode(errors="replace")
            self.filters.add(f)
            new.append(f)
            granted.append(0)
        self.send(0x90, struct.pack(">H", pid) + bytes(granted))
        for topic, payload in self.broker.retained.items():
            if any(Broker.matches(f, topic) for f in new):
                self.send(0x31, struct.pack(">H", len(topic.encode()))
                          + topic.encode() + payload)

    async def run(self):
        clean = False
        try:
            head, body = await self.read_packet()
            if head >> 4 != 1:
                return
            self.connect(body)
            self.broker.sessions.add(self)
            while True:
                head, body = await self.read_packet()
                kind = head >> 4
                if kind == 3:
                    self.publish(head, body)
                elif kind == 8:
                    self.subscribe(body)
                elif kind == 10:                          # UNSUBSCRIBE
                    pid = struct.unpack_from(">H", body, 0)[0]
                    self.send(0xB0, struct.pack(">H", pid))
                elif kind == 12:                          # PINGREQ
                    self.send(0xD0, b"")
                elif kind == 14:                          # DISCONNECT
                    clean = True
                    break
                await self.writer.drain()
        except asyncio.TimeoutError:
            say("[broker] %s silent past its keep-alive" % self.client_id,
                "broker")
        except (asyncio.IncompleteReadError, ConnectionError, OSError,
                struct.error):
            pass
        finally:
            self.broker.sessions.discard(self)
            if self.is_car() and not self.broker.car_connected():
                HUB.send({"t": "car", "up": False})
            say("[broker] %s %s" % (self.client_id, "disconnected" if clean
                                    else "connection lost"), "broker")
            if self.will and not clean:
                topic, msg, retain = self.will
                self.broker.route(topic, msg, retain)
            self.writer.close()


WEB_PAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "mqtt_web.html")


class Web:
    """The control page on http://localhost:<web port>: GET / is the page,
    GET /events a live feed (server-sent events), POST /key sends a key.
    Bound to 127.0.0.1 only. A key needs the X-Car header, which a page
    from another site cannot add without a CORS preflight this server never
    answers, so only our own page can drive the car."""

    def __init__(self, broker):
        self.broker = broker

    @staticmethod
    def reply(writer, status, ctype, body):
        writer.write(("HTTP/1.1 %s\r\nContent-Type: %s\r\nContent-Length: "
                      "%d\r\nCache-Control: no-store\r\nConnection: close"
                      "\r\n\r\n" % (status, ctype, len(body))).encode()
                     + body)

    @staticmethod
    def event(writer, msg):
        writer.write(b"data: " + json.dumps(msg).encode() + b"\n\n")

    async def feed(self, writer):
        writer.write(b"HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\n"
                     b"Cache-Control: no-store\r\nConnection: keep-alive"
                     b"\r\n\r\n")
        q = asyncio.Queue()
        HUB.queues.add(q)
        try:
            for m in list(HUB.backlog) + list(HUB.latest.values()):
                self.event(writer, m)
            self.event(writer, {"t": "car",
                                "up": self.broker.car_connected()})
            await writer.drain()
            while True:
                try:
                    m = await asyncio.wait_for(q.get(), 15)
                except asyncio.TimeoutError:
                    writer.write(b": ping\n\n")         # keeps it open
                else:
                    self.event(writer, m)
                await writer.drain()
        finally:
            HUB.queues.discard(q)

    async def handle(self, reader, writer):
        try:
            req = await asyncio.wait_for(reader.readline(), 10)
            parts = req.decode(errors="replace").split()
            if len(parts) < 2:
                return
            method, path = parts[0], parts[1].split("?")[0]
            headers = {}
            while True:
                h = await asyncio.wait_for(reader.readline(), 10)
                if h in (b"\r\n", b"\n", b""):
                    break
                k, _, v = h.decode(errors="replace").partition(":")
                headers[k.strip().lower()] = v.strip()
            length = int(headers.get("content-length", "0") or 0)
            body = await reader.readexactly(length) if length else b""
            if method == "GET" and path == "/":
                with open(WEB_PAGE, "rb") as f:
                    self.reply(writer, "200 OK", "text/html; charset=utf-8",
                               f.read())
            elif method == "GET" and path == "/events":
                await self.feed(writer)
            elif method == "POST" and path == "/key" \
                    and headers.get("x-car") == "1":
                ch = body.decode(errors="replace").strip()[:1].lower()
                ok = bool(ch) and ch in KEYS and self.broker.key(ch)
                self.reply(writer, "200 OK", "application/json",
                           json.dumps({"ok": ok}).encode())
            else:
                self.reply(writer, "404 Not Found", "text/plain",
                           b"not found")
            await writer.drain()
        except (asyncio.TimeoutError, asyncio.IncompleteReadError,
                ConnectionError, OSError, ValueError):
            pass
        finally:
            writer.close()


async def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--web-port", type=int, default=8080,
                    help="control page on http://localhost:<this>")
    ap.add_argument("--no-browser", action="store_true",
                    help="do not open the control page on start")
    ap.add_argument("--every", type=float, default=1.0,
                    help="seconds between status lines (0 = all); a state "
                         "change always prints")
    ap.add_argument("--csv", help="also write every message to this file")
    args = ap.parse_args()

    broker = Broker(Viewer(args.every, args.csv))
    server = await asyncio.start_server(broker.handle, "0.0.0.0", args.port)
    web = await asyncio.start_server(Web(broker).handle, "127.0.0.1",
                                     args.web_port)
    page = "http://localhost:%d" % args.web_port
    print("Buddy 1 MQTT broker + viewer")
    print("  this laptop: %s" % ", ".join(local_addresses()))
    print("  put the hotspot one in config/mqtt_config.h "
          "(UTK_MQTT_BROKER_ADDRESS)")
    print("  listening on 0.0.0.0:%d, Ctrl+C to stop" % args.port)
    print("  control page: %s" % page)
    print("  keys for the car: l o b a c r = start a run, s = stop, "
          "p = print the c-run log (type it, then Enter)\n")
    if not args.no_browser:
        webbrowser.open(page)

    loop = asyncio.get_running_loop()

    def keyboard():
        while True:
            try:
                line = input()
            except (EOFError, OSError):
                return
            ch = line.strip()[:1].lower()
            if ch in KEYS and ch:
                loop.call_soon_threadsafe(broker.key, ch)
            elif ch:
                print("  keys: %s" % " ".join(KEYS))

    threading.Thread(target=keyboard, daemon=True).start()
    async with server, web:
        await asyncio.gather(server.serve_forever(), web.serve_forever())


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nstopped")
        sys.exit(0)
