#!/usr/bin/env python3
"""
Buddy 1 laptop end: a small MQTT 3.1.1 broker that also prints the car's
telemetry. Plain Python 3.8+, nothing to install.

The car (WIFI_MQTT=1 build) connects to this laptop on port 1883 and
publishes:

    car27/online   "1" on connect, "0" (its last will) if it drops off
    car27/status   18-byte status snapshot, 5 Hz
    car27/event    14-byte controller event, one per event
    car27/sys      14-byte timing figures, once a second

and listens on car27/cmd: type a key here and press Enter to send it, the
same keys as the serial menu (l o b a c r to start a run, s to stop, p to
print a continuous run's log).

The byte layouts are in app_program/buddy1_telemetry/tel_source.h. Other
MQTT clients (MQTT Explorer, a phone app, mosquitto_sub) can connect too
and subscribe to car27/#: this is a real broker, with QoS 0/1 publishes,
retained messages, wildcards, keep-alive and last will. No QoS 2, no auth.

Usage:
    python tools/mqtt_viewer.py                 # status once a second
    python tools/mqtt_viewer.py --every 0       # every status (5 Hz)
    python tools/mqtt_viewer.py --csv run1.csv  # also log to a file

Put this laptop's hotspot address in config/mqtt_config.h before building.
Windows treats a phone hotspot as a Public network, so allow port 1883 once
(admin PowerShell):
    New-NetFirewallRule -DisplayName "MQTT 1883" -Direction Inbound
        -Protocol TCP -LocalPort 1883 -Action Allow -Profile Any
"""

import argparse
import asyncio
import csv
import socket
import struct
import sys
import threading
import time

STATES = ["IDLE", "START", "FOLLOW", "RAMP", "DONE", "OBSTACLE", "IMPACT",
          "FAULT", "TURN"]
EVENTS = ["NONE", "FAULT", "OBSTACLE", "LINE_END", "RAMP_ON", "RAMP_OFF",
          "IMPACT", "JOLT_IGNORED", "STOP_REQ", "BARCODE", "JUNCTION",
          "COMMAND", "OBST_WIDTH", "OBST_CREEP", "OBST_EDGE", "OBST_AVOID",
          "OBST_DONE"]
FAULTS = ["STALL", "ENC_L", "ENC_R", "IMU", "NO_ECHO", "WIFI", "SCAN",
          "LINE_LOST"]
COMMANDS = ["none", "left", "right", "straight", "U-turn"]

STATUS_FMT = "<BBBBhhhhhBBH"      # 18 bytes
EVENT_FMT = "<IiBBhH"             # 14 bytes
SYS_FMT = "<HIHHHH"               # 14 bytes
RUN_STATES = (1, 2, 3, 5, 8)      # START FOLLOW RAMP OBSTACLE TURN
KEYS = "lobacrsp"


def name(table, i):
    return table[i] if 0 <= i < len(table) else "?%d" % i


def fault_text(bits):
    names = [FAULTS[b] for b in range(8) if bits & (1 << b)]
    return "+".join(names) if names else "none"


def stamp():
    return time.strftime("%H:%M:%S")


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
    """Decodes and prints the car27/* topics."""

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
        if topic == "car27/online":
            text = payload.decode(errors="replace")
            print("%s  car %s" % (stamp(), "ONLINE" if text == "1"
                                  else "OFFLINE (last will)"))
            self.log(topic, [text])
        elif topic == "car27/status" and len(payload) == 18:
            self.status(struct.unpack(STATUS_FMT, payload))
        elif topic == "car27/event" and len(payload) == 14:
            self.event(struct.unpack(EVENT_FMT, payload))
        elif topic == "car27/sys" and len(payload) == 14:
            self.sys(struct.unpack(SYS_FMT, payload))
        elif topic == "car27/cmd":
            pass                                  # our own keys
        elif topic.startswith("car27/"):
            print("%s  %s: %d bytes %s" % (stamp(), topic, len(payload),
                                           payload.hex()))

    def status(self, v):
        (state, faults, last_evt, flags, dist, hdg, roll, obst, width,
         ramp, _spare, total_cm) = v
        now = time.time()
        self.status_count += 1
        if self.first_status is None:
            self.first_status = now
        self.log("status", list(v))
        if state in RUN_STATES and self.run is None:
            self.run = {"rate": None, "pass": 0, "busy": 0, "gap": 0,
                        "samples": None}
        elif state not in RUN_STATES and self.run is not None:
            self.run_summary()
        changed = state != self.last_state
        if not changed and self.every > 0 and now - self.last_print \
                < self.every:
            return
        self.last_state = state
        self.last_print = now
        rate = ""
        if now > self.first_status + 1:
            rate = "  %.1f/s" % (self.status_count / (now - self.first_status))
        cmd = (flags >> 4) & 7
        print("%s  %-8s d=%5d mm total=%5.1f m hdg=%7.2f roll=%6.2f "
              "sonar=%5s width=%4s ramp=%2d%s%s faults=%s%s"
              % (stamp(), name(STATES, state), dist, total_cm / 100.0,
                 hdg / 100.0, roll / 100.0,
                 "-" if obst < 0 else obst, width if width else "-", ramp,
                 " RAMP" if flags & 1 else "",
                 " cmd=" + name(COMMANDS, cmd) if cmd else "",
                 fault_text(faults), rate))

    def sys(self, v):
        rate, busy, pass_us, samples, gap, drops = v
        self.log("sys", list(v))
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
        print("%s  run timing (Wi-Fi on): motion loop at least %d/s "
              "(1000 = every 1 ms), longest pass %d us, busy up to %.1f%%; "
              "controller at least %s samples/s (50 = every 20 ms), longest "
              "gap %d ms; status drops so far %d"
              % (stamp(), r["rate"], r["pass"], r["busy"] / 10000.0,
                 r["samples"] if r["samples"] is not None else "-",
                 r["gap"], r.get("drops", 0)))

    def event(self, v):
        t_ms, d_mm, etype, arg, val, idx = v
        self.log("event", list(v))
        extra = ""
        if etype == 9:                                   # BARCODE
            extra = ("  read '%s' %s" % (chr(arg), "REVERSED" if val
                                         else "forward")) if arg else \
                "  no read"
        elif etype == 11:                                # COMMAND
            extra = "  %s done" % name(COMMANDS, arg)
        elif etype == 1:                                 # FAULT
            extra = "  " + fault_text(arg)
        print("%s  event #%d at %.1f s, %d mm: %s arg=%d val=%d%s"
              % (stamp(), idx, t_ms / 1000.0, d_mm, name(EVENTS, etype),
                 arg, val, extra))


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
        return any(s.client_id == "car27" for s in self.sessions)

    def key(self, ch):
        if not self.car_connected():
            print("%s  car not connected: '%s' not sent" % (stamp(), ch))
            return
        print("%s  -> sent '%s' to the car" % (stamp(), ch))
        self.route("car27/cmd", ch.encode(), False)

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
        print("%s  [broker] %s connected from %s (MQTT level %d, "
              "keep-alive %d s)" % (stamp(), self.client_id, self.peer[0],
                                    level, self.keepalive))
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
            print("%s  [broker] %s silent past its keep-alive"
                  % (stamp(), self.client_id))
        except (asyncio.IncompleteReadError, ConnectionError, OSError,
                struct.error):
            pass
        finally:
            self.broker.sessions.discard(self)
            print("%s  [broker] %s %s" % (stamp(), self.client_id,
                                          "disconnected" if clean
                                          else "connection lost"))
            if self.will and not clean:
                topic, msg, retain = self.will
                self.broker.route(topic, msg, retain)
            self.writer.close()


async def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--every", type=float, default=1.0,
                    help="seconds between status lines (0 = all); a state "
                         "change always prints")
    ap.add_argument("--csv", help="also write every message to this file")
    args = ap.parse_args()

    broker = Broker(Viewer(args.every, args.csv))
    server = await asyncio.start_server(broker.handle, "0.0.0.0", args.port)
    print("Buddy 1 MQTT broker + viewer")
    print("  this laptop: %s" % ", ".join(local_addresses()))
    print("  put the hotspot one in config/mqtt_config.h "
          "(UTK_MQTT_BROKER_ADDRESS)")
    print("  listening on 0.0.0.0:%d, Ctrl+C to stop" % args.port)
    print("  keys for the car: l o b a c r = start a run, s = stop, "
          "p = print the c-run log (type it, then Enter)\n")

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
    async with server:
        await server.serve_forever()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nstopped")
        sys.exit(0)
