
from datetime import datetime
import argparse

import paho.mqtt.client as mqtt

TOPIC = "inf2004/group27/telemetry"


def decode_telemetry(payload):
    if len(payload) != 10:
        raise ValueError(f"Expected 10 bytes, got {len(payload)}")

    # Same little-endian, bit-by-bit order as telemetry_codec.c
    bits = int.from_bytes(payload, "little")

    def read(width):
        nonlocal bits
        value = bits & ((1 << width) - 1)
        bits >>= width
        return value

    version = read(2)
    state = read(3)
    left_raw = read(11)
    right_raw = read(11)
    distance = read(16)
    barcode = read(2)
    barcode_valid = read(1)
    hump = read(8)
    obstacle = read(1)
    obstacle_distance = read(12)
    faults = read(5)
    heartbeat = read(8)

    if version != 1:
        raise ValueError(f"Unsupported version: {version}")
    if left_raw > 1100 or right_raw > 1100:
        raise ValueError("Invalid wheel-speed encoding")

    return {
        "state": state,
        "left": left_raw - 550,
        "right": right_raw - 550,
        "distance": distance,
        "barcode": barcode,
        "barcode_valid": barcode_valid,
        "hump": hump,
        "obstacle": obstacle,
        "obstacle_distance": obstacle_distance,
        "faults": faults,
        "heartbeat": heartbeat,
    }


def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        print("Connected! Waiting for Pico W telemetry...", flush=True)
        client.subscribe(TOPIC)
    else:
        print(f"MQTT connection failed: {reason_code}", flush=True)


def on_message(client, userdata, msg):
    now = datetime.now().strftime("%H:%M:%S.%f")[:-3]

    try:
        d = decode_telemetry(msg.payload)
    except ValueError as error:
        print(f"[{now}] Invalid packet: {error}", flush=True)
        return

    print(
        f"[{now}] "
        f"State={d['state']} | "
        f"L={d['left']} mm/s | "
        f"R={d['right']} mm/s | "
        f"Distance={d['distance']} mm | "
        f"Barcode={d['barcode']} "
        f"(valid={d['barcode_valid']}) | "
        f"Hump={d['hump']} mm | "
        f"Obstacle={d['obstacle']} "
        f"({d['obstacle_distance']} mm) | "
        f"Faults={d['faults']} | "
        f"Heartbeat={d['heartbeat']}",
        flush=True
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=1883)
    args = parser.parse_args()

    client = mqtt.Client(
        callback_api_version=mqtt.CallbackAPIVersion.VERSION2
    )
    client.on_connect = on_connect
    client.on_message = on_message

    print(f"Connecting to {args.host}:{args.port}...")
    client.connect(args.host, args.port, keepalive=30)
    client.loop_forever()


if __name__ == "__main__":
    main()
