Buddy 1 — Wi-Fi, MQTT, Telemetry and Command Handover

Project: INF2004 Group 27 — Autonomous Robotic Car
Hardware: Raspberry Pi Pico W
RTOS: µT-Kernel 3.0 SMP
Status: Standalone communication tested; final Vehicle Controller and live sensor integration pending.

1. Overview

Buddy 1 implements the Wi-Fi and MQTT communication subsystem for the robotic car.

The subsystem currently supports:

Connecting the Pico W to a configured Wi-Fi network.

Connecting to an MQTT broker.

Publishing compact binary telemetry approximately every 500 ms.

Including an 8-bit heartbeat counter.

Receiving and validating binary START and STOP commands.

Attempting automatic reconnection after communication interruptions.

Decoding binary telemetry into human-readable output using a Python application.

Important: The current telemetry values are simulated. MQTT commands are received and logged but are not yet connected to the Vehicle Controller.

2. Important source files

File

Purpose

lib/libnet/lwip/lwip_utk_mqtt.c

MQTT connection, subscriptions, publishing, reconnection and command reception

lib/libnet/lwip/lwip_utk_mqtt.h

MQTT public interfaces

lib/libnet/lwip/telemetry_codec.c

Binary telemetry encoding and decoding

lib/libnet/lwip/telemetry_codec.h

Telemetry data structure and protocol definitions

lib/libnet/lwip/include/mqtt_config.h

Local MQTT broker settings

lib/libnet/lwip/include/mqtt_config.example.h

Shareable example configuration

config/wifi_credentials.h

Local Wi-Fi credentials

lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c

Wi-Fi polling and MQTT integration

build_make/pico_rp2040.mk

Firmware build configuration

buddy1_mqtt_decoder.py

PC-side readable telemetry decoder

lib/libnet/lwip/telemetry_codec_test.c

Standalone encoder/decoder tests

Configuration files containing personal credentials should not be committed.

3. MQTT architecture

Pico W Robotic Car
    |
    | Wi-Fi
    v
MQTT Broker (Mosquitto)
    |
    +---- MQTT telemetry subscriber
    |
    +---- Python readable telemetry decoder
    |
    +---- Other authorised MQTT clients

The Pico W uses one configured MQTT broker at a time.

Multiple MQTT clients can subscribe to the same broker simultaneously.

The laptop and Pico W must be able to reach each other over the chosen network.

4. Configure Wi-Fi and MQTT

Each developer using an independent Wi-Fi network or broker must configure the relevant local files.

Wi-Fi

Copy the supplied example credentials file into the expected local configuration path if the file does not already exist.

Update the Wi-Fi SSID and password for the intended network.

MQTT

Check lib/libnet/lwip/include/mqtt_config.h.

Configure:

MQTT broker hostname or IP address.

MQTT broker port, normally 1883 for the local test setup.

MQTT client ID.

MQTT topic names.

The tested topic names are:

Topic

Direction

inf2004/group27/telemetry

Pico W publishes; PC subscribes

inf2004/group27/command

PC publishes; Pico W subscribes

Use a unique MQTT client ID for each connected client.

Mosquitto

Install Mosquitto on the broker computer.

Ensure its broker service is running, has a listener reachable from the Pico W, and is permitted by the local firewall.

A localhost-only listener will not be reachable from the Pico W.

Local unauthenticated development configuration should only be used on a trusted isolated network, not exposed publicly.

5. Build the Pico W firmware

Environment: WSL Ubuntu.

Navigate to the project build directory:

cd build_make

Run:

make -f Makefile \
  SMP=1 WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 \
  WIFI_DHCP=1 WIFI_DNS=1 WIFI_MQTT=1 \
  CONSOLE=usb_cdc PICO_SDK_PATH=$HOME/pico/pico-sdk \
  all -j8

The tested build uses the ARM cross-compiler and Pico SDK installed in WSL.

The firmware output is:

mtk3pico_smp1_usb_cdc_wifi_mqtt.uf2

Flashing

Disconnect the Pico W.

Hold BOOTSEL while reconnecting USB.

Copy the UF2 firmware onto the RPI-RP2 drive.

Wait for the board to restart.

Open COM3 or the assigned USB serial port at 115200 baud.

Confirm that Wi-Fi and MQTT connect successfully.

6. Telemetry V2 — Binary packet specification

The current provisional protocol is 10 bytes (80 bits).

Fields are packed sequentially, least-significant bit first, without gaps.

Field

Bit width

Encoding / permitted values

Version

2

Current version = 1

Robot state

3

0–7

Left wheel speed

11

Actual speed in mm/s + 550; supported −550 to +550

Right wheel speed

11

Actual speed in mm/s + 550; supported −550 to +550

Travelled distance

16

0–65,535 mm

Barcode command

2

0–3

Barcode valid

1

0 or 1

Maximum hump height

8

0–255 mm

Obstacle detected

1

0 or 1

Obstacle distance

12

0–4,095 mm

Fault flags

5

Bitmask, 0–31

Heartbeat counter

8

0–255, wraps around

All value ranges are provisional until confirmed by the subsystem owners and integration lead.

This is a compact binary representation of the selected fields, but the final minimum-bit compliance of all stored and shared robot data still needs to be reviewed for Change Requirement 1.

Current simulated inputs

The MQTT publishing function currently constructs a telemetry_data_t using:

Field

Test input

Version

1

Robot state

2

Left speed

300 mm/s

Right speed

295 mm/s

Distance

1,500 mm

Barcode command

1, valid

Maximum hump

24 mm

Obstacle detected

Yes

Obstacle distance

200 mm

Fault flags

0

Heartbeat

Incrementing 8-bit counter

An example transmitted payload is:

49 6A 4D E3 2E 28 46 64 00 C8

Here, C8 is hexadecimal for heartbeat 200.

Required integration

Replace the hardcoded test values in lwip_utk_mqtt_poll() with a consistent snapshot of live robot_status data.

The integration must establish:

Actual state identifiers.

Per-wheel speed units and signed ranges.

Travelled distance range and overflow policy.

Barcode command identifiers and validity.

Maximum hump measurement range.

Obstacle profile representation, including whether additional fields are necessary.

Fault-bit definitions.

Mutex protection and snapshot ownership.

Do not read shared fields inconsistently while other RTOS tasks are updating them.

Where fields or ranges change, update both the encoder and Python decoder, and rerun protocol tests.

7. Read raw MQTT telemetry

Environment: Windows PowerShell.

Navigate to Mosquitto:

cd "C:\Program Files\mosquitto"

Subscribe:

.\mosquitto_sub.exe -h localhost -p 1883 -t "inf2004/group27/telemetry" -F "%I | %x"

Expected output:

2026-10-31T01:45:35+0800 | 496a4de32e28466400c8

Telemetry is published automatically by the Pico W. No command needs to be sent first.

The hexadecimal output should contain 20 hexadecimal characters per message.

8. Read decoded telemetry

Install Python 3 and the required MQTT package on the monitoring computer:

py -m pip install paho-mqtt

From the directory containing buddy1_mqtt_decoder.py:

py buddy1_mqtt_decoder.py

If the broker is on another computer, specify its reachable IP address:

py buddy1_mqtt_decoder.py --host BROKER_IP_ADDRESS

Example output:

State=2 | L=300 mm/s | R=295 mm/s |
Distance=1500 mm | Barcode=1 (valid=1) |
Hump=24 mm | Obstacle=1 (200 mm) |
Faults=0 | Heartbeat=200

The decoder's timestamp represents PC reception time, not the exact Pico W transmission timestamp.

9. Binary MQTT command protocol

The current supported command payload is exactly one byte.

Hexadecimal byte

Command

Current implementation

00

START

Received, validated and logged as S

01

STOP

Received, validated and logged as P

02

Reserved

Rejected

Other values

Invalid

Rejected

Multiple bytes

Invalid payload length

Rejected

Send START

Windows PowerShell:

[System.IO.File]::WriteAllBytes("$env:TEMP\mqtt_start.bin", [byte[]]@(0))

.\mosquitto_pub.exe -h localhost -p 1883 -t "inf2004/group27/command" -f "$env:TEMP\mqtt_start.bin"

Send STOP

[System.IO.File]::WriteAllBytes("$env:TEMP\mqtt_stop.bin", [byte[]]@(1))

.\mosquitto_pub.exe -h localhost -p 1883 -t "inf2004/group27/command" -f "$env:TEMP\mqtt_stop.bin"

Expected Pico W serial output:

[mqtt] Received command: S
[mqtt] Received command: P

Required integration

The received commands currently pass through lwip_utk_mqtt_get_command() and are printed by the Wi-Fi task.

They are not yet forwarded into the Vehicle Controller's ctrl_evt_mbf.

The integration lead must connect START and STOP to the controller's agreed event interface and verify command acceptance by robot state.

Remote MQTT STOP must not be treated as a guaranteed emergency-stop mechanism because network communication can fail.

10. Heartbeat, reconnection and diagnostics

Telemetry is scheduled approximately every 500 ms while MQTT is connected.

The heartbeat increments for each publish accepted by the MQTT API and wraps after 255.

A heartbeat increment does not guarantee broker delivery because telemetry currently uses MQTT QoS 0.

The firmware includes:

Wi-Fi link status reporting.

MQTT connection state reporting.

Five-second retry logic for failed MQTT connections.

DNS resolution and retry logic.

A disconnected-client check.

Rate-limited publish-error diagnostics.

A hotspot OFF/ON test demonstrated that telemetry can resume without a manual Pico W reset.

The team's five-second post-link recovery acceptance target has not been formally timed or verified.

Debug logs

Useful temporary diagnostics include:

[wifi] link UP
[mqtt] Connected successfully!
[mqtt] Connection failed
[mqtt-debug] Publish failed: err=-1
[wifi-debug] poll_count=...

Keep debugging functionality available during integration.

Before release and real-time timing measurement, disable excessive logging and measure whether Wi-Fi/MQTT activities affect control-task timing.

11. Tests already performed

Test

Observed result

Codec normal-value round trip

Passed

Codec maximum-value round trip

Passed

Negative wheel-speed boundary

Passed

Invalid positive speed rejection

Passed

Pico W 10-byte telemetry publishing

Passed

Readable Python decoder

Passed

Heartbeat sequence

Passed in tested captures

Approximately 500 ms receipt interval

Observed

One-byte binary START reception

Passed

One-byte binary STOP reception

Passed

Unsupported command 02

No accepted command observed

Two-byte invalid command

No accepted command observed

Post-invalid valid STOP

Received successfully

Functional hotspot disconnection/recovery

Observed

Five-second recovery acceptance criterion

Not yet verified

Integrated vehicle-controller commands

Not yet implemented

Real live telemetry

Not yet implemented

Wi-Fi outage impact on motion-task deadlines

Not yet verified

12. Remaining work and ownership

Buddy 1 and integration lead

Agree on the final telemetry schema and field ranges.

Connect live robot-status data to the telemetry encoder.

Connect validated MQTT commands to Vehicle Controller events.

Handle unexpected commands and invalid states safely.

Confirm reconnection behaviour under repeated controlled outages.

Measure post-link telemetry recovery time.

Test the complete robot while Wi-Fi is disconnected or reconnecting.

Other subsystem owners

Confirm the units and ranges of the values they supply.

Provide documented shared-data interfaces.

Verify control and sensor timing when telemetry is enabled.

Agree on the fault bitmask and obstacle profile data required for reporting.

Final testing

Verify correctness at agreed boundary values.

Record transmission and recovery timing.

Check for MQTT packet loss and heartbeat gaps.

Measure real-time task interference and any RAM impact.

Compare the final packed representation with the Change Requirement 1 baseline.

13. Known limitations

Telemetry values are still hardcoded simulations.

Binary START/STOP commands do not yet control the robot.

The 10-byte packet format may change after integration.

The current packet does not include the full obstacle-profile detail.

Automatic reconnection was observed, but its worst-case recovery time is not established.

Complete robot timing and safety behaviour under Wi-Fi outage are unverified.

The current single-broker configuration is intended for local development and requires network-specific settings.

14. Integration into thouyong (31 Oct 2026)

Merged into the thouyong branch; this client replaces the earlier backup Buddy 1 client there (app_program/buddy1_telemetry/mqtt_tel.c, removed).

Telemetry V2 is now version 2, 88 bits (11 bytes): robot state 4 bits (9 controller states), barcode command 3 bits (5 commands), fault flags 8 bits (8 fault bits), plus 3 reserved bits. telemetry_codec_test.c and buddy1_mqtt_decoder.py are updated and the codec tests pass.

Live data: lwip_utk_mqtt_poll() fills the packet through buddy1_fill_telemetry() (app_program/buddy1_telemetry/tel_source.c): controller state, wheel speeds from the encoders, run distance (wraps at 65.5 m), pending barcode command, highest ramp, sonar reading, fault bits. The hooks are declared in lib/libnet/lwip/buddy1_hooks.h.

Commands reach the car: 00 START starts the continuous course, 01 STOP stops it, 10-15 start the l, o, b, a, r runs and p prints the continuous run's log. They are fed to the course test as keystrokes.

Added to the client: a last will ("0", retained, on inf2004/group27/online; "1" on connect; keep-alive 10 s), controller events on inf2004/group27/event (14 bytes each), timing figures on inf2004/group27/sys once a second, FAULT_WIFI_LOSS after 3 s without the broker (logged; the car drives on), and a Wi-Fi rejoin every 5 s in cyw43_utk.c. The periodic [wifi-debug] print is off unless BUDDY1_DEBUG=1.

Laptop: tools/mqtt_viewer.py is a broker and viewer in one (no Mosquitto needed) with a control page on http://localhost:8080; buddy1_mqtt_decoder.py also works against it.
