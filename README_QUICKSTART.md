# INF2004 Group 27: robot car quick start

The car's firmware runs on µT-Kernel 3.0 on a Raspberry Pi Pico W
(`mtk3smp-rp2040`). It uses the kernel's own **GNU Make** build, not the
Pico SDK's CMake flow from the lab exercises. What each part of the car's
code does is in [`mtk3smp-rp2040/app_program/README.md`](mtk3smp-rp2040/app_program/README.md).

## 1. Prerequisites

On Linux or WSL:

```bash
sudo apt update
sudo apt install build-essential make g++ git gcc-arm-none-eabi python3
```

Check the toolchain:

```bash
arm-none-eabi-gcc --version
make --version
```

The Wi-Fi build (section 3) also needs a Pico SDK checkout for the CYW43
driver and lwIP (we use SDK 2.3.0), with its submodules:

```bash
git clone -b 2.3.0 https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk && git submodule update --init
```

## 2. Build the car (no Wi-Fi)

```bash
cd mtk3smp-rp2040/build_make
make clean
make -j$(nproc)
```

Output: `mtk3pico_smp0_uart.uf2`. It runs the course test (test mode 31,
`CTRL_TEST_COURSE`). To build another test mode, change the default
`MOTION_TEST_MODE` in `app_program/test_modes/test_modes.h`, where the
modes are listed, then `make clean` and build again.

## 3. Build the car with Wi-Fi telemetry (Buddy 1)

1. Copy the two templates and fill them in. Both copies are git-ignored,
   so the hotspot password never reaches the repository:
   - `config/wifi_credentials.example.h` -> `config/wifi_credentials.h`:
     the hotspot's SSID and password, and set `UTK_WIFI_CREDENTIALS_SET`
     to 1.
   - `lib/libnet/lwip/include/mqtt_config.example.h` ->
     `lib/libnet/lwip/include/mqtt_config.h`: set `MQTT_BROKER_HOST` to
     the laptop's address on the hotspot.
2. Build:

```bash
cd mtk3smp-rp2040/build_make
make clean
make -j$(nproc) PICO_SDK_PATH=/path/to/pico-sdk \
     WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1 WIFI_MQTT=1
```

Output: `mtk3pico_smp0_uart_wifi_mqtt.uf2`. This UF2 contains the hotspot
password: keep it on your own machine and do not commit or share it.

## 4. Flash

Hold BOOTSEL while plugging the Pico W in, then copy the `.uf2` onto the
`RPI-RP2` drive that appears. The Pico restarts into the new firmware.

## 5. Serial console

UART on GP0 (TX) and GP1 (RX), 115200 baud, 8 data bits, no parity,
1 stop bit. With a Raspberry Pi Debug Probe: probe RX -> GP0, probe
TX -> GP1, GND -> GND.

After start-up the course test prints its menu and waits for a key:

| Key | Run |
|---|---|
| `l` | follow the line, report the ramp |
| `o` | follow the line, stop at an obstacle and measure it |
| `a` | as `o`, then go round it on the left and rejoin the line |
| `b` | follow the line and act on a barcode at the next cross |
| `r` | follow the line and read barcodes only |
| `c` | the whole course, no distance limit; `s` stops it, `p` prints its log |

Each run ends with its event log, the robot status every 500 ms and the
line trace.

## 6. Laptop viewer and control page (Wi-Fi build)

With the laptop on the same hotspot:

```bash
python mtk3smp-rp2040/tools/mqtt_viewer.py
```

It is the MQTT broker the car connects to, prints the car's telemetry and
events, and opens a control page on http://localhost:8080 with the same run
keys as the serial menu. Windows treats a phone hotspot as a Public network,
so allow port 1883 once (admin PowerShell):

```powershell
New-NetFirewallRule -DisplayName "MQTT 1883" -Direction Inbound -Protocol TCP -LocalPort 1883 -Action Allow -Profile Any
```

`mtk3smp-rp2040/buddy1_mqtt_decoder.py` decodes the telemetry packet on its
own; the packet format is in `BUDDY1_HANDOVER.md`.

## 7. Notes

- The status LED (the blink task) is on GP16 and the console on GP0/GP1;
  the car's own pins are in `app_program/common/pin_defs.h`.
- Test builds kept for the CR1 evidence are in `tests/`
  (`docs/tribunal1/cr1_evidence.md` says which run used which).
