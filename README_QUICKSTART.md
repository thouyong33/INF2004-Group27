# INF2004 Group 27
## µT-Kernel RP2040 Demo Build and Flash Quick Start

The micro T-kernel(mtk3smp-rp2040) uses its own **GNU Make** build flow.
It is not built the same way as a normal Pico SDK CMake project like the lab exercises.

## 1. Prerequisites

Install the required tools:

```bash
sudo apt update
sudo apt install build-essential make g++ git gcc-arm-none-eabi
```

### 1.1 Verify toolchain is installed

```bash
arm-none-eabi-gcc --version
make --version
g++ --version
```
## 2. Build the demo

```bash
cd mtk3smp-rp2040/build_make

# check nproc number (e.g 16)
nproc

# Single-core build
make -j16
# OR
make -j$(nproc)
# Output:
mtk3pico_smp0_uart.uf2

# Clean up between builds
make clean

# Dual-core SMP build
make SMP=1 -j16
# OR
make SMP=1 -j$(nproc)
# Output
mtk3pico_smp1_uart.uf2
```

## 3. Flash the UF2 to Pico W

## 4. Serial console setup

The default demo uses UART

**UART pins**
- GP0 = UART TX
- GP1 = UART RX
- GND = common ground

**Using Raspberry Pi Debug Probe**
Connect the UART port of the debug probe:
- Debug Probe RX -> Pico GP0
- Debug Probe TX -> Pico GP1
- Debug Probe GND -> Pico GND

**Serial settings**
- 115200 baud
- 8 data bits
- no parity
- 1 stop bit

## 5. Expected output

Single-core build:

```bash
microT-Kernel Version 3.00

=== uT-Kernel 3.0 / RP2040 demo ===
single-core build
producer -> [mbf] -> consumer, 2 credits, 500 ms period
```

SMP build:

```bash
microT-Kernel Version 3.00

=== uT-Kernel 3.0 / RP2040 demo ===
SMP build: 2 processors
producer -> [mbf] -> consumer, 2 credits, 500 ms period
```

## 6. Notes

- The default UART serial monitor uses GP0 and GP1
- The default microT-kernel blink task uses GP16
- Any hardware connections should be checked for conflict with these 3 pins