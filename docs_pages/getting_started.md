# Getting Started {#getting_started}

[TOC]

This page takes you from a fresh clone to a flashed board talking to the
ground station. About ten minutes if your toolchain is already set up.

## Prerequisites

- **STM32CubeIDE** 1.13+ (uses GCC ARM, ST-Link drivers, CubeMX).
- **Python 3.10+** for the dashboard (`Dashboard Python/`).
- **Doxygen 1.9+** if you want to rebuild this documentation locally.
- **Git Bash** on Windows. The `cmd.exe` shell does not have `ls`,
  `find`, or `grep`.

## Repository layout

```
R-D_Software_Magalhaes_V2/
├── Codigo/                          ← shared source, both boards
│   ├── Sensors/                     ← drivers + sensors thread
│   ├── Radio/                       ← LoRa drivers + radio thread
│   ├── Telemetry/                   ← packet builders, TDMA constants
│   ├── Flight Computer/             ← FSM, command dispatch
│   ├── Controller/                  ← TVC + motor control
│   ├── Estimator/                   ← altitude / velocity estimation
│   ├── Storage/                     ← SD card thread, FatFS glue
│   ├── Data Handler/                ← circular buffers, queue routing
│   ├── Threads/                     ← FreeRTOS thread creation
│   ├── Atuadores/                   ← actuators (ESC PWM)
│   ├── Tests/                       ← bench-only modes
│   ├── hal_callbacks.{c,h}          ← every HAL_*_Callback lives here
│   └── defs.{c,h}                   ← shared structs and queues
│
├── STM32H743ZIT6_Magalhaes/         ← Buzz V4 CubeIDE project
├── STM32F446ZE_Magalhaes/           ← Nucleo F446 CubeIDE project
├── STM32F413ZH_Magalhaes/           ← (legacy / experimental)
│
├── Buzz_V4 - STM32H7/               ← KiCad PCB design files (not built)
├── Dashboard Python/                ← web UI ground station
├── arduino_gs_software/             ← ground station radio firmware
└── docs/                            ← generated Doxygen output
```

The two CubeIDE projects each include the *same* `Codigo/` tree as
sibling source folders. They differ only by `Core/Inc/config.h`.

## Building the firmware

1. Open STM32CubeIDE.
2. **File → Import → Existing Projects into Workspace**, point to either
   `STM32H743ZIT6_Magalhaes/` (recommended) or `STM32F446ZE_Magalhaes/`.
3. Build (Ctrl+B). The `Codigo/` folder is linked in by reference; no
   duplication.
4. Flash via ST-Link (F11).

If you get an `RADIO_M0_PORT undeclared` error on H743, you're including
the wrong radio driver — the H743 build path uses
`RADIO_INTERFACE_SPI`, not UART. The drivers self-gate via `#ifdef`,
so this should never happen unless `config.h` has been edited.

## Talking to the board

After boot, the FC sits in `IDLE`, waiting for a configuration packet
from the ground station. To talk to it:

1. Plug the **E22-900M22T** Waveshare hat (or matching radio module)
   into the host running the dashboard.
2. `cd "Dashboard Python"` and start the dashboard.
3. The dashboard's `Connect` button opens the serial port to the radio
   bridge. The radio bridge runs the `arduino_gs_software/` firmware.

You should see telemetry within a second of connecting — the FC
broadcasts a fast-telemetry packet every 125 ms once synchronised.

## Building the documentation

```bash
doxygen Doxyfile
```

Output lands in `docs/html/index.html`. **You don't need to build it
manually most of the time** — see @ref porting_guide for the GitHub
Actions workflow that publishes it on every push.

## What to read next

- @ref architecture_tour — how the threads fit together.
- @ref flight_lifecycle — what the FSM is doing while you watch
  telemetry land in the dashboard.
