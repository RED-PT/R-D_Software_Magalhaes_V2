# Porting Guide {#porting_guide}

[TOC]

How to add a new MCU target, swap a sensor, or change a peripheral
mapping without breaking the other board. Read this *after*
@ref architecture_tour.

## The abstraction contract

Application code in `Codigo/` may **never** reference a specific HAL
handle (`hspi3`, `huart1`, etc.) or a specific GPIO. It must go
through `config.h`. That is the entire portability mechanism.

If you find yourself writing `&hspi3` in a file under `Codigo/`,
stop and add a macro instead.

### Current macro contract

| Macro | Used by | Buzz V4 (H743) | Nucleo (F446) |
|---|---|---|---|
| `SPI_IMU` | ASM330LHHX | `&hspi1` | `&hspi1` |
| `SPI_BARO` | MS5607 | `&hspi1` | `&hspi1` |
| `SPI_MAG` | MMC5983MA | `&hspi6` (BDMA) | `&hspi3` |
| `SPI_LORA` | SX1262 driver | `&hspi5` | *not defined* |
| `UART_RADIO` | E22 UART driver | *not defined* | `&huart3` |
| `UART_GPS` | u-blox | `&huart1` | `&huart1` |
| `UART_DEBUG` | printf | `&huart2` | `&huart6` |
| `I2C_BNO` | BNO055 | `&hi2c1` | `&hi2c1` |
| `I2C_INA` | INA219 | `&hi2c1` | *not defined* |
| `PWM_ESC_TIM` | ESC PWM | `&htim4` | `&htim3` |
| `RADIO_INTERFACE_SPI` | radio_thread | defined | undefined |
| `RADIO_INTERFACE_UART` | radio_thread | undefined | defined |
| `DMA_BUFFER` | DMA targets | `__attribute__((section(".dma_buffer")))` | (empty) |
| `BDMA_BUFFER` | SPI6 DMA targets | `__attribute__((section(".bdma_buffer")))` | (empty) |

If a board doesn't have a peripheral at all, **leave the macro
undefined** and gate the consumer with `#ifdef`. That is how the radio
driver split is implemented — see `e22_uart_dma.c` and
`lora_sx126x.c`.

## Adding a new MCU target

Skeleton:

1. **Create the CubeIDE project.** Generate the `.ioc`, configure
   peripherals, let CubeMX produce `Core/Inc` and `Core/Src`.
2. **Add `Codigo/` as a linked source folder.** Right-click the project
   → New → Folder → Advanced → Link to alternate location.
3. **Write `Core/Inc/config.h`.** Mirror the existing two; provide
   every macro from the table above (or `#ifdef` consumers if absent).
4. **If the MCU has a cache** (Cortex-M7), define `DMA_BUFFER` and
   `BDMA_BUFFER` to place buffers in non-cacheable SRAM, and add
   `.dma_buffer` / `.bdma_buffer` sections to the linker script. Copy
   from `STM32H743ZIT6_Magalhaes/STM32H743ZITX_FLASH.ld`.
5. **Add the new `config.h` to the Doxyfile** under `INPUT` and
   `INCLUDE_PATH`.

That's it. No driver changes should be required if the abstraction is
clean.

## Swapping a sensor

If the sensor uses an existing bus type, only the driver changes:

1. Drop the new driver under `Codigo/Sensors/<NAME>/`.
2. Expose `_Init()`, `_Read()`, and any `_Calibrate()` it needs.
3. Add it to `sensors_thread.c`: a global instance, an init call, and
   a read path (DMA-driven if the bus supports it).
4. Update @ref TelemetryPackets if the on-the-wire format changes;
   otherwise just feed the existing struct.

If the sensor is a *new kind* (e.g., adding a second IMU for
redundancy), also extend @ref defs.h shared structs and the
DataHandler routing.

## Changing peripheral mapping on an existing board

Edit only that board's `config.h`. Everything else stays untouched.
Verify the move with a clean build.

## Documentation auto-regeneration

A GitHub Actions workflow at `.github/workflows/docs.yml` rebuilds this
documentation on every push to `main` and publishes the HTML to the
`gh-pages` branch (GitHub Pages).

To enable it:

1. Push the workflow file (already in repo).
2. In GitHub repo settings → **Pages**, set source to **GitHub Actions**.

The workflow uses Doxygen 1.9 and Graphviz from the Ubuntu image.
Local rebuilds (`doxygen Doxyfile`) still work and overwrite
`docs/html/`; that folder is in `.gitignore`, so local builds don't
pollute the repo.

## Where the doc structure itself is defined

- **`Doxyfile`** — Doxygen config. `INPUT` lists what gets parsed;
  `EXCLUDE_PATTERNS` filters HAL/Middlewares noise.
- **`mainpage.md`** — landing page.
- **`docs_pages/`** — narrative subpages (you're reading one now) and
  `groups.dox` (the top-level Modules hierarchy).

If you want to add a new top-level Module to the sidebar, add a
`@defgroup` entry to `docs_pages/groups.dox` and reference children
with `@ref`.
