# Architecture Tour {#architecture_tour}

[TOC]

This page walks the system top-down. Read it once and the rest of the
codebase becomes findable: every file you open will fit into one of
the boxes below.

## The big picture

The flight computer is **eight FreeRTOS threads** sharing data through
queues, not globals. Each thread has one job. If a thread blocks, only
its own work pauses — the rest keep running.

```
                  +-------------------+
   ISR / DMA  --> |  Sensors thread   |  (highest, event-driven)
                  +---------+---------+
                            | producer
                            v
                  +---------+---------+
                  |  DataHandler      |  routes one sample to many
                  +-+---------+----+--+
                    |         |    |
                    v         v    v
              +-----+--+  +---+--+ +--------+
              | SDCard |  | Est. | |  ...   |
              +--------+  +------+ +--------+
                                |
                                v
                          +-----+------+
                          | Controller |  100 Hz
                          +-----+------+
                                |
                                v
                            ESC PWM out
```

In parallel:

```
   Ground station --radio--> +-----------+
                             |   Radio   |  TDMA, slot-driven
                             +-----+-----+
                                   |
                                   v
                             +-----+-----+
                             |    FSM    |  20 Hz
                             +-----+-----+
                                   | events
                                   v
                             +-----+------+
                             | Telemetry  |  20 Hz, builds packets
                             +-----+------+
                                   |
                                   +--> back to Radio
```

## Thread cheat sheet

| Priority | Thread | Rate | Lives in | Role |
|---:|---|---|---|---|
| Highest | Sensors | event | `Codigo/Sensors/sensors_thread.c` | Kicks off DMA reads, parses results. |
| High2 | DataHandler | event | `Codigo/Data Handler/data_handler_thread.c` | Fan-out: pushes one sample into multiple queues. |
| High1 | Radio | TDMA | `Codigo/Radio/radio_thread.c` | TX in our slots, RX in slot 9. |
| High | Estimator | 100 Hz | `Codigo/Estimator/estimator_thread.c` | Fuses IMU + barometer into altitude/velocity. |
| High | Controller | 100 Hz | `Codigo/Controller/controller_thread.c` | TVC servo + ESC commands. |
| AbvNorm1 | FSM | 20 Hz | `Codigo/Flight Computer/flight_computer_thread.c` | Big state machine; owns transitions. |
| AbvNorm | Telemetry | 20 Hz | `Codigo/Telemetry/telemetry_thread.c` | Builds telemetry packets. |
| Normal | SDCard | event | `Codigo/Storage/sd_card_thread.c` | FatFS writes. |

## Why these boundaries?

A few decisions worth understanding *before* you change anything:

### Why DMA-driven, not polled?

The IMU runs at 6.6 kHz. Polling it from a thread means a context switch
per sample — pure overhead. DMA-into-buffer + one notification per
batch is roughly an order of magnitude cheaper. See
`Codigo/hal_callbacks.c` for where the DMA-complete IRQs land.

### Why a separate DataHandler?

Two reasons:

1. **One sample, many consumers.** SD card and estimator both want every
   IMU sample, but they consume at different speeds. The DataHandler
   pushes into both queues; the slowest consumer doesn't slow the rest.
2. **The sensors thread should never block on a queue full.** If the SD
   card is busy, the estimator must still get the data. Centralising
   the routing keeps the sensors thread predictable.

### Why TDMA on the radio?

A 1000 ms superframe split into ten 100 ms slots. Eight slots for fast
telemetry (8 Hz IMU+altitude+state), one for slow GPS, one for ground
station RX. **No retries, no ACKs, no collisions.** Predictable
airtime is more useful than reliability for a vehicle that's moving;
old data is dead data anyway.

See @ref TDMAConfig for the constants and @ref RadioTDMA for the state
machine.

### Why SPI on H743 but UART on F446ZE?

The Buzz V4 PCB carries an **E22-900M22S** module: bare SX1262 silicon,
direct SPI access, full LoRa config from firmware. The Nucleo dev board
uses an **E22-xxxT30D**: the same SX1262 plus EBYTE's transparent-mode
firmware, fronted by a UART. Both implement the same TDMA superframe
and packet formats, so the upper layers don't change. The selector is
`RADIO_INTERFACE_SPI` vs `RADIO_INTERFACE_UART` in `config.h`.

### Why no global state?

There are a couple — the FSM state, the telemetry struct — but they're
written by exactly one thread each. Everything else flows through
queues. This makes debugging tractable: if a value is wrong, you grep
for the queue, not for the variable.

## Cortex-M7 gotcha (Buzz V4 only)

The H743 has a **D-Cache**. DMA-written buffers can become stale in
cache, so any DMA buffer must live in non-cacheable SRAM. Two macros
do this:

```c
DMA_BUFFER  uint8_t imu_dma_buf[64];     // D2 SRAM, for DMA1/DMA2
BDMA_BUFFER uint8_t mag_dma_buf[16];     // D3 SRAM, for SPI6 BDMA
```

On the F446ZE these macros expand to nothing. You don't need to
remember which is which when writing portable driver code — just slap
the appropriate macro on the buffer.

Linker sections `.dma_buffer` and `.bdma_buffer` are defined in
`STM32H743ZIT6_Magalhaes/STM32H743ZITX_FLASH.ld`.

## Where things actually live

| If you're looking for... | Open... |
|---|---|
| A new sensor driver | `Codigo/Sensors/<NAME>/` |
| Something that runs on every IMU sample | `sensors_thread.c` → callback path |
| What happens at LIFTOFF | `flight_computer.c`, search `STATE_FLIGHT` |
| The packet a telemetry byte ended up in | `Codigo/Telemetry/telemetry.h` (start with `@ref TelemetryPackets`) |
| Whether SPI3 or SPI6 is the magnetometer | `STM32{H743,F446}.../config.h`, macro `SPI_MAG` |

## Read next

- @ref flight_lifecycle — the FSM as a story, BOOT to SAFE.
- @ref porting_guide — adding a new MCU or sensor.
