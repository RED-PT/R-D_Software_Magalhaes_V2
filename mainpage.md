# Magalhães Flight Computer {#mainpage}

A TVC (Thrust Vector Control) flight computer for experimental rockets,
developed by **R&D Software** at AeroTéc. The same firmware runs on two
boards: a custom **Buzz V4** (STM32H743) and a **Nucleo F446ZE** dev
board.

This documentation is organised as a **guided tour**. If this is your
first time reading the codebase, follow the pages in order. If you
already know the system and want a specific function, use the sidebar
or the search bar.

---

## Start here

1. **@subpage getting_started** — set up your environment, flash a board,
   talk to the ground station. ~10 minutes.
2. **@subpage architecture_tour** — the eight FreeRTOS threads, how data
   moves between them, and *why* it's split that way.
3. **@subpage flight_lifecycle** — the FSM from BOOT to SAFE, including
   what each transition expects from the rest of the system.
4. **@subpage porting_guide** — adding a new MCU target or swapping a
   sensor. Read this *after* the architecture tour.

## Reference

- **Module reference** — see the *Modules* item in the sidebar. Groups
  are organised top-down: hardware → sensors → estimation → control →
  comms → data → flight core → tests.
- **File reference** — see *Files* in the sidebar.

---

## At a glance

The flight computer is a hard-real-time system built on FreeRTOS. Eight
threads cooperate via queues and notifications; nothing is shared
through globals that can race.

```
   [ Sensors ] -- DMA done --> [ DataHandler ] --queue--> [ SDCard ]
                                      |
                                      +--queue--> [ Estimator (100Hz) ]
                                                       |
                                                       v
                                                  [ Controller (100Hz) ]
                                                       |
                                                       v
                                                    [ ESC PWM ]

   [ FSM (20Hz) ] <-- commands -- [ Radio (TDMA) ] <-- ground station
        |
        +-- events --> [ Telemetry (20Hz) ] --packets--> [ Radio ]
```

Decisions you'll see throughout:

- **Sensor reads are DMA-driven**, never blocking. The sensors thread
  unblocks on a callback notification, not a delay.
- **Telemetry uses TDMA**, not handshakes. The ground station sends a
  sync packet at slot 0; the FC transmits in fixed slots. No collisions,
  no retries, predictable airtime.
- **State estimation is decoupled from control**. Estimator publishes;
  controller subscribes. Either can be replaced without touching the
  other.
- **Hardware differences live in `config.h`**. The application code
  never sees `&hspi5` vs `&hspi3` — only `SPI_MAG`, `SPI_LORA`, etc.

For the rationale behind each, see @ref architecture_tour.

---

## Hardware

Two boards are supported. They share `Codigo/` verbatim; everything
board-specific is a macro in `config.h`.

| Board | MCU | Clock | Radio | Notes |
|-------|-----|-------|-------|-------|
| **Buzz V4** (primary) | STM32H743ZIT6 (Cortex-M7) | 480 MHz | E22-900M22S via SPI | Custom PCB, D-Cache, 3 SRAM domains |
| **Nucleo F446ZE** | STM32F446ZE (Cortex-M4) | 180 MHz | E22-xxxT30D via UART | Dev board, simpler memory model |

Full pinouts and abstraction tables live in @ref getting_started and
@ref porting_guide.

---

## Authors and licence

**Tomás Teixeira** — lead developer.
Proprietary, R&D internal use.
