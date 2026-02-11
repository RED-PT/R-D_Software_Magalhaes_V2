# Magalhães Flight Computer Documentation {#mainpage}

Welcome to the official documentation for the **Magalhães Flight Computer** - a TVC (Thrust Vector Control) rocket flight computer system developed by R&D Software V2.

## System Overview

The Magalhães Flight Computer is a complete avionics system for experimental rockets featuring:

- **Thrust Vector Control** - Active motor gimbal control for stabilization
- **Multi-sensor Fusion** - IMU, barometer, magnetometer, GPS, and orientation sensors
- **Real-time Telemetry** - LoRa radio communication with ground station
- **Data Logging** - High-speed SD card logging for post-flight analysis
- **Autonomous Operation** - Full flight state machine from armed to recovery

## Hardware Platform

| Component | Model | Description |
|-----------|-------|-------------|
| MCU | STM32F446ZE | ARM Cortex-M4 @ 180 MHz |
| IMU | ASM330LHHX | 6-axis accel/gyro (6.6 kHz) |
| Orientation | BNO055 | 9-DOF fusion (100 Hz) |
| Barometer | MS5607 | Pressure/altitude (50 Hz) |
| Magnetometer | MMC5983MA | 3-axis compass (100 Hz) |
| GPS | u-blox NEO-7M | Position/velocity (1 Hz) |
| Radio | E22 (SX1262) | LoRa 433/868 MHz |
| Storage | microSD | FatFS logging |

## Software Architecture

The system uses FreeRTOS with 8 dedicated threads:

```
┌─────────────────────────────────────────────────────────────────┐
│                     Thread Architecture                          │
├─────────────────────────────────────────────────────────────────┤
│  Priority  │ Thread      │ Rate   │ Function                    │
├────────────┼─────────────┼────────┼─────────────────────────────┤
│  Highest   │ Sensors     │ Event  │ Sensor DMA handling         │
│  High2     │ DataHandler │ Event  │ Data routing to queues      │
│  High1     │ Radio       │ TDMA   │ LoRa communication          │
│  High      │ Estimator   │ 100Hz  │ State estimation            │
│  High      │ Controller  │ 100Hz  │ TVC control loop            │
│  AbvNorm1  │ FSM         │ 20Hz   │ Flight state machine        │
│  AbvNorm   │ Telemetry   │ 20Hz   │ Packet building             │
│  Normal    │ SDCard      │ Event  │ Data logging                │
└────────────┴─────────────┴────────┴─────────────────────────────┘
```

## Flight State Machine

@image html fsm_diagram.png "Flight State Machine" width=800px

The FSM manages the complete flight lifecycle:

1. **BOOT** - Hardware initialization and sensor verification
2. **IDLE** - Waiting for configuration from ground station
3. **CONFIGED** - Flight profile loaded, ready for arming
4. **ARMED** - Motor initialized, ready for launch or test
5. **TEST_STAND** - Static thrust testing mode
6. **FLIGHT** - Active flight (ignition → ascent → descent → landing)
7. **ABORT** - Emergency shutdown
8. **SAFE** - Post-flight safe state

## Communication Protocol

The system uses TDMA (Time Division Multiple Access) for ground station communication:

```
1000ms Superframe
├─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┤
│  0  │  1  │  2  │  3  │  4  │  5  │  6  │  7  │  8  │  9  │
├─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┤
│ Fast Telemetry (8 Hz)          │Slow │ GS  │
│ IMU + Altitude + State         │GPS  │ RX  │
└────────────────────────────────┴─────┴─────┘
```

## Module Documentation

### Core Modules
- @ref Flight_Computer - Central state machine and command processing
- @ref Sensors - Sensor drivers and data acquisition
- @ref Estimator - State estimation (altitude, velocity)
- @ref Controller - TVC and motor control

### Communication
- @ref Radio_Communication - LoRa radio and TDMA protocol
- @ref Telemetry - Packet formats and building

### Data Management
- @ref Data_Handler - Circular buffers and queue routing
- @ref Storage - SD card logging with FatFS

### Support
- @ref HAL_Callbacks - Hardware interrupt handling
- @ref Thread_Management - FreeRTOS thread creation

## Ground Station Components

### Python Dashboard
Web-based ground station interface:
- Real-time telemetry display
- 3D rocket visualization
- Map tracking
- Command interface
- Test data analysis

### Arduino Ground Station
Firmware for the ground station radio module:
- TDMA synchronization
- Command relay
- Telemetry forwarding to PC

## Building the Documentation

Generate this documentation using Doxygen:

```bash
doxygen Doxyfile
```

Then open `docs/html/index.html` in a web browser.

## Authors

- **Tomás Teixeira** - Lead Developer

## License

This project is proprietary software developed for R&D purposes.

---

*Generated with Doxygen - Magalhães Flight Computer v2.0*
