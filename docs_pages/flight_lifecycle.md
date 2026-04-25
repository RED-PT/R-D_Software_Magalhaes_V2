# Flight Lifecycle {#flight_lifecycle}

[TOC]

The flight computer is one big state machine. This page walks every
state, what triggers a transition, and what the rest of the system is
expected to be doing.

The authoritative source is @ref FSM in
`Codigo/Flight Computer/flight_computer.h`.

## State diagram

```
        BOOT --(self-test ok)--> IDLE
                                  |
                  (config rcvd) <-+
                                  |
                                  v
                              CONFIGED
                                  |
                          (ARM cmd) |
                                  v
                                ARMED ---(TEST cmd)---> TEST_STAND
                                  |                          |
                          (LAUNCH cmd) <----- (test done) ---+
                                  v
                               FLIGHT
                              (ascent → apogee → descent → landing)
                                  |
                                  v
                                SAFE

       At any point: ABORT cmd or fatal fault --> ABORT --> SAFE
```

## State by state

### BOOT
On reset. The MCU initialises peripherals (HAL → CubeMX init), starts
FreeRTOS, and the FSM thread runs once it's scheduled. Each sensor
driver runs `_Init()` and reports back; if a critical sensor fails
(IMU, barometer), the FSM stays in BOOT and lights an error LED.

You don't normally need to touch this state — it's automatic.

### IDLE
Self-test passed. The FC is alive, broadcasting fast telemetry (so the
ground station sees it), but **nothing else is happening**. ESC is
disarmed, pyrotechnics inhibited.

The FC sits here until the ground station sends a configuration
packet (flight profile: motor params, deploy altitudes, abort
thresholds). See @ref Commands.

### CONFIGED
A flight profile is loaded and validated. The system is now waiting
for `ARM`. No actuators are powered.

This state exists so an operator can verify the configuration on the
dashboard before committing.

### ARMED
The motor is initialised (ESC armed, idling at min throttle), the
deploy charges are connected, the FSM is watching for either:

- `LAUNCH` → enter FLIGHT.
- `TEST` → enter TEST_STAND (only if the profile permits it).
- `ABORT` → cut everything, jump to SAFE.

**Anything that runs the motor lives downstream of ARMED.** That is
the safety boundary.

### TEST_STAND
Bench-only. Used for static thrust characterisation. Drives the FX29
load cell, ramps throttle per the test profile, logs to SD card, and
streams progress telemetry to the dashboard. See @ref StaticTestAPI.

Returns to ARMED when the test completes (or to SAFE on abort).

### FLIGHT
The active flight phase. Internally subdivided:

1. **Ignition** — motor command issued; the FSM watches accelerometer
   for liftoff confirmation (vertical accel > threshold for N samples).
2. **Ascent** — controller is closing the TVC loop, telemetry rate
   increases.
3. **Apogee** — detected via barometer (altitude derivative crosses
   zero) with IMU-based fallback.
4. **Descent** — drogue deploy → main deploy at programmed altitude.
5. **Landing** — accel and altitude both stable for N seconds.

Transitions to SAFE on landing, or to ABORT on detected fault.

### ABORT
Emergency shutdown. ESC commanded off, deploy charges fired (or
inhibited, depending on phase), telemetry switched to abort packet.
Always lands in SAFE.

### SAFE
Post-flight idle. Logs continue to flush; nothing actuates. Power-cycle
to start over.

## What "an event" means

The FSM both **receives** commands (from radio) and **emits** events
(for telemetry and the SD card). See @ref Events for the enumeration.
A typical FLIGHT trajectory writes:

```
EVT_LIFTOFF
EVT_BURNOUT
EVT_APOGEE
EVT_DROGUE_DEPLOY
EVT_MAIN_DEPLOY
EVT_LANDED
```

Events are immutable timestamps in the SD log; they are how you
reconstruct what happened post-flight.

## When you're modifying the FSM

Two rules:

1. **Add a state, don't repurpose one.** Other code (telemetry, SD
   logging) inspects the state numerically; reusing a value silently
   breaks them.
2. **Anything that arms the motor or fires pyros must check the
   current state.** The driver-level functions don't enforce this —
   the FSM does.
