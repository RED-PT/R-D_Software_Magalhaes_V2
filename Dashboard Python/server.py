"""
@file server.py
@brief Ground Station Dashboard Server
@author Tomás Teixeira
@date 2025
@version 2.0

@details
FastAPI-based web server for the Magalhães Flight Computer Ground Station.
Provides real-time telemetry visualization, command interface, and data
analysis through a modern web dashboard.

## Features
- Real-time WebSocket telemetry streaming
- Serial communication with Arduino ground station
- Binary protocol parsing (Fast/Slow/Event packets)
- Command relay to flight computer
- Demo mode for testing without hardware

## Architecture
```
┌─────────────┐     Serial      ┌─────────────┐     LoRa      ┌──────────────┐
│  Dashboard  │◄───(USB)────────│   Arduino   │◄──────────────│    Flight    │
│   Server    │    115200 baud  │     GS      │   433 MHz     │   Computer   │
└──────┬──────┘                 └─────────────┘               └──────────────┘
       │
       │ WebSocket
       ▼
┌─────────────┐
│   Browser   │
│  Dashboard  │
└─────────────┘
```

## Binary Protocol
All messages use framed format:
| Field | Size | Description |
|-------|------|-------------|
| Sync1 | 1 | 0xAA |
| Sync2 | 1 | 0x55 |
| Length | 2 | Payload length (big-endian) |
| Type | 1 | Message type ID |
| Payload | N | Message-specific data |
| CRC16 | 2 | CRC-16 checksum |

## Usage
```bash
python server.py                    # Normal mode
python server.py --demo             # Demo mode (no hardware)
python server.py --port COM3        # Specify serial port
```

## API Endpoints
- GET  /           - Serve dashboard HTML
- GET  /api/ports  - List available serial ports
- GET  /api/status - Connection status
- GET  /api/help   - Command documentation
- POST /api/connect    - Connect to serial port
- POST /api/disconnect - Disconnect
- WS   /ws         - WebSocket for real-time data

@see app.js for frontend JavaScript
@see index.html for dashboard UI
"""

import argparse
import asyncio
import json
import struct
import threading
import time
from collections import deque
from typing import Optional

import serial
import serial.tools.list_ports
from contextlib import asynccontextmanager
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

# =============================================================================
# Protocol Constants
# =============================================================================

FRAME_SYNC_1 = 0xAA  #: First sync byte for frame detection
FRAME_SYNC_2 = 0x55  #: Second sync byte for frame detection

# Message type identifiers (must match Arduino GS firmware)
MSG_TYPE_FC_FAST = 0x01    #: Fast telemetry packet (8 Hz) - IMU, altitude, attitude
MSG_TYPE_FC_SLOW = 0x02    #: Slow telemetry packet (1 Hz) - GPS, temperature
MSG_TYPE_FC_EVENT = 0x03   #: Event packet - state changes, alerts
MSG_TYPE_GS_STATUS = 0x10  #: Ground station status update
MSG_TYPE_GS_ACK = 0x11     #: Command acknowledgment
MSG_TYPE_GS_PONG = 0x12    #: Ping response with RTT
MSG_TYPE_GS_STATS = 0x13   #: Communication statistics
MSG_TYPE_GS_LOG = 0x20     #: Debug log message from GS

COMMANDS_DOC = [
    ("P", "PING", "Send ping to FC, measures round-trip latency"),
    ("C", "CALIBRATE", "Calibrate barometer - sets current altitude as 0m AGL"),
    ("M", "MOTOR CAL", "Calibrate ESC min/max throttle - POWER CYCLE ESC DURING CALIBRATION!"),
    ("EXX", "STATIC TEST", "Static thrust test at XX% throttle (e.g., E20 for 20%). Saves Thrust-PWM data to SD."),
    ("1", "PROFILE 1", "Gutter Ramp: TEST_STAND mode, 5s ramp to 100%"),
    ("2", "PROFILE 2", "Gutter Hold: TEST_STAND mode, hold 30% for 10s"),
    ("3", "PROFILE 3", "Flight Mode: FLIGHT mode, 50m target, active TVC"),
    ("A", "ARM", "Arm flight computer and motors"),
    ("D", "DISARM", "Disarm flight computer"),
    ("T", "TEST", "Start test stand sequence"),
    ("L", "LAUNCH", "Initiate launch sequence"),
    ("X", "ABORT", "Emergency abort - cut throttle immediately"),
    ("S", "SAFE", "Force transition to SAFE state"),
    ("R", "RESET STATS", "Reset communication statistics"),
    ("?", "DEBUG", "Toggle debug output on Arduino"),
]

PROFILE_DETAILS = {
    1: {
        "name": "Gutter Ramp Test",
        "mode": "TEST_STAND",
        "description": "Gradually ramps throttle from 0% to 100% over 5 seconds",
        "params": [
            ("Ramp Time", "5 seconds"),
            ("Max Throttle", "100%"),
            ("Safety Timeout", "30 seconds"),
        ],
        "use_for": "Testing motor thrust curve and TVC servo response",
    },
    2: {
        "name": "Gutter Hold Test",
        "mode": "TEST_STAND",
        "description": "Holds constant throttle for steady-state testing",
        "params": [
            ("Hold Throttle", "30%"),
            ("Duration", "10 seconds"),
            ("Safety Timeout", "30 seconds"),
        ],
        "use_for": "Steady-state testing and thermal validation",
    },
    3: {
        "name": "Flight Mode",
        "mode": "FLIGHT",
        "description": "Full flight profile with active guidance",
        "params": [
            ("Target Altitude", "50 meters"),
            ("Guidance", "Active TVC"),
            ("Descent", "Controlled burn"),
        ],
        "use_for": "Actual rocket flights with altitude targeting",
    },
}


# =============================================================================
# Packet Parsing Functions
# =============================================================================

def parse_fast_packet(data):
    """
    Parse a fast telemetry packet from the flight computer.

    Fast packets are transmitted at 8 Hz (slots 0-7) and contain:
    - IMU data (accelerometer, gyroscope)
    - Altitude and vertical speed (vario)
    - Attitude (pitch, roll, yaw)
    - FSM state information
    - ACK piggyback for command confirmation

    @param data: Raw packet bytes (minimum 37 bytes)
    @return: Dictionary with parsed fields, or None if invalid

    Packet format (37 bytes):
    | Offset | Type | Field |
    |--------|------|-------|
    | 0 | u8 | packet_type (0x01) |
    | 1 | u8 | frame_id |
    | 2 | u8 | slot_id |
    | 3 | u8 | sequence |
    | 4 | u8 | flags |
    | 5 | u32 | time_ms |
    | 9 | u8 | state |
    | 10 | u8 | substate |
    | 11 | u8 | ack_seq |
    | 12 | u8 | ack_status |
    | 13 | i16[3] | accel_x/y/z (mG) |
    | 19 | i16[3] | gyro_x/y/z (0.01 dps) |
    | 25 | i16 | altitude (0.1 m) |
    | 27 | i16 | vario (0.01 m/s) |
    | 29 | i16[3] | pitch/roll/yaw (0.1 deg) |
    | 35 | u16 | CRC16 |
    """
    if len(data) < 37:
        return None
    fmt = '<BBBBBIBBBB3h3h5hH'
    u = struct.unpack(fmt, data[:37])
    return {
        'type': 'fast',
        'frame_id': u[1], 'slot_id': u[2], 'seq': u[3], 'flags': u[4],
        'time_ms': u[5], 'state': u[6], 'substate': u[7],
        'ack_seq': u[8], 'ack_status': u[9],
        'accel_x': u[10] / 1000.0, 'accel_y': u[11] / 1000.0, 'accel_z': u[12] / 1000.0,
        'gyro_x': u[13] / 100.0, 'gyro_y': u[14] / 100.0, 'gyro_z': u[15] / 100.0,
        'altitude': u[16] / 10.0, 'vario': u[17] / 100.0,
        'pitch': u[18] / 10.0, 'roll': u[19] / 10.0, 'yaw': u[20] / 10.0,
    }


# Phase 3-A (A5): CRC-16/MODBUS — matches Codigo/Radio/CRC16/crc16.c
def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


# Phase 3-A (A5): pack a test_control_packet_t (15 bytes wire) for the GS link.
# Wire format must match telemetry.h::test_control_packet_t exactly.
TELEM_PACKET_TEST_CTRL = 0x06
TEST_CTRL_FIELD_THROTTLE  = 1 << 0
TEST_CTRL_FIELD_ALPHA     = 1 << 1
TEST_CTRL_FIELD_H_REF     = 1 << 2
TEST_CTRL_FIELD_H_REF_DOT = 1 << 3
TEST_CTRL_FLAG_RUN   = 1 << 0
TEST_CTRL_FLAG_HOLD  = 1 << 1
TEST_CTRL_FLAG_ABORT = 1 << 2

_test_ctrl_seq = 0


# Phase 3-A (A7): pack the test profile into params[32] for CMD_SET_TEST_PROFILE.
# Wire layouts must match parse_test_profile_wire() in flight_computer_thread.c.
PROFILE_KIND_STATIC = 2
PROFILE_KIND_TORQUE = 3
PROFILE_KIND_TORQUE_CAL = 4
PROFILE_KIND_GUTTER = 5
CURVE_STEP = 0
CURVE_RAMP = 1


def _curve_byte(s):
    if isinstance(s, (int, float)):
        return int(s) & 0xFF
    if isinstance(s, str) and s.upper() == 'RAMP':
        return CURVE_RAMP
    return CURVE_STEP


def build_set_test_profile_params(profile):
    """Return a 32-byte buffer for CMD_SET_TEST_PROFILE.params[].
    The first byte is profile_kind_t; the rest is kind-specific. """
    buf = bytearray(32)
    kind = profile.get('kind', 'static')
    mode = profile.get('mode', 'auto')
    is_manual = 1 if mode == 'manual' else 0

    if kind == 'static':
        # JS gives throttle as percent (0..100); wire wants milli (0..1000).
        if mode == 'auto':
            throttle_milli = int(round(profile.get('max_throttle', 0) * 10))
            hold_ms = int(profile.get('hold_ms', 0))
            curve = _curve_byte(profile.get('curve', 'STEP'))
        else:
            throttle_milli = int(round(profile.get('max_throttle_clamp', 0) * 10))
            hold_ms = 0
            curve = CURVE_STEP
        buf[0] = PROFILE_KIND_STATIC
        buf[1] = is_manual
        struct.pack_into('<H', buf, 2, max(0, min(1000, throttle_milli)))
        struct.pack_into('<H', buf, 4, max(0, min(0xFFFF, hold_ms)))
        buf[6] = curve

    elif kind == 'gutter':
        # Assumes JS-side inputs are already in wire units (dm, ms, raw float gains).
        # If labels say "meters", multiply by 10 here.
        hold_ms       = int(profile.get('hold_ms', 0))
        h_ref_min_dm  = int(profile.get('h_ref_min', 0))
        h_ref_max_dm  = int(profile.get('h_ref_max', 0))
        h_ref_init_dm = int(profile.get('initial_h_ref', h_ref_min_dm))
        curve = _curve_byte(profile.get('curve', 'STEP'))
        kp = float(profile.get('kp', 0.5))
        ki = float(profile.get('ki', 0.1))
        kd = float(profile.get('kd', 0.2))
        ilim = float(profile.get('integral_limit', 0.3))
        buf[0] = PROFILE_KIND_GUTTER
        buf[1] = is_manual
        struct.pack_into('<H', buf, 2, max(0, min(0xFFFF, hold_ms)))
        struct.pack_into('<H', buf, 4, max(0, min(0xFFFF, h_ref_min_dm)))
        struct.pack_into('<H', buf, 6, max(0, min(0xFFFF, h_ref_max_dm)))
        struct.pack_into('<h', buf, 8, max(-32768, min(32767, h_ref_init_dm)))
        buf[10] = curve
        struct.pack_into('<f', buf, 12, kp)
        struct.pack_into('<f', buf, 16, ki)
        struct.pack_into('<f', buf, 20, kd)
        struct.pack_into('<f', buf, 24, ilim)

    elif kind in ('torque', 'torque_cal'):
        # B1 / B2: recruta defines wire layout. For now mark as kind only.
        buf[0] = PROFILE_KIND_TORQUE if kind == 'torque' else PROFILE_KIND_TORQUE_CAL
        buf[1] = is_manual

    else:
        return None

    return bytes(buf)


def build_test_control_packet(throttle_milli=0, alpha_centideg=0,
                              h_ref_dm=0, h_ref_dot_cms=0,
                              field_mask=0, flags=0):
    """Pack the 15-byte test_control wire packet (frame_id=0, FC ignores it).
    Caller is expected to clamp values; we only do the format-level packing. """
    global _test_ctrl_seq
    _test_ctrl_seq = (_test_ctrl_seq + 1) & 0xFF
    body = struct.pack(
        '<BBBBHhhhB',
        TELEM_PACKET_TEST_CTRL,
        0,                                 # frame_id (filled by Arduino if ever needed)
        _test_ctrl_seq,
        field_mask & 0xFF,
        max(0, min(0xFFFF, int(throttle_milli))),
        max(-32768, min(32767, int(alpha_centideg))),
        max(-32768, min(32767, int(h_ref_dm))),
        max(-32768, min(32767, int(h_ref_dot_cms))),
        flags & 0xFF,
    )
    crc = crc16_modbus(body)
    return body + struct.pack('<H', crc)


def parse_slow_packet(data):
    # Phase 3-A (A4): slow grew from 30 → 32 bytes (added tdma_mode + reserved)
    if len(data) < 32:
        return None
    fmt = '<BBBBIiiHBBhhBBHBBH'
    u = struct.unpack(fmt, data[:32])
    return {
        'type': 'slow',
        'frame_id': u[1], 'slot_id': u[2], 'seq': u[3], 'time_ms': u[4],
        'latitude': u[5] / 1e7, 'longitude': u[6] / 1e7,
        'gps_altitude': u[7] / 10.0, 'gps_lock': u[8], 'satellites': u[9],
        'pressure': (u[10] / 10.0) + 1000.0, 'temperature': u[11] / 10.0,
        'battery': u[12], 'sd_status': u[13], 'free_heap': u[14] * 10,
        'tdma_mode': u[15],  # 0 = FLIGHT, 1 = TEST_INTERACTIVE
    }


def parse_event_packet(data):
    if len(data) < 37:
        return None
    fmt = '<BBBBIBBB24sH'
    u = struct.unpack(fmt, data[:37])
    return {
        'type': 'event',
        'frame_id': u[1], 'slot_id': u[2], 'seq': u[3], 'time_ms': u[4],
        'event_type': u[5], 'state': u[6], 'substate': u[7],
        'payload': u[8].hex(),
    }


def parse_gs_status(data):
    if len(data) < 11:
        return None
    fmt = '<BBBBBBBI'
    u = struct.unpack(fmt, data[:11])
    return {
        'type': 'gs_status',
        'synced': u[0], 'frame_id': u[1], 'slot_id': u[2],
        'cmd_pending': u[3], 'awaiting_ack': u[4],
        'last_ack_seq': u[5], 'last_ack_status': u[6],
        'uptime_ms': u[7],
    }


def parse_gs_pong(data):
    if len(data) < 9:
        return None
    fmt = '<BII'
    u = struct.unpack(fmt, data[:9])
    return {
        'type': 'pong',
        'ping_seq': u[0], 'rtt_ms': u[1], 'fc_timestamp': u[2],
    }


def parse_gs_stats(data):
    if len(data) < 32:
        return None
    fmt = '<8I'
    u = struct.unpack(fmt, data[:32])
    return {
        'type': 'gs_stats',
        'rx_fast': u[0], 'rx_slow': u[1], 'rx_event': u[2],
        'tx_cmd': u[3], 'tx_sync': u[4], 'crc_errors': u[5],
        'ack_ok': u[6], 'ack_timeout': u[7],
    }


# =============================================================================
# WebSocket Connection Management
# =============================================================================

class ConnectionManager:
    """
    Manages WebSocket connections to browser clients.

    Keeps a small "sticky" snapshot of the latest state per packet type
    plus a ring buffer of recent events. New clients (e.g. after a browser
    refresh) receive the snapshot before live broadcast resumes, so they
    don't miss the boot report or current FSM state.

    @note Automatically removes disconnected clients on broadcast errors.
    """

    #: Number of recent events kept for replay to new clients.
    EVENT_HISTORY_LEN = 64

    #: Packet types we keep "last seen" of for snapshot replay.
    #: Thread stats arrive as 'event' packets and live in recent_events.
    STICKY_TYPES = (
        'fast', 'slow', 'gs_status', 'gs_stats', 'pong',
    )

    def __init__(self):
        """Initialize with empty connection set and empty snapshot."""
        self.active = set()  #: Set of active WebSocket connections
        self.snapshot = {t: None for t in self.STICKY_TYPES}
        self.recent_events = deque(maxlen=self.EVENT_HISTORY_LEN)

    async def connect(self, websocket: WebSocket):
        """
        Accept and register a new WebSocket connection, and replay the
        snapshot so the client immediately reflects current FC state.

        @param websocket: FastAPI WebSocket instance
        """
        await websocket.accept()
        self.active.add(websocket)
        await self._send_snapshot(websocket)

    def disconnect(self, websocket: WebSocket):
        """
        Remove a WebSocket from the active set.

        @param websocket: WebSocket to disconnect
        """
        self.active.discard(websocket)

    async def _send_snapshot(self, websocket: WebSocket):
        """Send last-known state + recent events to a freshly connected client."""
        try:
            for val in self.snapshot.values():
                if val is not None:
                    await websocket.send_text(json.dumps({**val, 'replay': True}))
            for evt in self.recent_events:
                await websocket.send_text(json.dumps({**evt, 'replay': True}))
            await websocket.send_text(json.dumps({'type': 'snapshot_complete'}))
        except Exception:
            self.disconnect(websocket)

    def _update_snapshot(self, message: dict):
        """Stash the message in the snapshot (sticky types) or event ring."""
        t = message.get('type')
        if t in self.snapshot:
            self.snapshot[t] = message
        elif t == 'event':
            self.recent_events.append(message)

    async def broadcast(self, message: dict):
        """
        Send a message to all connected WebSocket clients and update snapshot.

        @param message: Dictionary to send (JSON serialized)
        @note Failed sends result in automatic client disconnect
        """
        self._update_snapshot(message)
        if not self.active:
            return
        data = json.dumps(message)
        dead = []
        for ws in self.active:
            try:
                await ws.send_text(data)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.disconnect(ws)


# =============================================================================
# Serial Communication
# =============================================================================

class SerialReader:
    """
    Threaded serial port reader for Arduino Ground Station communication.

    Handles:
    - Asynchronous serial reading in background thread
    - Binary frame detection and parsing
    - Command transmission to GS
    - Error recovery

    The reader uses a framed protocol with sync bytes (0xAA 0x55) to
    detect packet boundaries reliably.

    @note Runs in a daemon thread to avoid blocking the async event loop.
    """

    def __init__(self, loop, queue):
        """
        Initialize the serial reader.

        @param loop: asyncio event loop for cross-thread communication
        @param queue: asyncio.Queue for parsed packets
        """
        self.loop = loop            #: Asyncio event loop reference
        self.queue = queue          #: Queue for parsed packets
        self.serial = None          #: pyserial Serial instance
        self.running = False        #: Thread running flag
        self.rx_buffer = bytearray()  #: Receive buffer for frame assembly
        self.thread = None          #: Background reader thread
        self.cmd_queue = deque()    #: Pending commands to send
        self.last_rx_time = None    #: Timestamp of last received data
        self.last_error = None      #: Last error message

    def connect(self, port, baudrate=115200):
        if self.running:
            self.disconnect()
        self.serial = serial.Serial(port, baudrate, timeout=0.05)
        self.running = True
        self.last_rx_time = time.time()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def disconnect(self):
        self.running = False
        if self.serial:
            try:
                self.serial.close()
            except Exception:
                pass
            self.serial = None
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self.thread = None
        self.rx_buffer = bytearray()

    def send_command(self, cmd):
        if cmd:
            self.cmd_queue.append(cmd)

    def run(self):
        while self.running:
            if not self.serial or not self.serial.is_open:
                time.sleep(0.1)
                continue
            try:
                while self.cmd_queue:
                    cmd = self.cmd_queue.popleft()
                    self.serial.write(cmd.encode() if isinstance(cmd, str) else cmd)
                data = self.serial.read(256)
                if data:
                    self.last_rx_time = time.time()
                    self.rx_buffer.extend(data)
                    self.process_buffer()
            except Exception:
                self.last_error = "serial read error"
                time.sleep(0.1)

    def process_buffer(self):
        while len(self.rx_buffer) >= 6:
            idx = 0
            while idx < len(self.rx_buffer) - 1:
                if self.rx_buffer[idx] == FRAME_SYNC_1 and self.rx_buffer[idx + 1] == FRAME_SYNC_2:
                    break
                idx += 1
            if idx > 0:
                self.rx_buffer = self.rx_buffer[idx:]
            if len(self.rx_buffer) < 6:
                break
            length = (self.rx_buffer[2] << 8) | self.rx_buffer[3]
            total_len = 4 + length + 2
            if length > 256:
                self.rx_buffer = self.rx_buffer[2:]
                continue
            if len(self.rx_buffer) < total_len:
                break
            frame = self.rx_buffer[:total_len]
            self.rx_buffer = self.rx_buffer[total_len:]
            msg_type = frame[4]
            payload = bytes(frame[5:-2])
            self.parse_message(msg_type, payload)

    def parse_message(self, msg_type, payload):
        pkt = None
        if msg_type == MSG_TYPE_FC_FAST:
            pkt = parse_fast_packet(payload)
        elif msg_type == MSG_TYPE_FC_SLOW:
            pkt = parse_slow_packet(payload)
        elif msg_type == MSG_TYPE_FC_EVENT:
            pkt = parse_event_packet(payload)
        elif msg_type == MSG_TYPE_GS_STATUS:
            pkt = parse_gs_status(payload)
        elif msg_type == MSG_TYPE_GS_PONG:
            pkt = parse_gs_pong(payload)
        elif msg_type == MSG_TYPE_GS_STATS:
            pkt = parse_gs_stats(payload)
        elif msg_type == MSG_TYPE_GS_LOG:
            pkt = {'type': 'log', 'message': payload.decode('utf-8', errors='replace')}
        if pkt:
            self.loop.call_soon_threadsafe(self.queue.put_nowait, pkt)


async def demo_stream(queue):
    yaw = 0.0
    while True:
        t = time.time()
        yaw = (yaw + 1.2) % 360
        fast = {
            'type': 'fast',
            'altitude': 120 + 5 * (1 + (t % 6) / 6.0),
            'vario': 2.4,
            'accel_x': 0.02,
            'accel_y': 0.01,
            'accel_z': 1.01,
            'gyro_x': 0.2,
            'gyro_y': 0.3,
            'gyro_z': 1.1,
            'pitch': 3.2,
            'roll': 2.1,
        }
        slow = {
            'type': 'slow',
            'temperature': 24.6,
            'pressure': 1020.2,
            'battery': 87,
            'latitude': 38.7223,
            'longitude': -9.1393,
            'gps_altitude': 110.2,
            'gps_lock': 3,
            'satellites': 6,
        }
        stats = {
            'type': 'gs_stats',
            'rx_fast': int(t * 2) % 10000,
            'rx_slow': int(t) % 10000,
            'rx_event': 10,
            'tx_cmd': int(t) % 1000,
            'tx_sync': int(t) % 1000,
            'crc_errors': 0,
            'ack_ok': int(t) % 100,
            'ack_timeout': 0,
        }
        status = {
            'type': 'gs_status',
            'synced': 1,
        }
        await queue.put(fast)
        await queue.put(slow)
        await queue.put(stats)
        await queue.put(status)
        await asyncio.sleep(0.05)


@asynccontextmanager
async def lifespan(app: FastAPI):
    app.state.queue = asyncio.Queue()
    app.state.reader = None
    app.state.connected_port = None
    if app.state.demo_mode:
        asyncio.create_task(demo_stream(app.state.queue))
    elif app.state.pending_port:
        try:
            app.state.reader = SerialReader(asyncio.get_event_loop(), app.state.queue)
            app.state.reader.connect(app.state.pending_port)
            app.state.connected_port = app.state.pending_port
        except Exception:
            app.state.reader = None

    async def broadcast_loop():
        while True:
            pkt = await app.state.queue.get()
            await manager.broadcast(pkt)

    async def watchdog_loop():
        while True:
            await asyncio.sleep(1.0)
            if app.state.demo_mode:
                continue
            if not app.state.reader or not app.state.connected_port:
                continue
            last_rx = app.state.reader.last_rx_time
            if last_rx and (time.time() - last_rx) > 3.0:
                try:
                    app.state.reader.disconnect()
                    app.state.reader = SerialReader(asyncio.get_event_loop(), app.state.queue)
                    app.state.reader.connect(app.state.connected_port)
                except Exception:
                    pass

    asyncio.create_task(broadcast_loop())
    asyncio.create_task(watchdog_loop())
    yield

app = FastAPI(lifespan=lifespan)
app.mount('/static', StaticFiles(directory='web', html=True), name='static')

# Disable caching for development - forces browser to always get fresh files
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import Response

class NoCacheMiddleware(BaseHTTPMiddleware):
    async def dispatch(self, request, call_next):
        response: Response = await call_next(request)
        if request.url.path.startswith('/static'):
            response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
            response.headers['Pragma'] = 'no-cache'
            response.headers['Expires'] = '0'
        return response

app.add_middleware(NoCacheMiddleware)

manager = ConnectionManager()


@app.get('/')
def index():
    return FileResponse('web/index.html')


@app.get('/api/ports')
def list_ports():
    ports = []
    for port in serial.tools.list_ports.comports():
        ports.append({'device': port.device, 'description': port.description})
    return {'ports': ports}

@app.get('/api/status')
def api_status():
    last_rx = None
    last_error = None
    if app.state.reader:
        last_rx = app.state.reader.last_rx_time
        last_error = app.state.reader.last_error
    return {
        'connected': bool(app.state.reader),
        'port': app.state.connected_port,
        'last_rx': last_rx,
        'last_error': last_error,
    }

@app.get('/api/help')
def api_help():
    profiles = []
    for key, value in PROFILE_DETAILS.items():
        profiles.append({'id': key, **value})
    return {
        'commands': [{'cmd': c[0], 'name': c[1], 'desc': c[2]} for c in COMMANDS_DOC],
        'profiles': profiles,
    }


@app.post('/api/connect')
async def api_connect(payload: dict):
    port = payload.get('port')
    if not port:
        return {'ok': False, 'error': 'port required'}
    if app.state.reader:
        app.state.reader.disconnect()
    app.state.connected_port = None
    try:
        app.state.reader = SerialReader(asyncio.get_event_loop(), app.state.queue)
        app.state.reader.connect(port)
        app.state.connected_port = port
        return {'ok': True}
    except Exception as exc:
        return {'ok': False, 'error': str(exc)}


@app.post('/api/disconnect')
async def api_disconnect():
    if app.state.reader:
        app.state.reader.disconnect()
    app.state.reader = None
    app.state.connected_port = None
    return {'ok': True}


@app.websocket('/ws')
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            try:
                msg = json.loads(data)
            except Exception:
                continue

            kind = msg.get('type')
            if kind == 'cmd' and app.state.reader:
                app.state.reader.send_command(msg.get('cmd'))

            elif kind == 'test_cmd' and app.state.reader:
                # Phase 3-A (A3/A7): translate test-page lifecycle commands to
                # the matching Arduino magic chars / binary frames.
                tc = msg.get('cmd', '').upper()
                if tc == 'SET_PROFILE':
                    params = build_set_test_profile_params(msg.get('profile') or {})
                    if params is not None:
                        # PC->Arduino framing: 0xAA 0x55 + type(0x02=TEST_PROFILE) + 32 bytes
                        app.state.reader.send_command(b'\xAA\x55\x02' + params)
                elif tc == 'ARM':    app.state.reader.send_command('A')
                elif tc == 'RUN':    app.state.reader.send_command('T')   # CMD_START_TEST
                elif tc == 'HOLD':   app.state.reader.send_command('H')
                elif tc == 'RESUME': app.state.reader.send_command('U')
                elif tc == 'STOP':   app.state.reader.send_command('K')   # CMD_STOP_TEST
                elif tc == 'ABORT':  app.state.reader.send_command('X')
                else:
                    print(f"[TEST_CMD] unhandled cmd={tc}")

            elif kind == 'test_control' and app.state.reader:
                # Phase 3-A (A5): pack the slider state into a 15-byte
                # test_control_packet_t and ship it to the Arduino with the
                # '~' magic prefix. Arduino queues it for the next test-mode
                # TX slot; the FC parses it on RX and feeds on_run_tick().
                pkt = build_test_control_packet(
                    throttle_milli=msg.get('throttle_milli', 0),
                    alpha_centideg=msg.get('alpha_centideg', 0),
                    h_ref_dm=msg.get('h_ref_dm', 0),
                    h_ref_dot_cms=msg.get('h_ref_dot_cms', 0),
                    field_mask=msg.get('field_mask', 0),
                    flags=msg.get('flags', 0),
                )
                # PC->Arduino framing: 0xAA 0x55 + type(0x01=TEST_CTRL) + 15 bytes
                app.state.reader.send_command(b'\xAA\x55\x01' + pkt)
    except WebSocketDisconnect:
        manager.disconnect(websocket)



def main():
    parser = argparse.ArgumentParser(description='Web Dashboard Server')
    parser.add_argument('--port', type=str, help='Serial port to open')
    parser.add_argument('--demo', action='store_true', help='Run with demo data')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--http-port', default=8000, type=int)
    args = parser.parse_args()

    app.state.demo_mode = args.demo
    app.state.pending_port = args.port

    import uvicorn
    uvicorn.run(app, host=args.host, port=args.http_port)


if __name__ == '__main__':
    main()
