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

FRAME_SYNC_1 = 0xAA
FRAME_SYNC_2 = 0x55

MSG_TYPE_FC_FAST = 0x01
MSG_TYPE_FC_SLOW = 0x02
MSG_TYPE_FC_EVENT = 0x03
MSG_TYPE_GS_STATUS = 0x10
MSG_TYPE_GS_ACK = 0x11
MSG_TYPE_GS_PONG = 0x12
MSG_TYPE_GS_STATS = 0x13
MSG_TYPE_GS_LOG = 0x20

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


def parse_fast_packet(data):
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


def parse_slow_packet(data):
    if len(data) < 30:
        return None
    fmt = '<BBBBIiiHBBhhBBHH'
    u = struct.unpack(fmt, data[:30])
    return {
        'type': 'slow',
        'frame_id': u[1], 'slot_id': u[2], 'seq': u[3], 'time_ms': u[4],
        'latitude': u[5] / 1e7, 'longitude': u[6] / 1e7,
        'gps_altitude': u[7] / 10.0, 'gps_lock': u[8], 'satellites': u[9],
        'pressure': (u[10] / 10.0) + 1000.0, 'temperature': u[11] / 10.0,
        'battery': u[12], 'sd_status': u[13], 'free_heap': u[14] * 10,
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


class ConnectionManager:
    def __init__(self):
        self.active = set()

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active.add(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active.discard(websocket)

    async def broadcast(self, message: dict):
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


class SerialReader:
    def __init__(self, loop, queue):
        self.loop = loop
        self.queue = queue
        self.serial = None
        self.running = False
        self.rx_buffer = bytearray()
        self.thread = None
        self.cmd_queue = deque()
        self.last_rx_time = None
        self.last_error = None

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
            if msg.get('type') == 'cmd' and app.state.reader:
                app.state.reader.send_command(msg.get('cmd'))
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
