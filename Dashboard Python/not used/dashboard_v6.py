#!/usr/bin/env python3
"""
Flight Computer Ground Station Dashboard v5
============================================

New Features in v5:
- Boot Status Widget showing sensor initialization
- IMU X/Y/Z labels on plots
- Attitude Indicator (artificial horizon)
- GPS Map with OpenStreetMap tiles
- PWM Gauge for motor power
- Detailed Help Dialog with profile info
- Multiple fullscreen support (separate + tabbed)
- GPS data persistence with stale indicator

Requirements:
    pip install pyserial pyqt5 pyqtgraph numpy pyopengl PyQtWebEngine

Usage:
    python dashboard_v5.py --port COM3
"""

import sys
import argparse
import struct
import time
import math
from collections import deque
from datetime import datetime
from queue import Queue, Empty

import serial
import serial.tools.list_ports
import numpy as np

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QComboBox, QGroupBox,
    QTextEdit, QTabWidget, QFrame, QLineEdit, QPlainTextEdit,
    QStyleFactory, QDialog, QTableWidget, QTableWidgetItem,
    QHeaderView, QSizePolicy, QScrollArea, QSplitter, QStackedWidget
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread, QRectF, QPointF
from PyQt5.QtGui import QFont, QColor, QTextCursor, QPainter, QPen, QBrush, QRadialGradient, QLinearGradient, QPolygonF
from PyQt5.QtSvg import QSvgWidget

import pyqtgraph as pg
import pyqtgraph.opengl as gl

# Try to import WebEngine for GPS map
try:
    from PyQt5.QtWebEngineWidgets import QWebEngineView
    HAS_WEBENGINE = True
except ImportError:
    HAS_WEBENGINE = False
    print("Warning: PyQtWebEngine not installed. GPS map will be disabled.")
    print("Install with: pip install PyQtWebEngine")

# ============================================================================
# Protocol Constants
# ============================================================================

FRAME_SYNC_1 = 0xAA
FRAME_SYNC_2 = 0x55

MSG_TYPE_FC_FAST    = 0x01
MSG_TYPE_FC_SLOW    = 0x02
MSG_TYPE_FC_EVENT   = 0x03
MSG_TYPE_GS_STATUS  = 0x10
MSG_TYPE_GS_ACK     = 0x11
MSG_TYPE_GS_PONG    = 0x12
MSG_TYPE_GS_STATS   = 0x13
MSG_TYPE_GS_LOG     = 0x20

STATE_NAMES = ["BOOT", "IDLE", "CONFIGED", "ARMED", "TEST_STAND", "FLIGHT", "ABORT", "SAFE"]
STATE_COLORS = {
    0: "#666666",  # BOOT
    1: "#3498db",  # IDLE
    2: "#9b59b6",  # CONFIGED
    3: "#e67e22",  # ARMED
    4: "#1abc9c",  # TEST_STAND
    5: "#2ecc71",  # FLIGHT
    6: "#e74c3c",  # ABORT
    7: "#7f8c8d",  # SAFE
}
SUBSTATE_NAMES = [
    "NONE", "TS_SENSOR_CHECK", "TS_THROTTLE_RAMP",
    "FL_IGNITION", "FL_LIFTOFF_DETECT", "FL_ASCENT", "FL_COAST",
    "FL_DESCENT_BRAKE", "FL_LANDING_FLARE", "FL_TOUCHDOWN", "FL_RECOVERY",
    "ARM_MOTOR_INIT", "ARM_MOTOR_CAL", "ARM_READY"
]
EVENT_NAMES = {
    0: "STATE_CHANGE", 1: "FAULT", 2: "ABORT", 3: "PROFILE_LOADED",
    4: "CHECKS_GREEN", 5: "CHECKS_RED", 6: "BOOT_REPORT", 7: "ARMED",
    8: "DISARMED", 9: "LIFTOFF", 10: "APOGEE", 11: "LANDING",
    12: "GENERIC", 13: "PONG", 14: "BARO_CAL", 15: "MOTOR_ARMED"
}

# Detailed command documentation
COMMANDS_DOC = [
    ("P", "PING", "Send ping to FC, measures round-trip latency"),
    ("C", "CALIBRATE", "Calibrate barometer - sets current altitude as 0m AGL"),
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

# Profile details for help dialog
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
        "use_for": "Testing motor thrust curve and TVC servo response"
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
        "use_for": "Steady-state testing and thermal validation"
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
        "use_for": "Actual rocket flights with altitude targeting"
    },
}

# SVG Icons
SVG_ICONS = {
    "fullscreen": """<svg viewBox="0 0 24 24"><path fill="#00ff88" d="M7 14H5v5h5v-2H7v-3zm-2-4h2V7h3V5H5v5zm12 7h-3v2h5v-5h-2v3zM14 5v2h3v3h2V5h-5z"/></svg>""",
    "refresh": """<svg viewBox="0 0 24 24"><path fill="#00ff88" d="M17.65 6.35C16.2 4.9 14.21 4 12 4c-4.42 0-7.99 3.58-7.99 8s3.57 8 7.99 8c3.73 0 6.84-2.55 7.73-6h-2.08c-.82 2.33-3.04 4-5.65 4-3.31 0-6-2.69-6-6s2.69-6 6-6c1.66 0 3.14.69 4.22 1.78L13 11h7V4l-2.35 2.35z"/></svg>""",
    "clear": """<svg viewBox="0 0 24 24"><path fill="#ff6b6b" d="M19 6.41L17.59 5 12 10.59 6.41 5 5 6.41 10.59 12 5 17.59 6.41 19 12 13.41 17.59 19 19 17.59 13.41 12z"/></svg>""",
    "multiview": """<svg viewBox="0 0 24 24"><path fill="#00ff88" d="M3 5v14h18V5H3zm4 12H5V7h2v10zm6 0H9V7h4v10zm6 0h-4V7h4v10z"/></svg>""",
    "menu": """<svg viewBox="0 0 24 24"><path fill="#9ca3af" d="M4 6h16v2H4zM4 11h16v2H4zM4 16h16v2H4z"/></svg>""",
    "chevron": """<svg viewBox="0 0 24 24"><path fill="#9ca3af" d="M9.4 6.4L8 7.8l4.2 4.2L8 16.2l1.4 1.4L15 12z"/></svg>""",
    "time": """<svg viewBox="0 0 24 24"><path fill="#9ca3af" d="M12 2a10 10 0 100 20 10 10 0 000-20zm1 5h-2v6l5 3 1-1.7-4-2.3z"/></svg>""",
    "satellite": """<svg viewBox="0 0 24 24"><path fill="#9ca3af" d="M3 17l4-4 4 4-4 4-4-4zm13-9l5-5 3 3-5 5-3-3zM10 8l6 6-2 2-6-6 2-2z"/></svg>""",
    "status": """<svg viewBox="0 0 24 24"><path fill="#9ca3af" d="M12 2l4 4-4 4-4-4 4-4zm7 7h3v3h-3V9zM2 9h3v3H2V9zm10 5a5 5 0 110 10 5 5 0 010-10z"/></svg>""",
    "signal": """<svg viewBox="0 0 24 24"><path fill="#9ca3af" d="M3 21h2v-4H3v4zm4 0h2v-7H7v7zm4 0h2v-10h-2v10zm4 0h2v-13h-2v13zm4 0h2v-16h-2v16z"/></svg>""",
    "battery": """<svg viewBox="0 0 24 24"><path fill="#9ca3af" d="M16 7h1a2 2 0 012 2v6a2 2 0 01-2 2h-1v2h-2v-2H6a2 2 0 01-2-2V9a2 2 0 012-2h8V5h2v2z"/></svg>""",
    "dashboard": """<svg viewBox="0 0 24 24"><path fill="#9ca3af" d="M4 13h6v7H4v-7zm0-9h6v7H4V4zm10 0h6v11h-6V4zm0 13h6v3h-6v-3z"/></svg>""",
    "flight": """<svg viewBox="0 0 24 24"><path fill="#9ca3af" d="M12 2l3 7h7l-5.5 4 2 7-6.5-4-6.5 4 2-7L2 9h7z"/></svg>""",
    "location": """<svg viewBox="0 0 24 24"><path fill="#9ca3af" d="M12 2a7 7 0 00-7 7c0 5 7 13 7 13s7-8 7-13a7 7 0 00-7-7zm0 9.5A2.5 2.5 0 1112 6a2.5 2.5 0 010 5.5z"/></svg>""",
    "console": """<svg viewBox="0 0 24 24"><path fill="#9ca3af" d="M3 5h18v14H3V5zm4 4l4 3-4 3v-2l2-1-2-1V9zm6 6h5v-2h-5v2z"/></svg>""",
    "comms": """<svg viewBox="0 0 24 24"><path fill="#9ca3af" d="M12 3a9 9 0 019 9h-2a7 7 0 00-7-7V3zm0 4a5 5 0 015 5h-2a3 3 0 00-3-3V7zm0 4a1 1 0 011 1h-2a1 1 0 011-1zm0 10c-4.4 0-8-3.6-8-8H2c0 5.5 4.5 10 10 10v-2z"/></svg>"""
}

# ============================================================================
# Packet Parsing
# ============================================================================

def parse_fast_packet(data):
    if len(data) < 35:
        return None
    fmt = '<BBBBBIBBBB3h3h4hH'
    u = struct.unpack(fmt, data[:35])
    return {
        'type': 'fast',
        'frame_id': u[1], 'slot_id': u[2], 'seq': u[3], 'flags': u[4],
        'time_ms': u[5], 'state': u[6], 'substate': u[7],
        'ack_seq': u[8], 'ack_status': u[9],
        'accel_x': u[10] / 1000.0, 'accel_y': u[11] / 1000.0, 'accel_z': u[12] / 1000.0,
        'gyro_x': u[13] / 100.0, 'gyro_y': u[14] / 100.0, 'gyro_z': u[15] / 100.0,
        'altitude': u[16] / 10.0, 'vario': u[17] / 100.0,
        'pitch': u[18] / 10.0, 'roll': u[19] / 10.0,
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
        'payload': u[8],
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

# ============================================================================
# Serial Worker Thread
# ============================================================================

class SerialWorker(QThread):
    packet_received = pyqtSignal(dict)
    log_received = pyqtSignal(str)
    connection_changed = pyqtSignal(bool)
    error_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.serial = None
        self.running = False
        self.rx_buffer = bytearray()
        self.cmd_queue = Queue()

    def connect_port(self, port, baudrate=115200):
        try:
            self.serial = serial.Serial(port, baudrate, timeout=0.05)
            self.running = True
            self.connection_changed.emit(True)
            return True
        except Exception as e:
            self.error_signal.emit(f"Connection failed: {e}")
            return False

    def disconnect_port(self):
        self.running = False
        if self.serial:
            self.serial.close()
            self.serial = None
        self.connection_changed.emit(False)

    def send_command(self, cmd):
        self.cmd_queue.put(cmd)

    def run(self):
        while self.running:
            if not self.serial or not self.serial.is_open:
                time.sleep(0.1)
                continue

            try:
                while not self.cmd_queue.empty():
                    try:
                        cmd = self.cmd_queue.get_nowait()
                        self.serial.write(cmd.encode() if isinstance(cmd, str) else cmd)
                    except Empty:
                        break

                data = self.serial.read(256)
                if data:
                    self.rx_buffer.extend(data)
                    self.process_buffer()

            except Exception as e:
                self.error_signal.emit(f"Serial error: {e}")
                time.sleep(0.1)

    def process_buffer(self):
        while len(self.rx_buffer) >= 6:
            try:
                idx = 0
                while idx < len(self.rx_buffer) - 1:
                    if self.rx_buffer[idx] == FRAME_SYNC_1 and self.rx_buffer[idx+1] == FRAME_SYNC_2:
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

            except Exception:
                self.rx_buffer = self.rx_buffer[1:]

    def parse_message(self, msg_type, payload):
        try:
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
                self.log_received.emit(payload.decode('utf-8', errors='replace'))
                return

            if pkt:
                self.packet_received.emit(pkt)
        except Exception:
            pass

# ============================================================================
# Help Dialog (Detailed)
# ============================================================================

class HelpDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Command Reference & Profiles")
        self.setMinimumSize(800, 700)
        self.setStyleSheet("""
            QDialog { background-color: #1a1a2e; }
            QLabel { color: #ddd; font-size: 12px; }
            QTableWidget {
                background-color: #0d0d1a;
                color: #ddd;
                border: 1px solid #3d3d5c;
                border-radius: 8px;
                gridline-color: #2d2d44;
            }
            QTableWidget::item { padding: 8px; }
            QHeaderView::section {
                background-color: #2d2d44;
                color: #fff;
                padding: 8px;
                border: none;
                font-weight: bold;
            }
            QPushButton {
                background-color: #3498db;
                color: white;
                border: none;
                border-radius: 8px;
                padding: 10px 30px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #2980b9; }
            QTabWidget::pane { border: 1px solid #3d3d5c; background-color: #1a1a2e; }
            QTabBar::tab { background-color: #2d2d44; color: #888; padding: 10px 20px; border-radius: 5px 5px 0 0; }
            QTabBar::tab:selected { background-color: #3d3d5c; color: white; }
        """)

        layout = QVBoxLayout(self)
        layout.setSpacing(15)
        layout.setContentsMargins(20, 20, 20, 20)

        title = QLabel("Flight Computer Reference Guide")
        title.setStyleSheet("font-size: 20px; font-weight: bold; color: #00ff88;")
        layout.addWidget(title)

        tabs = QTabWidget()

        # Commands Tab
        cmd_widget = QWidget()
        cmd_layout = QVBoxLayout(cmd_widget)
        table = QTableWidget()
        table.setColumnCount(3)
        table.setHorizontalHeaderLabels(["Key", "Command", "Description"])
        table.setRowCount(len(COMMANDS_DOC))
        table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        table.verticalHeader().setVisible(False)
        table.setEditTriggers(QTableWidget.NoEditTriggers)
        table.setSelectionBehavior(QTableWidget.SelectRows)

        for row, (key, name, desc) in enumerate(COMMANDS_DOC):
            key_item = QTableWidgetItem(key)
            key_item.setTextAlignment(Qt.AlignCenter)
            key_item.setFont(QFont("Consolas", 12, QFont.Bold))
            key_item.setForeground(QColor("#ffe66d"))
            table.setItem(row, 0, key_item)

            name_item = QTableWidgetItem(name)
            name_item.setFont(QFont("Arial", 10, QFont.Bold))
            name_item.setForeground(QColor("#00ff88"))
            table.setItem(row, 1, name_item)

            table.setItem(row, 2, QTableWidgetItem(desc))

        cmd_layout.addWidget(table)
        tabs.addTab(cmd_widget, "Commands")

        # Profiles Tab
        profiles_widget = QWidget()
        profiles_layout = QVBoxLayout(profiles_widget)
        profiles_scroll = QScrollArea()
        profiles_scroll.setWidgetResizable(True)
        profiles_scroll.setStyleSheet("QScrollArea { border: none; background: transparent; }")

        profiles_content = QWidget()
        profiles_content_layout = QVBoxLayout(profiles_content)

        for profile_id, details in PROFILE_DETAILS.items():
            card = QFrame()
            card.setStyleSheet("""
                QFrame {
                    background-color: #2d2d44;
                    border: 2px solid #3d3d5c;
                    border-radius: 12px;
                    padding: 15px;
                }
            """)
            card_layout = QVBoxLayout(card)

            header = QLabel(f"Profile {profile_id}: {details['name']}")
            header.setStyleSheet("color: #00ff88; font-size: 16px; font-weight: bold;")
            card_layout.addWidget(header)

            mode_label = QLabel(f"Mode: {details['mode']}")
            mode_label.setStyleSheet("color: #ffe66d; font-size: 12px;")
            card_layout.addWidget(mode_label)

            desc_label = QLabel(details['description'])
            desc_label.setStyleSheet("color: #aaa; font-size: 11px;")
            desc_label.setWordWrap(True)
            card_layout.addWidget(desc_label)

            params_frame = QFrame()
            params_layout = QGridLayout(params_frame)
            for i, (param_name, param_value) in enumerate(details['params']):
                name_lbl = QLabel(param_name + ":")
                name_lbl.setStyleSheet("color: #888;")
                value_lbl = QLabel(param_value)
                value_lbl.setStyleSheet("color: #4ecdc4; font-weight: bold;")
                params_layout.addWidget(name_lbl, i, 0)
                params_layout.addWidget(value_lbl, i, 1)
            card_layout.addWidget(params_frame)

            use_label = QLabel(f"Use for: {details['use_for']}")
            use_label.setStyleSheet("color: #a855f7; font-size: 11px; font-style: italic;")
            use_label.setWordWrap(True)
            card_layout.addWidget(use_label)

            profiles_content_layout.addWidget(card)

        profiles_content_layout.addStretch()
        profiles_scroll.setWidget(profiles_content)
        profiles_layout.addWidget(profiles_scroll)
        tabs.addTab(profiles_widget, "Profiles")

        # State Machine Tab
        states_widget = QWidget()
        states_layout = QVBoxLayout(states_widget)
        states_label = QLabel("""
        <h3 style="color: #00ff88;">State Machine Flow</h3>
        <p style="color: #aaa;">
        <b style="color: #666;">BOOT</b> &rarr;
        <b style="color: #3498db;">IDLE</b> &rarr;
        <b style="color: #9b59b6;">CONFIGED</b> &rarr;
        <b style="color: #e67e22;">ARMED</b> &rarr;
        <b style="color: #1abc9c;">TEST_STAND</b> / <b style="color: #2ecc71;">FLIGHT</b> &rarr;
        <b style="color: #7f8c8d;">SAFE</b>
        </p>
        <p style="color: #e74c3c;"><b>ABORT</b> can be triggered from any state</p>
        <br>
        <h4 style="color: #ffe66d;">Transitions:</h4>
        <ul style="color: #aaa;">
        <li><b>BOOT &rarr; IDLE:</b> Automatic after initialization</li>
        <li><b>IDLE &rarr; CONFIGED:</b> Load a profile (1, 2, or 3)</li>
        <li><b>CONFIGED &rarr; ARMED:</b> ARM command</li>
        <li><b>ARMED &rarr; TEST_STAND:</b> TEST command (profiles 1-2)</li>
        <li><b>ARMED &rarr; FLIGHT:</b> LAUNCH command (profile 3)</li>
        <li><b>Any &rarr; SAFE:</b> SAFE command or mission complete</li>
        <li><b>Any &rarr; ABORT:</b> ABORT command or fault detected</li>
        </ul>
        """)
        states_label.setTextFormat(Qt.RichText)
        states_layout.addWidget(states_label)
        states_layout.addStretch()
        tabs.addTab(states_widget, "State Machine")

        layout.addWidget(tabs)

        close_btn = QPushButton("Close")
        close_btn.clicked.connect(self.accept)
        layout.addWidget(close_btn, alignment=Qt.AlignCenter)

# ============================================================================
# Boot Status Widget
# ============================================================================

class BootStatusWidget(QFrame):
    """Shows sensor initialization status from boot report"""
    def __init__(self):
        super().__init__()
        self.setStyleSheet("""
            QFrame {
                background-color: #111827;
                border: 1px solid #243141;
                border-radius: 8px;
            }
        """)

        layout = QGridLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(4)

        title = QLabel("SENSORS")
        title.setStyleSheet("color: #9ca3af; font-size: 10px; font-weight: bold;")
        layout.addWidget(title, 0, 0, 1, 5)

        self.indicators = {}
        sensors = ["IMU", "MAG", "BARO", "BNO", "GPS"]

        for i, sensor in enumerate(sensors):
            indicator = QLabel("?")
            indicator.setAlignment(Qt.AlignCenter)
            indicator.setFixedSize(24, 24)
            indicator.setStyleSheet("""
                background-color: #1f2a37;
                color: #9ca3af;
                border-radius: 12px;
                font-weight: bold;
                font-size: 10px;
            """)
            self.indicators[sensor] = indicator

            name_label = QLabel(sensor)
            name_label.setStyleSheet("color: #6b7280; font-size: 9px;")
            name_label.setAlignment(Qt.AlignCenter)

            col_widget = QWidget()
            col_layout = QVBoxLayout(col_widget)
            col_layout.setContentsMargins(2, 0, 2, 0)
            col_layout.setSpacing(2)
            col_layout.addWidget(indicator, alignment=Qt.AlignCenter)
            col_layout.addWidget(name_label, alignment=Qt.AlignCenter)

            layout.addWidget(col_widget, 1, i)

    def update_status(self, sensor, status):
        """Update sensor status: -1=unknown, 0=failed, 1=ok"""
        if sensor not in self.indicators:
            return

        indicator = self.indicators[sensor]
        if status == 1:
            indicator.setText("OK")
            indicator.setStyleSheet("""
                background-color: #22c55e;
                color: #0b1117;
                border-radius: 12px;
                font-weight: bold;
                font-size: 9px;
            """)
        elif status == 0:
            indicator.setText("X")
            indicator.setStyleSheet("""
                background-color: #ef4444;
                color: #fff;
                border-radius: 12px;
                font-weight: bold;
                font-size: 10px;
            """)
        else:
            indicator.setText("?")
            indicator.setStyleSheet("""
                background-color: #1f2a37;
                color: #9ca3af;
                border-radius: 12px;
                font-weight: bold;
                font-size: 10px;
            """)

    def parse_boot_report(self, payload):
        """Parse boot report event payload"""
        if len(payload) >= 5:
            sensors = ["IMU", "MAG", "BARO", "BNO", "GPS"]
            for i, sensor in enumerate(sensors):
                if i < len(payload):
                    status = payload[i]
                    if isinstance(status, int):
                        self.update_status(sensor, 1 if status > 0 else 0)

# ============================================================================
# Attitude Indicator (Artificial Horizon)
# ============================================================================

class AttitudeIndicator(QWidget):
    """Artificial horizon / attitude indicator"""
    def __init__(self):
        super().__init__()
        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        self.setMinimumSize(200, 200)

    def set_attitude(self, pitch, roll, yaw=0):
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Get center and radius
        w, h = self.width(), self.height()
        size = min(w, h)
        cx, cy = w // 2, h // 2
        r = size // 2 - 10

        # Save state and translate to center
        painter.save()
        painter.translate(cx, cy)

        # Rotate for roll
        painter.rotate(-self.roll)

        # Calculate pitch offset (pixels per degree)
        pitch_scale = r / 30.0  # 30 degrees = full radius
        pitch_offset = self.pitch * pitch_scale

        # Clip to circle
        painter.setClipRegion(pg.QtGui.QRegion(
            int(-r), int(-r), int(2*r), int(2*r), pg.QtGui.QRegion.Ellipse
        ))

        # Draw sky (blue)
        sky_rect = QRectF(-r*2, -r*2 + pitch_offset, r*4, r*2)
        sky_gradient = QLinearGradient(0, -r*2 + pitch_offset, 0, pitch_offset)
        sky_gradient.setColorAt(0, QColor("#1a3a5c"))
        sky_gradient.setColorAt(1, QColor("#3498db"))
        painter.fillRect(sky_rect, QBrush(sky_gradient))

        # Draw ground (brown)
        ground_rect = QRectF(-r*2, pitch_offset, r*4, r*2)
        ground_gradient = QLinearGradient(0, pitch_offset, 0, r*2 + pitch_offset)
        ground_gradient.setColorAt(0, QColor("#8B4513"))
        ground_gradient.setColorAt(1, QColor("#5D3612"))
        painter.fillRect(ground_rect, QBrush(ground_gradient))

        # Draw horizon line
        painter.setPen(QPen(QColor("#fff"), 2))
        painter.drawLine(int(-r*2), int(pitch_offset), int(r*2), int(pitch_offset))

        # Draw pitch ladder
        painter.setPen(QPen(QColor("#fff"), 1))
        for deg in range(-30, 31, 10):
            if deg == 0:
                continue
            y = pitch_offset - deg * pitch_scale
            line_width = 30 if abs(deg) == 10 else 20
            painter.drawLine(int(-line_width), int(y), int(line_width), int(y))
            painter.drawText(int(line_width + 5), int(y + 4), str(abs(deg)))

        painter.restore()

        # Draw fixed aircraft symbol
        painter.setPen(QPen(QColor("#ffe66d"), 3))
        painter.drawLine(cx - 40, cy, cx - 15, cy)
        painter.drawLine(cx + 15, cy, cx + 40, cy)
        painter.drawLine(cx, cy - 5, cx, cy + 10)
        painter.drawEllipse(cx - 5, cy - 5, 10, 10)

        # Draw outer ring
        painter.setPen(QPen(QColor("#3d3d5c"), 3))
        painter.drawEllipse(cx - r, cy - r, 2*r, 2*r)

        # Draw roll indicator marks
        painter.setPen(QPen(QColor("#888"), 2))
        for angle in [0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330]:
            rad = math.radians(angle - 90)
            inner = r - 8
            outer = r - 2
            x1 = cx + inner * math.cos(rad)
            y1 = cy + inner * math.sin(rad)
            x2 = cx + outer * math.cos(rad)
            y2 = cy + outer * math.sin(rad)
            painter.drawLine(int(x1), int(y1), int(x2), int(y2))

        # Draw roll pointer
        painter.setPen(QPen(QColor("#ff6b6b"), 2))
        roll_rad = math.radians(-self.roll - 90)
        px = cx + (r - 15) * math.cos(roll_rad)
        py = cy + (r - 15) * math.sin(roll_rad)
        triangle = QPolygonF([
            QPointF(px, py),
            QPointF(px + 8 * math.cos(roll_rad + 2.5), py + 8 * math.sin(roll_rad + 2.5)),
            QPointF(px + 8 * math.cos(roll_rad - 2.5), py + 8 * math.sin(roll_rad - 2.5))
        ])
        painter.setBrush(QBrush(QColor("#ff6b6b")))
        painter.drawPolygon(triangle)

        # Draw attitude values
        painter.setPen(QColor("#888"))
        painter.setFont(QFont("Consolas", 9))
        painter.drawText(10, h - 10, f"P:{self.pitch:+.1f}  R:{self.roll:+.1f}  Y:{self.yaw:.1f}")

# ============================================================================
# PWM Gauge Widget
# ============================================================================

class PWMGauge(QWidget):
    """Dial-style gauge for motor power 0-100%"""
    def __init__(self):
        super().__init__()
        self.value = 0
        self.setMinimumSize(150, 150)

    def set_value(self, value):
        self.value = max(0, min(100, value))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()
        size = min(w, h)
        cx, cy = w // 2, h // 2 + 10
        r = size // 2 - 20

        # Draw background arc
        painter.setPen(QPen(QColor("#2d2d44"), 12, Qt.SolidLine, Qt.RoundCap))
        start_angle = 225 * 16
        span_angle = -270 * 16
        painter.drawArc(cx - r, cy - r, 2*r, 2*r, start_angle, span_angle)

        # Draw value arc with gradient color
        if self.value > 0:
            # Color based on value: green -> yellow -> red
            if self.value < 30:
                color = QColor("#00ff88")
            elif self.value < 70:
                color = QColor("#ffe66d")
            else:
                color = QColor("#ff6b6b")

            painter.setPen(QPen(color, 10, Qt.SolidLine, Qt.RoundCap))
            value_span = int(-270 * 16 * self.value / 100)
            painter.drawArc(cx - r, cy - r, 2*r, 2*r, start_angle, value_span)

        # Draw needle
        angle = math.radians(225 - 270 * self.value / 100)
        needle_len = r - 15
        nx = cx + needle_len * math.cos(angle)
        ny = cy - needle_len * math.sin(angle)
        painter.setPen(QPen(QColor("#fff"), 3))
        painter.drawLine(cx, cy, int(nx), int(ny))

        # Draw center circle
        painter.setBrush(QBrush(QColor("#1a1a2e")))
        painter.setPen(QPen(QColor("#4d4d6d"), 2))
        painter.drawEllipse(cx - 8, cy - 8, 16, 16)

        # Draw value text
        painter.setPen(QColor("#00ff88"))
        painter.setFont(QFont("Consolas", 18, QFont.Bold))
        text = f"{int(self.value)}%"
        text_rect = painter.fontMetrics().boundingRect(text)
        painter.drawText(cx - text_rect.width() // 2, cy + r // 2, text)

        # Draw label
        painter.setPen(QColor("#888"))
        painter.setFont(QFont("Arial", 10))
        painter.drawText(cx - 20, cy + r // 2 + 20, "PWM")

        # Draw scale marks
        painter.setPen(QPen(QColor("#666"), 1))
        for pct in [0, 25, 50, 75, 100]:
            angle = math.radians(225 - 270 * pct / 100)
            inner = r + 5
            outer = r + 12
            x1 = cx + inner * math.cos(angle)
            y1 = cy - inner * math.sin(angle)
            x2 = cx + outer * math.cos(angle)
            y2 = cy - outer * math.sin(angle)
            painter.drawLine(int(x1), int(y1), int(x2), int(y2))

# ============================================================================
# GPS Map Widget
# ============================================================================

class GPSMapWidget(QWidget):
    """GPS Map using OpenStreetMap via Leaflet.js"""
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.lat = 0.0
        self.lon = 0.0
        self.last_update = 0
        self.has_fix = False

        if HAS_WEBENGINE:
            self.web = QWebEngineView()
            self.web.setMinimumHeight(300)
            layout.addWidget(self.web)
            self.load_map()
        else:
            no_map = QLabel("GPS Map requires PyQtWebEngine\npip install PyQtWebEngine")
            no_map.setStyleSheet("color: #888; font-size: 12px;")
            no_map.setAlignment(Qt.AlignCenter)
            layout.addWidget(no_map)
            self.web = None

    def load_map(self):
        if not self.web:
            return

        html = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <style>
        body { margin: 0; padding: 0; background: #151a22; }
        #map { width: 100%; height: 100vh; }
        .no-fix-overlay {
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            background: rgba(21, 26, 34, 0.9);
            color: #ef4444;
            padding: 20px 40px;
            border-radius: 10px;
            font-family: Arial, sans-serif;
            font-size: 16px;
            z-index: 1000;
            display: block;
        }
    </style>
</head>
<body>
    <div id="map"></div>
    <div id="nofix" class="no-fix-overlay">Waiting for GPS Fix...</div>
    <script>
        var map = L.map('map', {
            center: [38.7223, -9.1393],
            zoom: 15,
            zoomControl: true
        });

        L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 19,
            attribution: '&copy; OpenStreetMap'
        }).addTo(map);

        var rocketIcon = L.divIcon({
            className: 'rocket-marker',
            html: '<div style="width:20px;height:20px;background:#00ff88;border:3px solid #fff;border-radius:50%;box-shadow:0 0 10px #00ff88;"></div>',
            iconSize: [20, 20],
            iconAnchor: [10, 10]
        });

        var marker = L.marker([38.7223, -9.1393], {icon: rocketIcon}).addTo(map);
        var path = [];
        var pathLine = L.polyline(path, {color: '#00ff88', weight: 2, opacity: 0.7}).addTo(map);

        function updatePosition(lat, lon, hasFix) {
            document.getElementById('nofix').style.display = hasFix ? 'none' : 'block';
            if (hasFix && lat != 0 && lon != 0) {
                marker.setLatLng([lat, lon]);
                map.setView([lat, lon]);
                path.push([lat, lon]);
                if (path.length > 500) path.shift();
                pathLine.setLatLngs(path);
            }
        }

        function clearPath() {
            path = [];
            pathLine.setLatLngs(path);
        }
    </script>
</body>
</html>
"""
        self.web.setHtml(html)

    def update_position(self, lat, lon, has_fix=False):
        self.lat = lat
        self.lon = lon
        self.has_fix = has_fix
        self.last_update = time.time()

        if self.web:
            js = f"if (typeof updatePosition === 'function') {{ updatePosition({lat}, {lon}, {'true' if has_fix else 'false'}); }}"
            self.web.page().runJavaScript(js)

    def clear_path(self):
        if self.web:
            self.web.page().runJavaScript("clearPath()")

# ============================================================================
# GPS Widget (Enhanced with persistence)
# ============================================================================

class GPSWidget(QFrame):
    def __init__(self):
        super().__init__()
        self.setStyleSheet("QFrame { background-color: #151a22; border: 1px solid #273241; border-radius: 8px; }")

        self.last_update = 0
        self.last_lat = 0
        self.last_lon = 0

        layout = QGridLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        # Row 0: Title and stale indicator
        title = QLabel("GPS NAVIGATION")
        title.setStyleSheet("color: #9ca3af; font-size: 10px; font-weight: bold;")
        layout.addWidget(title, 0, 0, 1, 3)

        self.stale_indicator = QLabel("STALE")
        self.stale_indicator.setStyleSheet("color: #ef4444; font-size: 9px; font-weight: bold;")
        self.stale_indicator.hide()
        layout.addWidget(self.stale_indicator, 0, 3)

        # Row 1: Lat/Lon
        layout.addWidget(QLabel("LAT"), 1, 0)
        self.lat_value = QLabel("---.------")
        self.lat_value.setStyleSheet("color: #22c55e; font-size: 14px; font-weight: bold; font-family: Consolas;")
        layout.addWidget(self.lat_value, 1, 1)

        layout.addWidget(QLabel("LON"), 1, 2)
        self.lon_value = QLabel("---.------")
        self.lon_value.setStyleSheet("color: #22c55e; font-size: 14px; font-weight: bold; font-family: Consolas;")
        layout.addWidget(self.lon_value, 1, 3)

        # Row 2: Alt, Sats, Lock
        layout.addWidget(QLabel("ALT"), 2, 0)
        self.alt_value = QLabel("---")
        self.alt_value.setStyleSheet("color: #4ecdc4; font-size: 14px; font-weight: bold;")
        layout.addWidget(self.alt_value, 2, 1)

        layout.addWidget(QLabel("SATS"), 2, 2)
        self.sats_value = QLabel("---")
        self.sats_value.setStyleSheet("color: #ffe66d; font-size: 14px; font-weight: bold;")
        layout.addWidget(self.sats_value, 2, 3)

        # Row 3: Lock status
        layout.addWidget(QLabel("LOCK"), 3, 0)
        self.lock_value = QLabel("NO FIX")
        self.lock_value.setStyleSheet("color: #ff6b6b; font-size: 14px; font-weight: bold;")
        layout.addWidget(self.lock_value, 3, 1, 1, 3)

        # Style labels
        for i in range(layout.count()):
            w = layout.itemAt(i).widget()
            if isinstance(w, QLabel) and w.text() in ["LAT", "LON", "ALT", "SATS", "LOCK"]:
                w.setStyleSheet("color: #666; font-size: 10px;")

        # Timer to check for stale data
        self.stale_timer = QTimer(self)
        self.stale_timer.timeout.connect(self.check_stale)
        self.stale_timer.start(1000)

    def check_stale(self):
        if self.last_update > 0 and time.time() - self.last_update > 3.0:
            self.stale_indicator.show()
        else:
            self.stale_indicator.hide()

    def update_data(self, lat, lon, alt, sats, lock):
        self.last_update = time.time()

        # Only update if valid (non-zero) coordinates received
        if lat != 0 or lon != 0:
            self.last_lat = lat
            self.last_lon = lon
            self.lat_value.setText(f"{lat:.6f}")
            self.lon_value.setText(f"{lon:.6f}")
        elif self.last_lat != 0 or self.last_lon != 0:
            # Keep showing last known position
            self.lat_value.setText(f"{self.last_lat:.6f}")
            self.lon_value.setText(f"{self.last_lon:.6f}")
        else:
            self.lat_value.setText("---.------")
            self.lon_value.setText("---.------")

        self.alt_value.setText(f"{alt:.1f} m" if alt else "---")

        self.sats_value.setText(str(sats) if sats is not None else "---")
        if sats and sats >= 6:
            self.sats_value.setStyleSheet("color: #00ff88; font-size: 14px; font-weight: bold;")
        elif sats and sats >= 4:
            self.sats_value.setStyleSheet("color: #ffe66d; font-size: 14px; font-weight: bold;")
        else:
            self.sats_value.setStyleSheet("color: #ff6b6b; font-size: 14px; font-weight: bold;")

        if lock and lock >= 3:
            self.lock_value.setText("3D FIX")
            self.lock_value.setStyleSheet("color: #00ff88; font-size: 14px; font-weight: bold;")
        elif lock and lock >= 2:
            self.lock_value.setText("2D FIX")
            self.lock_value.setStyleSheet("color: #ffe66d; font-size: 14px; font-weight: bold;")
        else:
            self.lock_value.setText("NO FIX")
            self.lock_value.setStyleSheet("color: #ff6b6b; font-size: 14px; font-weight: bold;")

# ============================================================================
# Fullscreen Windows
# ============================================================================

class FullscreenPlotWindow(QDialog):
    def __init__(self, original_plot, title, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"{title} - Fullscreen")
        self.setMinimumSize(1000, 700)
        self.setStyleSheet("background-color: #0d0d1a;")
        self.original_plot = original_plot

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)

        self.plot = pg.PlotWidget()
        self.plot.setTitle(title, color='w', size='14pt')
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setBackground('#0d0d1a')
        layout.addWidget(self.plot)

        btn_layout = QHBoxLayout()
        auto_btn = QPushButton("[R] Auto Range")
        auto_btn.setStyleSheet("background-color: #3498db; color: white; border-radius: 8px; padding: 8px 15px;")
        auto_btn.clicked.connect(lambda: self.plot.enableAutoRange())
        btn_layout.addWidget(auto_btn)
        btn_layout.addStretch()
        close_btn = QPushButton("[X] Close")
        close_btn.setStyleSheet("background-color: #e74c3c; color: white; border-radius: 8px; padding: 8px 15px;")
        close_btn.clicked.connect(self.close)
        btn_layout.addWidget(close_btn)
        layout.addLayout(btn_layout)

        colors = ['#00ff88', '#ff6b6b', '#4ecdc4', '#ffe66d', '#a855f7', '#06b6d4']
        self.curves = []
        legend_names = getattr(self.original_plot, 'legend_names', None)

        if legend_names and self.original_plot.num_lines == 3:
            self.plot.addLegend()
            for i in range(self.original_plot.num_lines):
                self.curves.append(self.plot.plot(
                    pen=pg.mkPen(colors[i % len(colors)], width=2),
                    name=legend_names[i]
                ))
        else:
            for i in range(self.original_plot.num_lines):
                self.curves.append(self.plot.plot(pen=pg.mkPen(colors[i % len(colors)], width=2)))

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(50)

    def update_plot(self):
        if hasattr(self.original_plot, 'data') and hasattr(self.original_plot, 'times'):
            times = list(self.original_plot.times)
            for i, curve in enumerate(self.curves):
                if i < len(self.original_plot.data) and len(self.original_plot.data[i]) > 0:
                    data = list(self.original_plot.data[i])
                    if len(times) >= len(data):
                        curve.setData(times[:len(data)], data)

    def closeEvent(self, event):
        self.timer.stop()
        event.accept()

class Fullscreen3DWindow(QDialog):
    def __init__(self, parent_view, parent=None):
        super().__init__(parent)
        self.setWindowFlags(Qt.Window)
        self.setWindowTitle("3D Attitude View - Fullscreen")
        self.resize(900, 700)
        self.setStyleSheet("background-color: #0d0d1a;")
        self.parent_view = parent_view

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        self.rocket_view = RocketView3D()
        self.rocket_view.setCameraPosition(distance=20, elevation=25, azimuth=45)
        layout.addWidget(self.rocket_view, stretch=1)

        info_panel = QFrame()
        info_panel.setStyleSheet("""
            QFrame {
                background-color: #1a1a2e;
                border: 2px solid #3d3d5c;
                border-radius: 12px;
            }
        """)
        info_panel.setFixedHeight(80)

        info_layout = QHBoxLayout(info_panel)
        info_layout.setContentsMargins(20, 10, 20, 10)

        lbl_style = "font-size: 24px; font-weight: bold; font-family: 'Consolas', monospace;"

        self.pitch_lbl = QLabel("PITCH: 0.0")
        self.pitch_lbl.setStyleSheet(f"color: #a855f7; {lbl_style}")

        self.roll_lbl = QLabel("ROLL: 0.0")
        self.roll_lbl.setStyleSheet(f"color: #06b6d4; {lbl_style}")

        self.yaw_lbl = QLabel("YAW: 0.0")
        self.yaw_lbl.setStyleSheet(f"color: #ffe66d; {lbl_style}")

        info_layout.addWidget(self.pitch_lbl)
        info_layout.addStretch()
        info_layout.addWidget(self.roll_lbl)
        info_layout.addStretch()
        info_layout.addWidget(self.yaw_lbl)

        layout.addWidget(info_panel)

        btn_layout = QHBoxLayout()
        btn_layout.addStretch()
        close_btn = QPushButton("Close")
        close_btn.setStyleSheet("""
            QPushButton {
                background-color: #e74c3c;
                color: white;
                border: none;
                border-radius: 10px;
                padding: 12px 30px;
                font-size: 16px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #c0392b; }
        """)
        close_btn.clicked.connect(self.close)
        btn_layout.addWidget(close_btn)
        layout.addLayout(btn_layout)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.sync_view)
        self.timer.start(50)

    def sync_view(self):
        try:
            p = self.parent_view.pitch
            r = self.parent_view.roll
            y = self.parent_view.yaw

            self.pitch_lbl.setText(f"PITCH: {p:>7.1f}")
            self.roll_lbl.setText(f"ROLL: {r:>7.1f}")
            self.yaw_lbl.setText(f"YAW: {y:>7.1f}")

            self.rocket_view.update_attitude(p, r, y)
        except Exception:
            pass

    def closeEvent(self, event):
        self.timer.stop()
        super().closeEvent(event)

class FullscreenConsoleWindow(QDialog):
    def __init__(self, log_entries, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Console - Fullscreen")
        self.setMinimumSize(900, 600)
        self.setStyleSheet("""
            QDialog { background-color: #0d0d1a; }
            QPlainTextEdit {
                background-color: #0a0a12;
                color: #aaa;
                border: 1px solid #2d2d44;
                border-radius: 8px;
                font-family: monospace;
                font-size: 12px;
                padding: 10px;
            }
        """)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(15, 15, 15, 15)

        self.text = QPlainTextEdit()
        self.text.setReadOnly(True)
        self.text.setPlainText('\n'.join(log_entries))
        self.text.moveCursor(QTextCursor.End)
        layout.addWidget(self.text)

        btn_layout = QHBoxLayout()
        clear_btn = QPushButton("[C] Clear")
        clear_btn.setStyleSheet("background-color: #6c757d; color: white; border-radius: 8px; padding: 8px 15px;")
        clear_btn.clicked.connect(self.text.clear)
        btn_layout.addWidget(clear_btn)
        btn_layout.addStretch()
        close_btn = QPushButton("[X] Close")
        close_btn.setStyleSheet("background-color: #e74c3c; color: white; border-radius: 8px; padding: 8px 15px;")
        close_btn.clicked.connect(self.close)
        btn_layout.addWidget(close_btn)
        layout.addLayout(btn_layout)

# ============================================================================
# Multi-View Window (Tabbed Fullscreen)
# ============================================================================

class MultiViewWindow(QMainWindow):
    """Tabbed window with multiple fullscreen views"""
    def __init__(self, dashboard, parent=None):
        super().__init__(parent)
        self.dashboard = dashboard
        self.setWindowTitle("Multi-View Dashboard")
        self.resize(1200, 800)
        self.setStyleSheet("""
            QMainWindow { background-color: #0d0d1a; }
            QTabWidget::pane { border: 1px solid #3d3d5c; background-color: #0d0d1a; }
            QTabBar::tab { background-color: #1e1e2e; color: #888; padding: 10px 25px; margin-right: 2px; }
            QTabBar::tab:selected { background-color: #2d2d44; color: white; }
        """)

        tabs = QTabWidget()

        # 3D Attitude Tab
        att_widget = QWidget()
        att_layout = QVBoxLayout(att_widget)
        self.mv_rocket = RocketView3D()
        self.mv_rocket.setCameraPosition(distance=18, elevation=25, azimuth=45)
        att_layout.addWidget(self.mv_rocket)

        info_panel = QWidget()
        info_layout = QHBoxLayout(info_panel)
        lbl_style = "font-size: 20px; font-weight: bold; font-family: 'Consolas';"
        self.mv_pitch = QLabel("PITCH: 0.0")
        self.mv_pitch.setStyleSheet(f"color: #a855f7; {lbl_style}")
        self.mv_roll = QLabel("ROLL: 0.0")
        self.mv_roll.setStyleSheet(f"color: #06b6d4; {lbl_style}")
        self.mv_yaw = QLabel("YAW: 0.0")
        self.mv_yaw.setStyleSheet(f"color: #ffe66d; {lbl_style}")
        info_layout.addWidget(self.mv_pitch)
        info_layout.addStretch()
        info_layout.addWidget(self.mv_roll)
        info_layout.addStretch()
        info_layout.addWidget(self.mv_yaw)
        att_layout.addWidget(info_panel)
        tabs.addTab(att_widget, "3D Attitude")

        # Altitude Tab
        alt_widget = QWidget()
        alt_layout = QVBoxLayout(alt_widget)
        self.mv_alt_plot = pg.PlotWidget()
        self.mv_alt_plot.setTitle("Altitude", color='w', size='14pt')
        self.mv_alt_plot.showGrid(x=True, y=True, alpha=0.3)
        self.mv_alt_plot.setBackground('#0d0d1a')
        self.mv_alt_curve = self.mv_alt_plot.plot(pen=pg.mkPen('#00ff88', width=2))
        alt_layout.addWidget(self.mv_alt_plot)
        tabs.addTab(alt_widget, "Altitude")

        # IMU Tab
        imu_widget = QWidget()
        imu_layout = QVBoxLayout(imu_widget)

        self.mv_accel_plot = pg.PlotWidget()
        self.mv_accel_plot.setTitle("Acceleration", color='w', size='12pt')
        self.mv_accel_plot.showGrid(x=True, y=True, alpha=0.3)
        self.mv_accel_plot.setBackground('#0d0d1a')
        self.mv_accel_plot.addLegend()
        self.mv_accel_curves = [
            self.mv_accel_plot.plot(pen=pg.mkPen('#ff6b6b', width=2), name='X'),
            self.mv_accel_plot.plot(pen=pg.mkPen('#4ecdc4', width=2), name='Y'),
            self.mv_accel_plot.plot(pen=pg.mkPen('#ffe66d', width=2), name='Z'),
        ]
        imu_layout.addWidget(self.mv_accel_plot)

        self.mv_gyro_plot = pg.PlotWidget()
        self.mv_gyro_plot.setTitle("Gyroscope", color='w', size='12pt')
        self.mv_gyro_plot.showGrid(x=True, y=True, alpha=0.3)
        self.mv_gyro_plot.setBackground('#0d0d1a')
        self.mv_gyro_plot.addLegend()
        self.mv_gyro_curves = [
            self.mv_gyro_plot.plot(pen=pg.mkPen('#ff6b6b', width=2), name='X'),
            self.mv_gyro_plot.plot(pen=pg.mkPen('#4ecdc4', width=2), name='Y'),
            self.mv_gyro_plot.plot(pen=pg.mkPen('#ffe66d', width=2), name='Z'),
        ]
        imu_layout.addWidget(self.mv_gyro_plot)
        tabs.addTab(imu_widget, "IMU")

        # GPS Map Tab
        if HAS_WEBENGINE:
            self.mv_gps_map = GPSMapWidget()
            tabs.addTab(self.mv_gps_map, "GPS Map")
        else:
            no_gps = QLabel("GPS Map requires PyQtWebEngine")
            no_gps.setStyleSheet("color: #888;")
            no_gps.setAlignment(Qt.AlignCenter)
            tabs.addTab(no_gps, "GPS Map")

        self.setCentralWidget(tabs)

        # Update timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.sync_data)
        self.timer.start(50)

    def sync_data(self):
        try:
            # Sync 3D view
            p = self.dashboard.rocket_view.pitch
            r = self.dashboard.rocket_view.roll
            y = self.dashboard.rocket_view.yaw
            self.mv_rocket.update_attitude(p, r, y)
            self.mv_pitch.setText(f"PITCH: {p:>7.1f}")
            self.mv_roll.setText(f"ROLL: {r:>7.1f}")
            self.mv_yaw.setText(f"YAW: {y:>7.1f}")

            # Sync altitude plot
            if hasattr(self.dashboard, 'alt_plot'):
                times = list(self.dashboard.alt_plot.times)
                if len(self.dashboard.alt_plot.data[0]) > 0:
                    data = list(self.dashboard.alt_plot.data[0])
                    self.mv_alt_curve.setData(times[:len(data)], data)

            # Sync IMU plots
            if hasattr(self.dashboard, 'accel_plot'):
                times = list(self.dashboard.accel_plot.times)
                for i, curve in enumerate(self.mv_accel_curves):
                    if i < len(self.dashboard.accel_plot.data) and len(self.dashboard.accel_plot.data[i]) > 0:
                        data = list(self.dashboard.accel_plot.data[i])
                        curve.setData(times[:len(data)], data)

            if hasattr(self.dashboard, 'gyro_plot'):
                times = list(self.dashboard.gyro_plot.times)
                for i, curve in enumerate(self.mv_gyro_curves):
                    if i < len(self.dashboard.gyro_plot.data) and len(self.dashboard.gyro_plot.data[i]) > 0:
                        data = list(self.dashboard.gyro_plot.data[i])
                        curve.setData(times[:len(data)], data)

            # Sync GPS map
            if HAS_WEBENGINE and hasattr(self, 'mv_gps_map') and hasattr(self.dashboard, 'slow_data'):
                sd = self.dashboard.slow_data
                if sd:
                    lat = sd.get('latitude', 0)
                    lon = sd.get('longitude', 0)
                    lock = sd.get('gps_lock', 0)
                    self.mv_gps_map.update_position(lat, lon, lock >= 2)

        except Exception:
            pass

    def closeEvent(self, event):
        self.timer.stop()
        super().closeEvent(event)

# ============================================================================
# 3D Rocket Visualization
# ============================================================================

class RocketView3D(gl.GLViewWidget):
    def __init__(self):
        super().__init__()
        self.setCameraPosition(distance=15, elevation=20, azimuth=45)
        self.setBackgroundColor('#1a1a2e')

        grid = gl.GLGridItem()
        grid.setSize(20, 20, 1)
        grid.setSpacing(2, 2, 2)
        grid.setColor((100, 100, 100, 100))
        self.addItem(grid)

        self.rocket_mesh = self.create_rocket()
        self.addItem(self.rocket_mesh)
        self.add_axes()

        self.pitch = 0
        self.roll = 0
        self.yaw = 0

    def create_rocket(self):
        n, r, h = 16, 0.3, 4.0
        verts, faces, colors = [], [], []

        for i in range(n):
            angle = 2 * math.pi * i / n
            x, y = r * math.cos(angle), r * math.sin(angle)
            verts.extend([[x, y, 0], [x, y, h * 0.7]])

        for i in range(n):
            angle = 2 * math.pi * i / n
            verts.append([r * 0.5 * math.cos(angle), r * 0.5 * math.sin(angle), h * 0.85])
        verts.append([0, 0, h])

        for i in range(n):
            i2 = (i + 1) % n
            faces.extend([[i*2, i*2+1, i2*2+1], [i*2, i2*2+1, i2*2]])
            colors.extend([[0.8, 0.2, 0.2, 1], [0.8, 0.2, 0.2, 1]])

        for i in range(n):
            faces.append([i*2, (i+1)%n*2, n*2])
            colors.append([0.3, 0.3, 0.3, 1])
        verts.append([0, 0, 0])

        nose_start, tip_idx = n * 2, len(verts) - 2
        for i in range(n):
            i2 = (i + 1) % n
            faces.extend([[i*2+1, i2*2+1, nose_start + i2], [i*2+1, nose_start + i2, nose_start + i], [nose_start + i, nose_start + i2, tip_idx]])
            colors.extend([[0.9, 0.9, 0.9, 1], [0.9, 0.9, 0.9, 1], [0.9, 0.9, 0.9, 1]])

        for fin_angle in [0, 120, 240]:
            rad = math.radians(fin_angle)
            dx, dy = math.cos(rad), math.sin(rad)
            idx = len(verts)
            verts.extend([[r*dx, r*dy, 0], [r*dx, r*dy, 1.0], [(r+0.8)*dx, (r+0.8)*dy, 0]])
            faces.append([idx, idx+1, idx+2])
            colors.append([0.2, 0.2, 0.8, 1])

        return gl.GLMeshItem(vertexes=np.array(verts), faces=np.array(faces), faceColors=np.array(colors), smooth=False, drawEdges=True, edgeColor=(0.5, 0.5, 0.5, 0.5))

    def add_axes(self):
        for pos, color in [([[0,0,0],[3,0,0]], (1,0,0,1)), ([[0,0,0],[0,3,0]], (0,1,0,1)), ([[0,0,0],[0,0,3]], (0,0,1,1))]:
            self.addItem(gl.GLLinePlotItem(pos=np.array(pos), color=color, width=2))

    def update_attitude(self, pitch, roll, yaw=0):
        self.pitch, self.roll, self.yaw = pitch, roll, yaw
        self.rocket_mesh.resetTransform()
        self.rocket_mesh.rotate(roll, 1, 0, 0)
        self.rocket_mesh.rotate(pitch, 0, 1, 0)
        self.rocket_mesh.rotate(yaw, 0, 0, 1)

# ============================================================================
# Custom Widgets
# ============================================================================

class ValueDisplay(QFrame):
    def __init__(self, label, unit="", decimals=1):
        super().__init__()
        self.decimals, self.unit = decimals, unit
        self.setStyleSheet("QFrame { background-color: #111827; border: 1px solid #243141; border-radius: 8px; padding: 5px; }")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(2)

        self.label = QLabel(label)
        self.label.setStyleSheet("color: #9ca3af; font-size: 10px;")
        layout.addWidget(self.label)

        self.value = QLabel("---")
        self.value.setStyleSheet("color: #22c55e; font-size: 16px; font-weight: bold;")
        layout.addWidget(self.value)

    def set_value(self, val):
        if val is None:
            self.value.setText("---")
        elif isinstance(val, float):
            self.value.setText(f"{val:.{self.decimals}f} {self.unit}")
        else:
            self.value.setText(f"{val} {self.unit}")

    def set_color(self, color):
        self.value.setStyleSheet(f"color: {color}; font-size: 16px; font-weight: bold;")

class StateIndicator(QFrame):
    def __init__(self, title):
        super().__init__()
        self.setMinimumHeight(70)
        self.setStyleSheet("QFrame { background-color: #1f2a37; border: 2px solid #2a3a4f; border-radius: 12px; }")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 8, 12, 8)

        self.title_label = QLabel(title)
        self.title_label.setStyleSheet("color: #9ca3af; font-size: 10px; font-weight: bold;")
        self.title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.title_label)

        self.state_label = QLabel("---")
        self.state_label.setStyleSheet("color: white; font-size: 20px; font-weight: bold;")
        self.state_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.state_label)

    def set_state(self, state, color="#ffffff"):
        self.state_label.setText(state)
        self.setStyleSheet(f"QFrame {{ background-color: {color}; border: 2px solid {color}; border-radius: 12px; }}")

class SmallButton(QPushButton):
    def __init__(self, icon_type, tooltip=""):
        super().__init__()
        self.setToolTip(tooltip)
        self.setFixedSize(30, 30)

        icon_map = {
            "[ ]": "fullscreen",
            "R": "refresh",
            "C": "clear",
        }

        key = icon_map.get(icon_type, icon_type).lower()

        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)

        self.svg_widget = QSvgWidget()
        if key in SVG_ICONS:
            self.svg_widget.load(SVG_ICONS[key].encode('utf-8'))
        else:
            self.setText(icon_type)

        layout.addWidget(self.svg_widget)

        self.setStyleSheet("""
            QPushButton {
                background-color: #1f2a37;
                border: 1px solid #2a3a4f;
                border-radius: 4px;
            }
            QPushButton:hover { background-color: #273244; border-color: #22c55e; }
            QPushButton:pressed { background-color: #22c55e; }
        """)

class IconButton(QPushButton):
    def __init__(self, icon_key, tooltip=""):
        super().__init__()
        self.setToolTip(tooltip)
        self.setCursor(Qt.PointingHandCursor)
        self.setFixedSize(40, 32)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 6, 10, 6)
        layout.setSpacing(0)

        self.svg_widget = QSvgWidget()
        self.set_icon(icon_key)
        self.svg_widget.setFixedSize(20, 20)
        layout.addWidget(self.svg_widget, alignment=Qt.AlignCenter)

        self.setStyleSheet("""
            QPushButton {
                background-color: #1f2a37;
                border: 1px solid #2a3a4f;
                border-radius: 8px;
            }
            QPushButton:hover { background-color: #273244; border-color: #22c55e; }
            QPushButton:pressed { background-color: #22c55e; }
        """)

    def set_icon(self, icon_key):
        if icon_key in SVG_ICONS:
            self.svg_widget.load(SVG_ICONS[icon_key].encode('utf-8'))

class SideNavButton(QPushButton):
    def __init__(self, label, icon_key):
        super().__init__()
        self.full_label = label
        self.icon_key = icon_key
        self.setCheckable(True)
        self.setCursor(Qt.PointingHandCursor)
        self.setObjectName("sideNavButton")
        self.setMinimumHeight(44)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        self.layout = QHBoxLayout(self)
        self.layout.setContentsMargins(10, 0, 10, 0)
        self.layout.setSpacing(10)

        self.icon = QSvgWidget()
        if icon_key in SVG_ICONS:
            self.icon.load(SVG_ICONS[icon_key].encode('utf-8'))
        self.icon.setFixedSize(20, 20)
        self.layout.addWidget(self.icon, alignment=Qt.AlignLeft | Qt.AlignVCenter)

        self.label = QLabel(label)
        self.label.setStyleSheet("font-weight: 600;")
        self.layout.addWidget(self.label, alignment=Qt.AlignLeft | Qt.AlignVCenter)
        self.layout.addStretch()

    def set_collapsed(self, collapsed):
        self.label.setVisible(not collapsed)
        self.setToolTip(self.full_label)
        if collapsed:
            self.layout.setContentsMargins(0, 0, 0, 0)
            self.layout.setAlignment(self.icon, Qt.AlignCenter)
        else:
            self.layout.setContentsMargins(10, 0, 10, 0)
            self.layout.setAlignment(self.icon, Qt.AlignLeft | Qt.AlignVCenter)

class StatusItem(QFrame):
    def __init__(self, title, value="--", icon_key=None):
        super().__init__()
        self.setStyleSheet("""
            QFrame {
                background-color: #151a22;
                border: 1px solid #273241;
                border-radius: 10px;
            }
        """)
        self.setFixedHeight(54)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 6, 10, 6)
        layout.setSpacing(8)

        self.icon = QSvgWidget()
        if icon_key and icon_key in SVG_ICONS:
            self.icon.load(SVG_ICONS[icon_key].encode('utf-8'))
        self.icon.setFixedSize(18, 18)
        layout.addWidget(self.icon, alignment=Qt.AlignLeft | Qt.AlignVCenter)

        text_block = QWidget()
        text_layout = QVBoxLayout(text_block)
        text_layout.setContentsMargins(0, 0, 0, 0)
        text_layout.setSpacing(2)

        self.title = QLabel(title)
        self.title.setStyleSheet("color: #9ca3af; font-size: 10px; font-weight: bold;")
        text_layout.addWidget(self.title)

        self.value = QLabel(value)
        self.value.setStyleSheet("color: #e5e7eb; font-size: 14px; font-weight: bold;")
        text_layout.addWidget(self.value)

        layout.addWidget(text_block)
        layout.addStretch()

    def set_value(self, value):
        self.value.setText(value)

    def set_value_color(self, color):
        self.value.setStyleSheet(f"color: {color}; font-size: 14px; font-weight: bold;")

class XYZFooter(QWidget):
    def __init__(self, labels=("X", "Y", "Z"), colors=("#38bdf8", "#22c55e", "#a855f7"), unit=""):
        super().__init__()
        layout = QHBoxLayout(self)
        layout.setContentsMargins(6, 0, 6, 0)
        layout.setSpacing(10)

        self.values = []
        for label, color in zip(labels, colors):
            block = QWidget()
            block_layout = QVBoxLayout(block)
            block_layout.setContentsMargins(0, 0, 0, 0)
            block_layout.setSpacing(2)

            lbl = QLabel(label)
            lbl.setStyleSheet(f"color: {color}; font-size: 10px; font-weight: bold;")
            lbl.setAlignment(Qt.AlignCenter)
            block_layout.addWidget(lbl)

            val = QLabel(f"-- {unit}".strip())
            val.setStyleSheet("color: #e5e7eb; font-size: 11px; font-weight: bold;")
            val.setAlignment(Qt.AlignCenter)
            block_layout.addWidget(val)
            self.values.append(val)

            layout.addWidget(block)

    def set_values(self, values, unit=""):
        for i, val in enumerate(values):
            if i >= len(self.values):
                break
            if val is None:
                text = "--"
            else:
                text = f"{val:.2f}" if isinstance(val, float) else str(val)
            if unit:
                text = f"{text} {unit}"
            self.values[i].setText(text)

class SignalStatusWidget(QFrame):
    def __init__(self):
        super().__init__()
        self.setStyleSheet("""
            QFrame {
                background-color: #151a22;
                border: 1px solid #273241;
                border-radius: 10px;
            }
        """)
        self.setFixedHeight(54)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 6, 10, 6)
        layout.setSpacing(8)

        self.icon = QSvgWidget()
        if "signal" in SVG_ICONS:
            self.icon.load(SVG_ICONS["signal"].encode('utf-8'))
        self.icon.setFixedSize(18, 18)
        layout.addWidget(self.icon, alignment=Qt.AlignLeft | Qt.AlignVCenter)

        stack = QWidget()
        stack_layout = QVBoxLayout(stack)
        stack_layout.setContentsMargins(0, 0, 0, 0)
        stack_layout.setSpacing(2)

        title = QLabel("Signal")
        title.setStyleSheet("color: #9ca3af; font-size: 10px; font-weight: bold;")
        stack_layout.addWidget(title)

        icon_container = QWidget()
        icon_layout = QGridLayout(icon_container)
        icon_layout.setContentsMargins(0, 0, 0, 0)

        bars_widget = QWidget()
        bars_layout = QHBoxLayout(bars_widget)
        bars_layout.setContentsMargins(0, 0, 0, 0)
        bars_layout.setSpacing(3)
        bars_layout.setAlignment(Qt.AlignLeft | Qt.AlignBottom)

        self.bars = []
        for i in range(4):
            bar = QFrame()
            bar.setFixedSize(6, 8 + i * 5)
            bar.setStyleSheet("background-color: #334155; border-radius: 2px;")
            bars_layout.addWidget(bar)
            self.bars.append(bar)

        self.cross = QLabel("X")
        self.cross.setAlignment(Qt.AlignCenter)
        self.cross.setStyleSheet("color: #ef4444; font-weight: bold;")

        icon_layout.addWidget(bars_widget, 0, 0, alignment=Qt.AlignLeft | Qt.AlignBottom)
        icon_layout.addWidget(self.cross, 0, 0, alignment=Qt.AlignCenter)
        stack_layout.addWidget(icon_container)

        self.value = QLabel("-- kb/s | -- ms")
        self.value.setStyleSheet("color: #e5e7eb; font-size: 12px; font-weight: bold;")
        stack_layout.addWidget(self.value)

        layout.addWidget(stack)
        layout.addStretch()

    def set_signal(self, kbps, rtt_ms, ok):
        if ok:
            self.cross.hide()
        else:
            self.cross.show()

        strength = 0
        if ok:
            if kbps > 15:
                strength = 4
            elif kbps > 8:
                strength = 3
            elif kbps > 2:
                strength = 2
            else:
                strength = 1

        for i, bar in enumerate(self.bars):
            if ok and i < strength:
                bar.setStyleSheet("background-color: #22c55e; border-radius: 2px;")
            else:
                bar.setStyleSheet("background-color: #334155; border-radius: 2px;")

        rtt_text = f"{int(rtt_ms)} ms" if rtt_ms > 0 else "-- ms"
        self.value.setText(f"{kbps:.1f} kb/s | {rtt_text}")

class RealtimePlot(pg.PlotWidget):
    def __init__(self, title="", ylabel="", num_lines=1, colors=None, window=300, legend_names=None):
        super().__init__()
        self.window, self.num_lines = window, num_lines
        self.data = [deque(maxlen=window) for _ in range(num_lines)]
        self.times = deque(maxlen=window)
        self.start_time = time.time()
        self.plot_title = title
        self.legend_names = legend_names

        self.setTitle(title, color='#e5e7eb', size='10pt')
        self.setLabel('left', ylabel, color='#9ca3af')
        self.setLabel('bottom', 'Time (s)', color='#9ca3af')
        self.showGrid(x=True, y=True, alpha=0.3)
        self.setBackground('#0f1722')
        self.getAxis('left').setPen('#3b4a5f')
        self.getAxis('bottom').setPen('#3b4a5f')

        colors = colors or ['#22c55e', '#ef4444', '#06b6d4', '#f59e0b', '#14b8a6', '#0ea5e9']

        # Add legend for multi-line plots with X/Y/Z labels
        if legend_names and num_lines == 3:
            self.addLegend(offset=(70, 30))
            self.curves = [
                self.plot(pen=pg.mkPen(colors[i % len(colors)], width=2), name=legend_names[i])
                for i in range(num_lines)
            ]
        else:
            self.curves = [self.plot(pen=pg.mkPen(colors[i % len(colors)], width=2)) for i in range(num_lines)]

        self.enableAutoRange()

    def add_data(self, *values):
        self.times.append(time.time() - self.start_time)
        for i, v in enumerate(values):
            if i < self.num_lines:
                self.data[i].append(v)
        self.update_plot()

    def update_plot(self):
        if len(self.times) < 2:
            return
        times = list(self.times)
        for i, curve in enumerate(self.curves):
            if len(self.data[i]) > 0:
                curve.setData(times[:len(self.data[i])], list(self.data[i]))

    def reset_view(self):
        self.enableAutoRange()

# ============================================================================
# Plot Container with Controls
# ============================================================================

class PlotContainer(QWidget):
    def __init__(self, plot, title, parent_window=None, footer=None):
        super().__init__()
        self.plot, self.title, self.parent_window = plot, title, parent_window
        self.setMinimumHeight(220)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.setStyleSheet("""
            QWidget {
                background-color: #151a22;
                border: 1px solid #273241;
                border-radius: 10px;
            }
        """)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 6, 8, 8)
        layout.setSpacing(6)

        ctrl_bar = QWidget()
        ctrl_bar.setStyleSheet("background-color: #0f141c; border-radius: 6px;")
        ctrl_layout = QHBoxLayout(ctrl_bar)
        ctrl_layout.setContentsMargins(5, 2, 5, 2)
        ctrl_layout.addStretch()

        reset_btn = SmallButton("R", "Reset to auto-range")
        reset_btn.clicked.connect(self.reset_range)
        ctrl_layout.addWidget(reset_btn)

        expand_btn = SmallButton("[ ]", "Fullscreen")
        expand_btn.clicked.connect(self.open_fullscreen)
        ctrl_layout.addWidget(expand_btn)

        layout.addWidget(ctrl_bar)
        layout.addWidget(plot)
        if footer is not None:
            layout.addWidget(footer)

    def reset_range(self):
        self.plot.reset_view()

    def open_fullscreen(self):
        win = FullscreenPlotWindow(self.plot, self.title, self.parent_window)
        win.show()

class View3DContainer(QWidget):
    def __init__(self, view3d, parent_window=None):
        super().__init__()
        self.view3d, self.parent_window = view3d, parent_window

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        ctrl_bar = QWidget()
        ctrl_bar.setStyleSheet("background-color: #111827; border-radius: 5px;")
        ctrl_layout = QHBoxLayout(ctrl_bar)
        ctrl_layout.setContentsMargins(5, 2, 5, 2)
        ctrl_layout.addStretch()

        btn = SmallButton("[ ]", "Fullscreen")
        btn.clicked.connect(self.open_fullscreen)
        ctrl_layout.addWidget(btn)

        layout.addWidget(ctrl_bar)
        layout.addWidget(view3d)

    def open_fullscreen(self):
        win = Fullscreen3DWindow(self.view3d, self.parent_window)
        win.show()

# ============================================================================
# Console Widget with Header
# ============================================================================

class ConsoleWidget(QWidget):
    def __init__(self, parent_window=None):
        super().__init__()
        self.parent_window = parent_window
        self.log_entries = []
        self.max_entries = 500

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        header = QWidget()
        header.setStyleSheet("background-color: #111827; border-radius: 5px 5px 0 0;")
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(8, 4, 8, 4)

        title = QLabel("Console")
        title.setStyleSheet("color: #9ca3af; font-size: 11px; font-weight: bold;")
        header_layout.addWidget(title)
        header_layout.addStretch()

        clear_btn = SmallButton("C", "Clear console")
        clear_btn.clicked.connect(self.clear)
        header_layout.addWidget(clear_btn)

        expand_btn = SmallButton("[ ]", "Fullscreen")
        expand_btn.clicked.connect(self.open_fullscreen)
        header_layout.addWidget(expand_btn)

        layout.addWidget(header)

        self.text = QPlainTextEdit()
        self.text.setReadOnly(True)
        self.text.setMaximumBlockCount(self.max_entries)
        self.text.setStyleSheet("""
            QPlainTextEdit {
                background-color: #0b1117;
                color: #9ca3af;
                border: 1px solid #1f2a37;
                border-radius: 0 0 8px 8px;
                font-family: monospace;
                font-size: 11px;
                padding: 5px;
            }
        """)
        layout.addWidget(self.text)

    def log(self, msg):
        timestamp = datetime.now().strftime('%H:%M:%S')
        entry = f"[{timestamp}] {msg}"
        self.log_entries.append(entry)
        if len(self.log_entries) > self.max_entries:
            self.log_entries = self.log_entries[-self.max_entries:]
        self.text.appendPlainText(entry)

    def clear(self):
        self.text.clear()
        self.log_entries.clear()

    def open_fullscreen(self):
        win = FullscreenConsoleWindow(self.log_entries, self.parent_window)
        win.show()

# ============================================================================
# Main Dashboard Window
# ============================================================================

class DashboardWindow(QMainWindow):
    def __init__(self, port=None):
        super().__init__()
        self.setWindowTitle("Flight Computer Ground Station v5")
        self.setGeometry(50, 50, 1850, 1000)
        self.setStyleSheet("""
            QMainWindow { background-color: #0b1117; }
            QWidget { color: #d1d5db; font-family: 'Segoe UI'; }
            QGroupBox {
                color: #9ca3af;
                border: 1px solid #273241;
                background-color: #151a22;
                border-radius: 10px;
                margin-top: 12px;
                padding-top: 10px;
                font-weight: bold;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 6px; }
            QPushButton { background-color: #1f2a37; color: #e5e7eb; border: 1px solid #2a3a4f; border-radius: 8px; padding: 8px 15px; font-weight: bold; }
            QPushButton:hover { background-color: #273244; }
            QPushButton:pressed { background-color: #1b2431; }
            QComboBox { background-color: #1f2a37; color: #e5e7eb; border: 1px solid #2a3a4f; border-radius: 6px; padding: 5px 10px; }
            QComboBox:hover { background-color: #273244; }
            QComboBox::drop-down { border: none; width: 25px; }
            QComboBox::down-arrow { border-left: 5px solid transparent; border-right: 5px solid transparent; border-top: 6px solid #9ca3af; margin-right: 8px; }
            QComboBox QAbstractItemView { background-color: #1f2a37; color: #e5e7eb; border: 1px solid #2a3a4f; border-radius: 6px; selection-background-color: #22c55e; selection-color: #0b1117; outline: none; }
            QComboBox QAbstractItemView::item { padding: 8px; min-height: 25px; }
            QComboBox QAbstractItemView::item:hover { background-color: #273244; }
            QLineEdit { background-color: #111827; color: #9fffd5; border: 1px solid #2a3a4f; border-radius: 6px; padding: 8px; font-family: Consolas; }
            QTabWidget::pane { border: 1px solid #243141; border-radius: 8px; background-color: #0b1117; }
            QTabBar::tab { background-color: #111827; color: #9ca3af; padding: 8px 20px; border: 1px solid #243141; border-radius: 5px 5px 0 0; margin-right: 2px; }
            QTabBar::tab:selected { background-color: #1f2a37; color: #e5e7eb; }
            QScrollBar:vertical { background-color: #0f1722; width: 12px; border-radius: 6px; }
            QScrollBar::handle:vertical { background-color: #2a3a4f; border-radius: 6px; min-height: 30px; }
            QScrollBar::handle:vertical:hover { background-color: #35465e; }
            QPushButton#sideNavButton {
                text-align: left;
                padding-left: 12px;
                background-color: #1a2230;
                border: 1px solid #273241;
                border-radius: 10px;
            }
            QPushButton#sideNavButton:hover { background-color: #222c3b; }
            QPushButton#sideNavButton:checked { background-color: #22c55e; color: #0b1117; border-color: #22c55e; }
        """)

        self.fast_data, self.slow_data, self.gs_status, self.gs_stats = {}, {}, {}, {}
        self.rtt_ms, self.yaw = 0, 0
        self.open_windows = []
        self.multi_view_window = None

        self.serial_worker = SerialWorker()
        self.serial_worker.packet_received.connect(self.on_packet)
        self.serial_worker.log_received.connect(self.on_log)
        self.serial_worker.connection_changed.connect(self.on_connection)
        self.serial_worker.error_signal.connect(self.on_error)

        self.init_ui()
        self.last_stats_time = time.time()
        self.last_rx_total = 0
        self.rx_rate_kbps = 0.0
        self.rx_rate_history = []  # Rolling average for smoother display
        self.is_connected = False

        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(50)

        if port:
            self.port_combo.setCurrentText(port)
            self.connect_clicked()

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(8)
        main_layout.setContentsMargins(10, 10, 10, 10)

        # TOP STATUS BAR
        top_bar = QWidget()
        top_layout = QHBoxLayout(top_bar)
        top_layout.setContentsMargins(0, 0, 0, 0)
        top_layout.setSpacing(8)

        self.time_status = StatusItem("Time", "--:--:--", icon_key="time")
        self.sat_status = StatusItem("Satellite", "--", icon_key="satellite")
        self.state_status = StatusItem("Status", "BOOT / NONE", icon_key="status")
        self.signal_status = SignalStatusWidget()
        self.batt_status = StatusItem("Battery", "-- %", icon_key="battery")

        for widget in [self.time_status, self.sat_status, self.state_status, self.signal_status, self.batt_status]:
            top_layout.addWidget(widget, 1)

        main_layout.addWidget(top_bar)

        # MAIN CONTENT AREA
        content = QWidget()
        content_layout = QHBoxLayout(content)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(10)

        self.side_menu = QFrame()
        self.side_menu.setStyleSheet("QFrame { background-color: #0f141c; border: 1px solid #1f2a37; border-radius: 10px; }")
        self.side_menu.setFixedWidth(220)
        side_layout = QVBoxLayout(self.side_menu)
        side_layout.setContentsMargins(8, 8, 8, 8)
        side_layout.setSpacing(6)

        menu_header = QWidget()
        menu_header_layout = QHBoxLayout(menu_header)
        menu_header_layout.setContentsMargins(0, 0, 0, 0)
        menu_header_layout.setSpacing(6)

        self.menu_toggle = IconButton("menu", tooltip="Toggle menu")
        self.menu_toggle.clicked.connect(self.toggle_side_menu)
        menu_header_layout.addWidget(self.menu_toggle)
        menu_header_layout.addStretch()
        side_layout.addWidget(menu_header)

        self.nav_buttons = {}
        nav_items = [
            ("Dashboard", "dashboard"),
            ("3D Flight", "flight"),
            ("Location", "location"),
            ("Console", "console"),
            ("Communications", "comms"),
        ]
        for name, icon_key in nav_items:
            btn = SideNavButton(name, icon_key)
            btn.clicked.connect(lambda _, n=name: self.set_active_page(n))
            self.nav_buttons[name] = btn
            side_layout.addWidget(btn)

        side_layout.addStretch()
        content_layout.addWidget(self.side_menu)

        self.pages = QStackedWidget()
        self.page_widgets = {
            "Dashboard": self.build_dashboard_page(),
            "3D Flight": self.build_flight_page(),
            "Location": self.build_location_page(),
            "Console": self.build_console_page(),
            "Communications": self.build_comms_page(),
        }
        for page in self.page_widgets.values():
            self.pages.addWidget(page)

        content_layout.addWidget(self.pages, stretch=1)
        main_layout.addWidget(content, stretch=1)

        self.side_collapsed = False
        self.set_active_page("Dashboard")

    def toggle_side_menu(self):
        self.side_collapsed = not self.side_collapsed
        width = 64 if self.side_collapsed else 220
        self.side_menu.setFixedWidth(width)
        self.menu_toggle.set_icon("chevron" if self.side_collapsed else "menu")
        for btn in self.nav_buttons.values():
            btn.set_collapsed(self.side_collapsed)

    def set_active_page(self, name):
        page = self.page_widgets.get(name)
        if not page:
            return
        self.pages.setCurrentWidget(page)
        for btn_name, btn in self.nav_buttons.items():
            btn.setChecked(btn_name == name)
        if name == "Location" and HAS_WEBENGINE and self.gps_map is None:
            self.gps_map = GPSMapWidget()
            self.location_map_layout.addWidget(self.gps_map)

    def build_dashboard_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setSpacing(8)

        telem_widget = QWidget()
        telem_layout = QHBoxLayout(telem_widget)
        telem_layout.setSpacing(6)
        telem_layout.setContentsMargins(0, 0, 0, 0)

        self.alt_display = ValueDisplay("Altitude", "m", 1)
        self.vario_display = ValueDisplay("Velocity", "m/s", 2)
        self.temp_display = ValueDisplay("Temp", "C", 1)
        self.press_display = ValueDisplay("Pressure", "mbar", 1)
        self.batt_display = ValueDisplay("Battery", "%", 0)

        for w in [
            self.alt_display, self.vario_display, self.temp_display, self.press_display,
            self.batt_display
        ]:
            telem_layout.addWidget(w)

        layout.addWidget(telem_widget)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)

        grid_host = QWidget()
        grid = QGridLayout(grid_host)
        grid.setSpacing(10)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)
        grid_host.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.alt_plot = RealtimePlot("Altitude", "m", 1)
        self.vario_plot = RealtimePlot("Velocity", "m/s", 1)
        self.accel_plot = RealtimePlot("Accelerometers", "g", 3, legend_names=['X', 'Y', 'Z'])
        self.gyro_plot = RealtimePlot("Gyroscopes", "deg/s", 3, legend_names=['X', 'Y', 'Z'])
        self.orient_plot = RealtimePlot("Pitch / Roll / Yaw", "deg", 3, legend_names=['Pitch', 'Roll', 'Yaw'])
        self.temp_plot = RealtimePlot("Temperature", "C", 1)
        self.press_plot = RealtimePlot("Pressure", "mbar", 1)

        grid.addWidget(PlotContainer(self.alt_plot, "Altitude", self), 0, 0)
        grid.addWidget(PlotContainer(self.vario_plot, "Velocity", self), 0, 1)
        self.accel_footer = XYZFooter(unit="g")
        self.gyro_footer = XYZFooter(unit="deg/s")
        self.orient_footer = XYZFooter(labels=("Pitch", "Roll", "Yaw"), colors=("#38bdf8", "#22c55e", "#f59e0b"), unit="deg")
        grid.addWidget(PlotContainer(self.accel_plot, "Accelerometers", self, footer=self.accel_footer), 1, 0)
        grid.addWidget(PlotContainer(self.gyro_plot, "Gyroscopes", self, footer=self.gyro_footer), 1, 1)
        grid.addWidget(PlotContainer(self.temp_plot, "Temperature", self), 2, 0)
        grid.addWidget(PlotContainer(self.press_plot, "Pressure", self), 2, 1)
        grid.addWidget(PlotContainer(self.orient_plot, "Pitch / Roll / Yaw", self, footer=self.orient_footer), 3, 0)

        pwm_group = QGroupBox("MOTOR PWM")
        pwm_group.setMinimumHeight(220)
        pwm_group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        pwm_layout = QVBoxLayout(pwm_group)
        self.pwm_gauge = PWMGauge()
        pwm_layout.addWidget(self.pwm_gauge)
        grid.addWidget(pwm_group, 3, 1)

        status_group = QGroupBox("SYSTEM STATUS")
        status_group.setMinimumHeight(260)
        status_group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        status_layout = QVBoxLayout(status_group)
        self.boot_status = BootStatusWidget()
        self.state_indicator = StateIndicator("STATE")
        self.substate_indicator = StateIndicator("SUBSTATE")
        status_layout.addWidget(self.boot_status)
        status_layout.addWidget(self.state_indicator)
        status_layout.addWidget(self.substate_indicator)
        grid.addWidget(status_group, 4, 0, 1, 2)

        for row in range(0, 4):
            grid.setRowMinimumHeight(row, 220)

        scroll.setWidget(grid_host)
        layout.addWidget(scroll, stretch=1)

        return page

    def build_flight_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setSpacing(8)

        header = QLabel("3D Flight")
        header.setStyleSheet("color: #e5e7eb; font-size: 16px; font-weight: bold;")
        layout.addWidget(header)

        card = QFrame()
        card.setStyleSheet("QFrame { background-color: #151a22; border: 1px solid #273241; border-radius: 12px; }")
        card_layout = QVBoxLayout(card)
        card_layout.setContentsMargins(10, 10, 10, 10)
        card_layout.setSpacing(10)

        self.rocket_view = RocketView3D()
        self.rocket_view.setMinimumHeight(420)
        card_layout.addWidget(View3DContainer(self.rocket_view, self), stretch=1)

        info_row = QWidget()
        info_layout = QHBoxLayout(info_row)
        info_layout.setContentsMargins(0, 0, 0, 0)
        info_layout.setSpacing(12)

        self.attitude_indicator = AttitudeIndicator()
        self.attitude_indicator.setMinimumSize(180, 180)
        info_layout.addWidget(self.attitude_indicator, alignment=Qt.AlignLeft | Qt.AlignVCenter)

        self.flight_pitch_display = ValueDisplay("Pitch", "deg", 1)
        self.flight_roll_display = ValueDisplay("Roll", "deg", 1)
        self.flight_yaw_display = ValueDisplay("Yaw", "deg", 1)
        info_layout.addWidget(self.flight_pitch_display)
        info_layout.addWidget(self.flight_roll_display)
        info_layout.addWidget(self.flight_yaw_display)
        info_layout.addStretch()

        card_layout.addWidget(info_row)
        layout.addWidget(card, stretch=1)

        return page

    def build_location_page(self):
        page = QWidget()
        layout = QHBoxLayout(page)
        layout.setSpacing(10)

        self.gps_widget = GPSWidget()
        if HAS_WEBENGINE:
            self.gps_map = None
            map_holder = QGroupBox("MAP")
            self.location_map_layout = QVBoxLayout(map_holder)
            placeholder = QLabel("Map loading...")
            placeholder.setAlignment(Qt.AlignCenter)
            placeholder.setStyleSheet("color: #9ca3af;")
            self.location_map_layout.addWidget(placeholder)
            layout.addWidget(self.gps_widget, 1)
            layout.addWidget(map_holder, 2)
        else:
            self.gps_map = None
            layout.addWidget(self.gps_widget, 1)
            no_map = QLabel("GPS Map requires PyQtWebEngine")
            no_map.setStyleSheet("color: #9ca3af;")
            no_map.setAlignment(Qt.AlignCenter)
            layout.addWidget(no_map, 2)

        return page

    def build_console_page(self):
        page = QWidget()
        layout = QHBoxLayout(page)
        layout.setSpacing(10)

        controls = QWidget()
        controls_layout = QVBoxLayout(controls)
        controls_layout.setSpacing(10)

        conn_group = QGroupBox("CONNECTION")
        conn_layout = QGridLayout(conn_group)
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(100)
        self.refresh_ports()
        conn_layout.addWidget(self.port_combo, 0, 0)
        refresh_btn = SmallButton("refresh", "Refresh ports")
        refresh_btn.clicked.connect(self.refresh_ports)
        conn_layout.addWidget(refresh_btn, 0, 1)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setStyleSheet("background-color: #22c55e; color: #0b1117; border-radius: 8px;")
        self.connect_btn.clicked.connect(self.connect_clicked)
        conn_layout.addWidget(self.connect_btn, 1, 0, 1, 3)
        self.conn_status = QLabel("Disconnected")
        self.conn_status.setStyleSheet("color: #ef4444; font-size: 11px;")
        self.conn_status.setAlignment(Qt.AlignCenter)
        conn_layout.addWidget(self.conn_status, 2, 0, 1, 3)
        controls_layout.addWidget(conn_group)

        cmd_group = QGroupBox("COMMANDS")
        cmd_layout = QGridLayout(cmd_group)
        cmd_layout.setSpacing(6)
        cmd_buttons = [
            ("PING", 'P', "#0ea5e9"),
            ("CALIBRATE", 'C', "#14b8a6"),
            ("PROFILE 1", '1', "#22c55e"),
            ("PROFILE 2", '2', "#22c55e"),
            ("PROFILE 3", '3', "#22c55e"),
            ("ARM", 'A', "#f59e0b"),
            ("DISARM", 'D', "#64748b"),
            ("TEST", 'T', "#0ea5e9"),
            ("LAUNCH", 'L', "#f97316"),
            ("ABORT", 'X', "#ef4444"),
            ("SAFE", 'S', "#475569"),
            ("HELP", 'H', "#22c55e"),
        ]
        for i, (label, cmd, color) in enumerate(cmd_buttons):
            btn = QPushButton(label)
            btn.setStyleSheet(f"background-color: {color}; color: #0b1117; border-radius: 8px; padding: 10px;")
            btn.clicked.connect(self.show_help if cmd == 'H' else lambda _, c=cmd: self.send_command(c))
            cmd_layout.addWidget(btn, i // 3, i % 3)

        multiview_btn = QPushButton("MULTI-VIEW")
        multiview_btn.setStyleSheet("background-color: #14b8a6; color: #0b1117; border-radius: 8px; padding: 10px;")
        multiview_btn.clicked.connect(self.open_multiview)
        cmd_layout.addWidget(multiview_btn, 4, 0, 1, 3)
        controls_layout.addWidget(cmd_group)

        cmdline_group = QGroupBox("COMMAND LINE")
        cmdline_layout = QVBoxLayout(cmdline_group)
        self.cmd_input = QLineEdit()
        self.cmd_input.setPlaceholderText("Enter command...")
        self.cmd_input.returnPressed.connect(self.send_cmdline)
        cmdline_layout.addWidget(self.cmd_input)
        controls_layout.addWidget(cmdline_group)

        controls_layout.addStretch()

        console_group = QGroupBox("CONSOLE")
        console_layout = QVBoxLayout(console_group)
        self.console = ConsoleWidget(self)
        console_layout.addWidget(self.console)

        layout.addWidget(controls, 1)
        layout.addWidget(console_group, 2)
        return page

    def build_comms_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setSpacing(10)

        grid = QGridLayout()
        grid.setSpacing(10)

        self.rtt_display = ValueDisplay("RTT", "ms", 0)
        self.rx_display = ValueDisplay("RX Packets", "", 0)
        self.tx_display = ValueDisplay("TX Packets", "", 0)
        self.sync_display = ValueDisplay("TDMA", "", 0)
        self.crc_display = ValueDisplay("CRC Errors", "", 0)
        self.ack_ok_display = ValueDisplay("ACK OK", "", 0)
        self.ack_timeout_display = ValueDisplay("ACK Timeout", "", 0)

        grid.addWidget(self.rtt_display, 0, 0)
        grid.addWidget(self.rx_display, 0, 1)
        grid.addWidget(self.tx_display, 0, 2)
        grid.addWidget(self.sync_display, 1, 0)
        grid.addWidget(self.crc_display, 1, 1)
        grid.addWidget(self.ack_ok_display, 1, 2)
        grid.addWidget(self.ack_timeout_display, 2, 0)

        layout.addLayout(grid)
        layout.addStretch()
        return page

    def open_multiview(self):
        if self.multi_view_window is None or not self.multi_view_window.isVisible():
            self.multi_view_window = MultiViewWindow(self)
            self.multi_view_window.show()
        else:
            self.multi_view_window.raise_()
            self.multi_view_window.activateWindow()

    def show_help(self):
        HelpDialog(self).exec_()

    def refresh_ports(self):
        self.port_combo.clear()
        for port in serial.tools.list_ports.comports():
            self.port_combo.addItem(port.device)

    def connect_clicked(self):
        if self.serial_worker.isRunning():
            self.serial_worker.disconnect_port()
            self.serial_worker.wait()
            self.connect_btn.setText("Connect")
            self.connect_btn.setStyleSheet("background-color: #22c55e; color: #0b1117; border-radius: 8px;")
        else:
            port = self.port_combo.currentText()
            if port and self.serial_worker.connect_port(port):
                self.serial_worker.start()
                self.connect_btn.setText("Disconnect")
                self.connect_btn.setStyleSheet("background-color: #ef4444; color: #0b1117; border-radius: 8px;")

    def send_command(self, cmd):
        self.serial_worker.send_command(cmd)
        self.console.log(f"[TX] {cmd}")

    def send_cmdline(self):
        cmd = self.cmd_input.text().strip()
        if cmd:
            self.serial_worker.send_command(cmd)
            self.console.log(f"[CMD] {cmd}")
            self.cmd_input.clear()

    def on_packet(self, pkt):
        pkt_type = pkt.get('type', '')
        if pkt_type == 'fast':
            self.fast_data = pkt
            self.alt_plot.add_data(pkt['altitude'])
            self.vario_plot.add_data(pkt['vario'])
            self.accel_plot.add_data(pkt['accel_x'], pkt['accel_y'], pkt['accel_z'])
            self.gyro_plot.add_data(pkt['gyro_x'], pkt['gyro_y'], pkt['gyro_z'])
            self.yaw += pkt['gyro_z'] * 0.1
            self.yaw = self.yaw % 360
            self.orient_plot.add_data(pkt['pitch'], pkt['roll'], self.yaw)
            self.rocket_view.update_attitude(pkt['pitch'], pkt['roll'], self.yaw)
            self.attitude_indicator.set_attitude(pkt['pitch'], pkt['roll'], self.yaw)
        elif pkt_type == 'slow':
            self.slow_data = pkt
            self.temp_plot.add_data(pkt['temperature'])
            self.press_plot.add_data(pkt['pressure'])
            # Update GPS map
            if self.gps_map:
                lat = pkt.get('latitude', 0)
                lon = pkt.get('longitude', 0)
                lock = pkt.get('gps_lock', 0)
                self.gps_map.update_position(lat, lon, lock >= 2)
        elif pkt_type == 'event':
            evt_type = pkt['event_type']
            evt_name = EVENT_NAMES.get(evt_type, f"EVT_{evt_type}")
            self.console.log(f"[EVENT] {evt_name}")
            # Parse boot report
            if evt_type == 6:  # BOOT_REPORT
                self.boot_status.parse_boot_report(pkt['payload'])
        elif pkt_type == 'pong':
            self.rtt_ms = pkt['rtt_ms']
            self.console.log(f"[PONG] RTT={self.rtt_ms}ms")
        elif pkt_type == 'gs_status':
            self.gs_status = pkt
        elif pkt_type == 'gs_stats':
            self.gs_stats = pkt

    def on_log(self, msg):
        self.console.log(f"[GS] {msg}")

    def on_connection(self, connected):
        self.is_connected = connected
        if connected:
            self.conn_status.setText("Connected")
            self.conn_status.setStyleSheet("color: #22c55e; font-size: 11px;")
        else:
            self.conn_status.setText("Disconnected")
            self.conn_status.setStyleSheet("color: #ef4444; font-size: 11px;")
            # Reset rate tracking on disconnect
            self.rx_rate_kbps = 0.0
            self.rx_rate_history.clear()
            self.last_rx_total = 0
            self.last_stats_time = time.time()

    def on_error(self, msg):
        self.console.log(f"[ERROR] {msg}")

    def update_display(self):
        if self.fast_data:
            self.alt_display.set_value(self.fast_data.get('altitude'))
            self.vario_display.set_value(self.fast_data.get('vario'))
            ax, ay, az = self.fast_data.get('accel_x', 0), self.fast_data.get('accel_y', 0), self.fast_data.get('accel_z', 0)
            if hasattr(self, "accel_footer"):
                self.accel_footer.set_values((ax, ay, az), unit="g")
            gx, gy, gz = self.fast_data.get('gyro_x', 0), self.fast_data.get('gyro_y', 0), self.fast_data.get('gyro_z', 0)
            if hasattr(self, "gyro_footer"):
                self.gyro_footer.set_values((gx, gy, gz), unit="deg/s")
            if hasattr(self, "orient_footer"):
                self.orient_footer.set_values((self.fast_data.get('pitch'), self.fast_data.get('roll'), self.yaw), unit="deg")
            if hasattr(self, "flight_pitch_display"):
                self.flight_pitch_display.set_value(self.fast_data.get('pitch'))
                self.flight_roll_display.set_value(self.fast_data.get('roll'))
                self.flight_yaw_display.set_value(self.yaw)
            state_idx, substate_idx = self.fast_data.get('state', 0), self.fast_data.get('substate', 0)
            state_name = STATE_NAMES[state_idx] if state_idx < len(STATE_NAMES) else f"S{state_idx}"
            substate_name = SUBSTATE_NAMES[substate_idx] if substate_idx < len(SUBSTATE_NAMES) else f"SS{substate_idx}"
            self.state_indicator.set_state(state_name, STATE_COLORS.get(state_idx, "#666"))
            self.substate_indicator.set_state(substate_name, "#1f2a37")
            if self.is_connected:
                self.state_status.set_value("CONNECTED")
                self.state_status.set_value_color("#22c55e")
            else:
                self.state_status.set_value("DISCONNECTED")
                self.state_status.set_value_color("#ef4444")

        if self.slow_data:
            self.temp_display.set_value(self.slow_data.get('temperature'))
            self.press_display.set_value(self.slow_data.get('pressure'))
            self.batt_display.set_value(self.slow_data.get('battery'))
            self.gps_widget.update_data(
                self.slow_data.get('latitude'),
                self.slow_data.get('longitude'),
                self.slow_data.get('gps_altitude'),
                self.slow_data.get('satellites'),
                self.slow_data.get('gps_lock')
            )
            sats = self.slow_data.get('satellites')
            if sats is not None:
                self.sat_status.set_value(str(sats))
            batt = self.slow_data.get('battery')
            if batt is not None:
                self.batt_status.set_value(f"{batt} %")

        self.rtt_display.set_value(self.rtt_ms if self.rtt_ms > 0 else None)
        if self.rtt_ms > 0:
            self.rtt_display.set_color("#22c55e" if self.rtt_ms < 200 else "#f59e0b" if self.rtt_ms < 500 else "#ef4444")

        if self.gs_stats:
            rx_total = self.gs_stats.get('rx_fast', 0) + self.gs_stats.get('rx_slow', 0) + self.gs_stats.get('rx_event', 0)
            tx_total = self.gs_stats.get('tx_cmd', 0) + self.gs_stats.get('tx_sync', 0)
            self.rx_display.set_value(rx_total)
            self.tx_display.set_value(tx_total)
            self.crc_display.set_value(self.gs_stats.get('crc_errors', 0))
            self.ack_ok_display.set_value(self.gs_stats.get('ack_ok', 0))
            self.ack_timeout_display.set_value(self.gs_stats.get('ack_timeout', 0))

            now = time.time()
            dt = now - self.last_stats_time
            if dt >= 0.5:
                delta = rx_total - self.last_rx_total
                bytes_per_pkt = 32
                instant_rate = max(0.0, (delta * bytes_per_pkt * 8) / (dt * 1000.0))
                # Use rolling average (keep last 4 samples = ~2 seconds)
                self.rx_rate_history.append(instant_rate)
                if len(self.rx_rate_history) > 4:
                    self.rx_rate_history.pop(0)
                self.rx_rate_kbps = sum(self.rx_rate_history) / len(self.rx_rate_history)
                self.last_rx_total = rx_total
                self.last_stats_time = now

        if self.gs_status:
            synced = self.gs_status.get('synced')
            self.sync_display.set_value("SYNC" if synced else "UNSYNC")
            self.sync_display.set_color("#22c55e" if synced else "#ef4444")

        ok_signal = self.rtt_ms > 0 and (self.gs_status.get('synced') if self.gs_status else True)
        self.signal_status.set_signal(self.rx_rate_kbps, self.rtt_ms, ok_signal)
        self.time_status.set_value(datetime.now().strftime("%H:%M:%S"))
        if not self.fast_data:
            if self.is_connected:
                self.state_status.set_value("CONNECTED")
                self.state_status.set_value_color("#22c55e")
            else:
                self.state_status.set_value("DISCONNECTED")
                self.state_status.set_value_color("#ef4444")

    def closeEvent(self, event):
        self.serial_worker.running = False
        if self.serial_worker.isRunning():
            self.serial_worker.wait()
        if self.multi_view_window:
            self.multi_view_window.close()
        event.accept()

def main():
    parser = argparse.ArgumentParser(description='Flight Computer Dashboard v5')
    parser.add_argument('--port', '-p', type=str, help='Serial port')
    args = parser.parse_args()

    # Enable OpenGL context sharing BEFORE creating QApplication
    QApplication.setAttribute(Qt.AA_ShareOpenGLContexts)

    app = QApplication(sys.argv)
    app.setStyle(QStyleFactory.create('Fusion'))
    DashboardWindow(args.port).show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
