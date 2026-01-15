#!/usr/bin/env python3
"""
Flight Computer Ground Station Dashboard v4
============================================

Fixes from v3:
- Improved console with fullscreen and clear buttons
- Fixed 3D fullscreen view
- Better icons for expand/reset buttons
- Complete GPS section with lock status
- Improved Flight State and Commands styling

Requirements:
    pip install pyserial pyqt5 pyqtgraph numpy pyopengl

Usage:
    python dashboard_v4.py --port COM3
"""

import sys
import argparse
import struct
import time
import math
from collections import deque
from datetime import datetime
from queue import Queue, Empty
from PyQt5.QtSvg import QSvgWidget

import serial
import serial.tools.list_ports
import numpy as np

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QComboBox, QGroupBox,
    QTextEdit, QTabWidget, QFrame, QLineEdit, QPlainTextEdit,
    QStyleFactory, QDialog, QTableWidget, QTableWidgetItem,
    QHeaderView, QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread
from PyQt5.QtGui import QFont, QColor, QTextCursor

import pyqtgraph as pg
import pyqtgraph.opengl as gl

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

COMMANDS_DOC = [
    ("P", "PING", "Send ping to FC, measures round-trip time"),
    ("C", "CALIBRATE", "Calibrate barometer (set current altitude as zero)"),
    ("1", "PROFILE 1", "Load profile 1: Gutter Ramp (5s ramp time)"),
    ("2", "PROFILE 2", "Load profile 2: Gutter Hold (0.3 throttle)"),
    ("3", "PROFILE 3", "Load profile 3: Flight Param (50m target)"),
    ("A", "ARM", "Arm the flight computer and motors"),
    ("D", "DISARM", "Disarm the flight computer"),
    ("T", "TEST", "Start test stand sequence"),
    ("L", "LAUNCH", "Initiate launch sequence"),
    ("X", "ABORT", "Emergency abort - cut throttle immediately"),
    ("S", "SAFE", "Force transition to SAFE state"),
    ("R", "RESET STATS", "Reset communication statistics"),
    ("?", "DEBUG", "Toggle debug output on Arduino"),
]

# Ícones em formato SVG (Base64/String) para garantir que aparecem sempre
SVG_ICONS = {
    "fullscreen": """<svg viewBox="0 0 24 24"><path fill="#00ff88" d="M7 14H5v5h5v-2H7v-3zm-2-4h2V7h3V5H5v5zm12 7h-3v2h5v-5h-2v3zM14 5v2h3v3h2V5h-5z"/></svg>""",
    "refresh": """<svg viewBox="0 0 24 24"><path fill="#00ff88" d="M17.65 6.35C16.2 4.9 14.21 4 12 4c-4.42 0-7.99 3.58-7.99 8s3.57 8 7.99 8c3.73 0 6.84-2.55 7.73-6h-2.08c-.82 2.33-3.04 4-5.65 4-3.31 0-6-2.69-6-6s2.69-6 6-6c1.66 0 3.14.69 4.22 1.78L13 11h7V4l-2.35 2.35z"/></svg>""",
    "clear": """<svg viewBox="0 0 24 24"><path fill="#ff6b6b" d="M19 6.41L17.59 5 12 10.59 6.41 5 5 6.41 10.59 12 5 17.59 6.41 19 12 13.41 17.59 19 19 17.59 13.41 12z"/></svg>"""
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
# Help Dialog
# ============================================================================

class HelpDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Command Reference")
        self.setMinimumSize(650, 550)
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
        """)

        layout = QVBoxLayout(self)
        layout.setSpacing(15)
        layout.setContentsMargins(20, 20, 20, 20)

        title = QLabel("Flight Computer Commands")
        title.setStyleSheet("font-size: 18px; font-weight: bold; color: #00ff88;")
        layout.addWidget(title)

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

        layout.addWidget(table)

        states_label = QLabel("State Machine: BOOT > IDLE > CONFIGED > ARMED > TEST_STAND/FLIGHT > SAFE")
        states_label.setStyleSheet("color: #888; font-size: 11px;")
        layout.addWidget(states_label)

        close_btn = QPushButton("Close")
        close_btn.clicked.connect(self.accept)
        layout.addWidget(close_btn, alignment=Qt.AlignCenter)

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

        # Create curves matching original
        colors = ['#00ff88', '#ff6b6b', '#4ecdc4', '#ffe66d', '#a855f7', '#06b6d4']
        self.curves = []
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
    """
    Fullscreen 3D attitude display with rocket model.
    Uses AA_ShareOpenGLContexts set in main() to share OpenGL context.
    """
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

        # 3D Rocket View (context sharing enabled via AA_ShareOpenGLContexts)
        self.rocket_view = RocketView3D()
        self.rocket_view.setCameraPosition(distance=20, elevation=25, azimuth=45)
        layout.addWidget(self.rocket_view, stretch=1)

        # Info panel with attitude values
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

        # Close button
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

        # Update timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.sync_view)
        self.timer.start(50)

    def sync_view(self):
        """Sync attitude from parent view to fullscreen 3D model."""
        try:
            p = self.parent_view.pitch
            r = self.parent_view.roll
            y = self.parent_view.yaw

            # Update labels
            self.pitch_lbl.setText(f"PITCH: {p:>7.1f}")
            self.roll_lbl.setText(f"ROLL: {r:>7.1f}")
            self.yaw_lbl.setText(f"YAW: {y:>7.1f}")

            # Update 3D model
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
        self.setStyleSheet("QFrame { background-color: #1e1e2e; border: 1px solid #3d3d5c; border-radius: 8px; padding: 5px; }")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(2)

        self.label = QLabel(label)
        self.label.setStyleSheet("color: #888; font-size: 10px;")
        layout.addWidget(self.label)

        self.value = QLabel("---")
        self.value.setStyleSheet("color: #00ff88; font-size: 16px; font-weight: bold;")
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
        self.setStyleSheet("QFrame { background-color: #2d2d44; border: 2px solid #4d4d6d; border-radius: 12px; }")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 8, 12, 8)

        self.title_label = QLabel(title)
        self.title_label.setStyleSheet("color: #888; font-size: 10px; font-weight: bold;")
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
        
        # TRADUÇÃO AUTOMÁTICA (Corrige o problema dos ícones invisíveis)
        # Mapeia o texto antigo do teu código para os novos SVGs
        icon_map = {
            "[ ]": "fullscreen",  # Traduz "[ ]" para o ícone de ecrã inteiro
            "R": "refresh",       # Traduz "R" para o ícone de refresh/reset
            "C": "clear",         # Traduz "C" para o ícone de limpar
        }
        
        # Verifica se é um atalho conhecido ou usa o valor direto (ex: "fullscreen")
        # .lower() garante que funciona mesmo se escreveres "Fullscreen"
        key = icon_map.get(icon_type, icon_type).lower()
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5) # Margem para o ícone não ficar gigante
        
        self.svg_widget = QSvgWidget()
        if key in SVG_ICONS:
            self.svg_widget.load(SVG_ICONS[key].encode('utf-8'))
        else:
            # Fallback: Se não encontrar ícone, mostra o texto original (segurança)
            self.setText(icon_type)
            
        layout.addWidget(self.svg_widget)
        
        self.setStyleSheet("""
            QPushButton {
                background-color: #2a2a40;
                border: 1px solid #3d3d5c;
                border-radius: 4px;
            }
            QPushButton:hover { background-color: #3d3d5c; border-color: #00ff88; }
            QPushButton:pressed { background-color: #00ff88; }
        """)

class RealtimePlot(pg.PlotWidget):
    def __init__(self, title="", ylabel="", num_lines=1, colors=None, window=300):
        super().__init__()
        self.window, self.num_lines = window, num_lines
        self.data = [deque(maxlen=window) for _ in range(num_lines)]
        self.times = deque(maxlen=window)
        self.start_time = time.time()
        self.plot_title = title

        self.setTitle(title, color='w', size='10pt')
        self.setLabel('left', ylabel, color='#888')
        self.setLabel('bottom', 'Time (s)', color='#888')
        self.showGrid(x=True, y=True, alpha=0.3)
        self.setBackground('#1a1a2e')
        self.getAxis('left').setPen('#555')
        self.getAxis('bottom').setPen('#555')

        colors = colors or ['#00ff88', '#ff6b6b', '#4ecdc4', '#ffe66d', '#a855f7', '#06b6d4']
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
    def __init__(self, plot, title, parent_window=None):
        super().__init__()
        self.plot, self.title, self.parent_window = plot, title, parent_window

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        ctrl_bar = QWidget()
        ctrl_bar.setStyleSheet("background-color: #1e1e2e; border-radius: 5px;")
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

    def reset_range(self):
        self.plot.reset_view()

    def open_fullscreen(self):
        FullscreenPlotWindow(self.plot, self.title, self.parent_window).exec_()

class View3DContainer(QWidget):
    def __init__(self, view3d, parent_window=None):
        super().__init__()
        self.view3d, self.parent_window = view3d, parent_window

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        ctrl_bar = QWidget()
        ctrl_bar.setStyleSheet("background-color: #1e1e2e; border-radius: 5px;")
        ctrl_layout = QHBoxLayout(ctrl_bar)
        ctrl_layout.setContentsMargins(5, 2, 5, 2)
        ctrl_layout.addStretch()

        btn = SmallButton("[ ]", "Fullscreen")
        btn.clicked.connect(self.open_fullscreen)
        ctrl_layout.addWidget(btn)

        layout.addWidget(ctrl_bar)
        layout.addWidget(view3d)

    def open_fullscreen(self):
        Fullscreen3DWindow(self.view3d, self.parent_window).exec_()

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

        # Header bar
        header = QWidget()
        header.setStyleSheet("background-color: #1e1e2e; border-radius: 5px 5px 0 0;")
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(8, 4, 8, 4)

        title = QLabel("Console")
        title.setStyleSheet("color: #888; font-size: 11px; font-weight: bold;")
        header_layout.addWidget(title)
        header_layout.addStretch()

        clear_btn = SmallButton("C", "Clear console")
        clear_btn.clicked.connect(self.clear)
        header_layout.addWidget(clear_btn)

        expand_btn = SmallButton("[ ]", "Fullscreen")
        expand_btn.clicked.connect(self.open_fullscreen)
        header_layout.addWidget(expand_btn)

        layout.addWidget(header)

        # Text area
        self.text = QPlainTextEdit()
        self.text.setReadOnly(True)
        self.text.setMaximumBlockCount(self.max_entries)
        self.text.setStyleSheet("""
            QPlainTextEdit {
                background-color: #0d0d1a;
                color: #888;
                border: 1px solid #2d2d44;
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
        FullscreenConsoleWindow(self.log_entries, self.parent_window).exec_()

# ============================================================================
# GPS Widget
# ============================================================================

class GPSWidget(QFrame):
    def __init__(self):
        super().__init__()
        self.setStyleSheet("QFrame { background-color: #1e1e2e; border: 1px solid #3d3d5c; border-radius: 8px; }")

        layout = QGridLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        # Row 0: Title
        title = QLabel("GPS NAVIGATION")
        title.setStyleSheet("color: #888; font-size: 10px; font-weight: bold;")
        layout.addWidget(title, 0, 0, 1, 4)

        # Row 1: Lat/Lon
        layout.addWidget(QLabel("LAT"), 1, 0)
        self.lat_value = QLabel("---.------")
        self.lat_value.setStyleSheet("color: #00ff88; font-size: 14px; font-weight: bold; font-family: monospace;")
        layout.addWidget(self.lat_value, 1, 1)

        layout.addWidget(QLabel("LON"), 1, 2)
        self.lon_value = QLabel("---.------")
        self.lon_value.setStyleSheet("color: #00ff88; font-size: 14px; font-weight: bold; font-family: monospace;")
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

    def update_data(self, lat, lon, alt, sats, lock):
        self.lat_value.setText(f"{lat:.6f}" if lat else "---.------")
        self.lon_value.setText(f"{lon:.6f}" if lon else "---.------")
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
# Main Dashboard Window
# ============================================================================

class DashboardWindow(QMainWindow):
    def __init__(self, port=None):
        super().__init__()
        self.setWindowTitle("Flight Computer Ground Station v4")
        self.setGeometry(50, 50, 1800, 1000)
        self.setStyleSheet("""
            QMainWindow { background-color: #0d0d1a; }
            QLabel { color: #ddd; }
            QGroupBox {
                color: #aaa;
                border: 1px solid #3d3d5c;
                border-radius: 10px;
                margin-top: 12px;
                padding-top: 10px;
                font-weight: bold;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 6px; }
            QPushButton { background-color: #2d2d44; color: white; border: 1px solid #4d4d6d; border-radius: 8px; padding: 8px 15px; font-weight: bold; }
            QPushButton:hover { background-color: #3d3d5c; }
            QPushButton:pressed { background-color: #1d1d2e; }
            QComboBox { background-color: #2d2d44; color: white; border: 1px solid #4d4d6d; border-radius: 6px; padding: 5px 10px; }
            QComboBox:hover { background-color: #3d3d5c; }
            QComboBox::drop-down { border: none; width: 25px; }
            QComboBox::down-arrow { border-left: 5px solid transparent; border-right: 5px solid transparent; border-top: 6px solid #888; margin-right: 8px; }
            QComboBox QAbstractItemView { background-color: #2d2d44; color: white; border: 1px solid #4d4d6d; border-radius: 6px; selection-background-color: #3498db; selection-color: white; outline: none; }
            QComboBox QAbstractItemView::item { padding: 8px; min-height: 25px; }
            QComboBox QAbstractItemView::item:hover { background-color: #3d3d5c; }
            QLineEdit { background-color: #1e1e2e; color: #00ff88; border: 1px solid #4d4d6d; border-radius: 6px; padding: 8px; font-family: monospace; }
            QTabWidget::pane { border: 1px solid #3d3d5c; border-radius: 8px; background-color: #0d0d1a; }
            QTabBar::tab { background-color: #1e1e2e; color: #888; padding: 8px 20px; border: 1px solid #3d3d5c; border-radius: 5px 5px 0 0; margin-right: 2px; }
            QTabBar::tab:selected { background-color: #2d2d44; color: white; }
            QScrollBar:vertical { background-color: #1a1a2e; width: 12px; border-radius: 6px; }
            QScrollBar::handle:vertical { background-color: #3d3d5c; border-radius: 6px; min-height: 30px; }
            QScrollBar::handle:vertical:hover { background-color: #4d4d6d; }
        """)

        self.fast_data, self.slow_data, self.gs_status, self.gs_stats = {}, {}, {}, {}
        self.rtt_ms, self.yaw = 0, 0

        self.serial_worker = SerialWorker()
        self.serial_worker.packet_received.connect(self.on_packet)
        self.serial_worker.log_received.connect(self.on_log)
        self.serial_worker.connection_changed.connect(self.on_connection)
        self.serial_worker.error_signal.connect(self.on_error)

        self.init_ui()

        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(50)

        if port:
            self.port_combo.setCurrentText(port)
            self.connect_clicked()

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(10, 10, 10, 10)

        # LEFT PANEL
        left_panel = QWidget()
        left_panel.setMaximumWidth(320)
        left_layout = QVBoxLayout(left_panel)
        left_layout.setSpacing(10)

        # Connection
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
        self.connect_btn.setStyleSheet("background-color: #2ecc71; border-radius: 8px;")
        self.connect_btn.clicked.connect(self.connect_clicked)
        conn_layout.addWidget(self.connect_btn, 1, 0, 1, 3)
        self.conn_status = QLabel("Disconnected")
        self.conn_status.setStyleSheet("color: #ff6b6b; font-size: 11px;")
        self.conn_status.setAlignment(Qt.AlignCenter)
        conn_layout.addWidget(self.conn_status, 2, 0, 1, 3)
        left_layout.addWidget(conn_group)

        # State
        state_group = QGroupBox("FLIGHT STATE")
        state_layout = QVBoxLayout(state_group)
        self.state_indicator = StateIndicator("STATE")
        self.substate_indicator = StateIndicator("SUBSTATE")
        state_layout.addWidget(self.state_indicator)
        state_layout.addWidget(self.substate_indicator)
        left_layout.addWidget(state_group)

        # Commands
        cmd_group = QGroupBox("COMMANDS")
        cmd_layout = QGridLayout(cmd_group)
        cmd_layout.setSpacing(6)
        cmd_buttons = [
            ("PING", 'P', "#3498db"),
            ("CALIBRATE", 'C', "#9b59b6"),
            ("PROFILE 1", '1', "#27ae60"),
            ("PROFILE 2", '2', "#27ae60"),
            ("PROFILE 3", '3', "#27ae60"),
            ("ARM", 'A', "#e67e22"),
            ("DISARM", 'D', "#7f8c8d"),
            ("TEST", 'T', "#1abc9c"),
            ("LAUNCH", 'L', "#c0392b"),
            ("ABORT", 'X', "#8e1b1b"),
            ("SAFE", 'S', "#5d6d7e"),
            ("HELP", 'H', "#8e44ad"),
        ]
        for i, (label, cmd, color) in enumerate(cmd_buttons):
            btn = QPushButton(label)
            btn.setStyleSheet(f"background-color: {color}; border-radius: 8px; padding: 10px;")
            btn.clicked.connect(self.show_help if cmd == 'H' else lambda _, c=cmd: self.send_command(c))
            cmd_layout.addWidget(btn, i // 3, i % 3)
        left_layout.addWidget(cmd_group)

        # Command Line
        cmdline_group = QGroupBox("COMMAND LINE")
        cmdline_layout = QVBoxLayout(cmdline_group)
        self.cmd_input = QLineEdit()
        self.cmd_input.setPlaceholderText("Enter command...")
        self.cmd_input.returnPressed.connect(self.send_cmdline)
        cmdline_layout.addWidget(self.cmd_input)
        left_layout.addWidget(cmdline_group)

        # Communication Stats
        comm_group = QGroupBox("COMMUNICATION")
        comm_layout = QGridLayout(comm_group)
        self.rtt_display = ValueDisplay("RTT", "ms", 0)
        self.rx_display = ValueDisplay("RX Packets", "", 0)
        self.sync_display = ValueDisplay("TDMA", "", 0)
        self.crc_display = ValueDisplay("CRC Errors", "", 0)
        comm_layout.addWidget(self.rtt_display, 0, 0)
        comm_layout.addWidget(self.rx_display, 0, 1)
        comm_layout.addWidget(self.sync_display, 1, 0)
        comm_layout.addWidget(self.crc_display, 1, 1)
        left_layout.addWidget(comm_group)
        left_layout.addStretch()
        main_layout.addWidget(left_panel)

        # CENTER PANEL
        center_panel = QWidget()
        center_layout = QVBoxLayout(center_panel)
        center_layout.setSpacing(10)

        # Telemetry Values
        telem_widget = QWidget()
        telem_layout = QHBoxLayout(telem_widget)
        telem_layout.setSpacing(5)
        self.alt_display = ValueDisplay("Altitude", "m", 1)
        self.vario_display = ValueDisplay("Vario", "m/s", 2)
        self.accel_display = ValueDisplay("Accel", "g", 2)
        self.pitch_display = ValueDisplay("Pitch", "deg", 1)
        self.roll_display = ValueDisplay("Roll", "deg", 1)
        self.yaw_display = ValueDisplay("Yaw", "deg", 1)
        self.temp_display = ValueDisplay("Temp", "C", 1)
        self.press_display = ValueDisplay("Pressure", "mbar", 1)
        self.batt_display = ValueDisplay("Battery", "%", 0)
        for w in [self.alt_display, self.vario_display, self.accel_display, self.pitch_display, self.roll_display, self.yaw_display, self.temp_display, self.press_display, self.batt_display]:
            telem_layout.addWidget(w)
        center_layout.addWidget(telem_widget)

        # Plots
        plots_tabs = QTabWidget()
        self.alt_plot = RealtimePlot("Altitude", "m", 1)
        plots_tabs.addTab(PlotContainer(self.alt_plot, "Altitude", self), "Altitude")

        imu_widget = QWidget()
        imu_layout = QVBoxLayout(imu_widget)
        self.accel_plot = RealtimePlot("Acceleration", "g", 3, ['#ff6b6b', '#4ecdc4', '#ffe66d'])
        self.gyro_plot = RealtimePlot("Gyroscope", "deg/s", 3, ['#ff6b6b', '#4ecdc4', '#ffe66d'])
        imu_layout.addWidget(PlotContainer(self.accel_plot, "Acceleration", self))
        imu_layout.addWidget(PlotContainer(self.gyro_plot, "Gyroscope", self))
        plots_tabs.addTab(imu_widget, "IMU")

        self.orient_plot = RealtimePlot("Orientation", "deg", 3, ['#a855f7', '#06b6d4', '#ffe66d'])
        plots_tabs.addTab(PlotContainer(self.orient_plot, "Orientation", self), "Orientation")

        env_widget = QWidget()
        env_layout = QVBoxLayout(env_widget)
        self.temp_plot = RealtimePlot("Temperature", "C", 1, ['#ff6b6b'])
        self.press_plot = RealtimePlot("Pressure", "mbar", 1, ['#4ecdc4'])
        env_layout.addWidget(PlotContainer(self.temp_plot, "Temperature", self))
        env_layout.addWidget(PlotContainer(self.press_plot, "Pressure", self))
        plots_tabs.addTab(env_widget, "Environment")

        center_layout.addWidget(plots_tabs, stretch=1)
        main_layout.addWidget(center_panel, stretch=1)

        # RIGHT PANEL
        right_panel = QWidget()
        right_panel.setMaximumWidth(400)
        right_layout = QVBoxLayout(right_panel)
        right_layout.setSpacing(10)

        # 3D View
        view3d_group = QGroupBox("3D ATTITUDE")
        view3d_layout = QVBoxLayout(view3d_group)
        self.rocket_view = RocketView3D()
        self.rocket_view.setMinimumHeight(300)
        view3d_layout.addWidget(View3DContainer(self.rocket_view, self))
        right_layout.addWidget(view3d_group)

        # GPS
        gps_group = QGroupBox("GPS")
        gps_layout = QVBoxLayout(gps_group)
        self.gps_widget = GPSWidget()
        gps_layout.addWidget(self.gps_widget)
        right_layout.addWidget(gps_group)

        # Console
        console_group = QGroupBox("CONSOLE")
        console_layout = QVBoxLayout(console_group)
        self.console = ConsoleWidget(self)
        console_layout.addWidget(self.console)
        right_layout.addWidget(console_group, stretch=1)

        main_layout.addWidget(right_panel)

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
            self.connect_btn.setStyleSheet("background-color: #2ecc71; border-radius: 8px;")
        else:
            port = self.port_combo.currentText()
            if port and self.serial_worker.connect_port(port):
                self.serial_worker.start()
                self.connect_btn.setText("Disconnect")
                self.connect_btn.setStyleSheet("background-color: #e74c3c; border-radius: 8px;")

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
            self.accel_plot.add_data(pkt['accel_x'], pkt['accel_y'], pkt['accel_z'])
            self.gyro_plot.add_data(pkt['gyro_x'], pkt['gyro_y'], pkt['gyro_z'])
            self.yaw += pkt['gyro_z'] * 0.1
            self.yaw = self.yaw % 360
            self.orient_plot.add_data(pkt['pitch'], pkt['roll'], self.yaw)
            self.rocket_view.update_attitude(pkt['pitch'], pkt['roll'], self.yaw)
        elif pkt_type == 'slow':
            self.slow_data = pkt
            self.temp_plot.add_data(pkt['temperature'])
            self.press_plot.add_data(pkt['pressure'])
        elif pkt_type == 'event':
            evt_type = pkt['event_type']
            evt_name = EVENT_NAMES.get(evt_type, f"EVT_{evt_type}")
            self.console.log(f"[EVENT] {evt_name}")
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
        if connected:
            self.conn_status.setText("Connected")
            self.conn_status.setStyleSheet("color: #00ff88; font-size: 11px;")
        else:
            self.conn_status.setText("Disconnected")
            self.conn_status.setStyleSheet("color: #ff6b6b; font-size: 11px;")

    def on_error(self, msg):
        self.console.log(f"[ERROR] {msg}")

    def update_display(self):
        if self.fast_data:
            self.alt_display.set_value(self.fast_data.get('altitude'))
            self.vario_display.set_value(self.fast_data.get('vario'))
            ax, ay, az = self.fast_data.get('accel_x', 0), self.fast_data.get('accel_y', 0), self.fast_data.get('accel_z', 0)
            self.accel_display.set_value(math.sqrt(ax*ax + ay*ay + az*az))
            self.pitch_display.set_value(self.fast_data.get('pitch'))
            self.roll_display.set_value(self.fast_data.get('roll'))
            self.yaw_display.set_value(self.yaw)
            state_idx, substate_idx = self.fast_data.get('state', 0), self.fast_data.get('substate', 0)
            state_name = STATE_NAMES[state_idx] if state_idx < len(STATE_NAMES) else f"S{state_idx}"
            substate_name = SUBSTATE_NAMES[substate_idx] if substate_idx < len(SUBSTATE_NAMES) else f"SS{substate_idx}"
            self.state_indicator.set_state(state_name, STATE_COLORS.get(state_idx, "#666"))
            self.substate_indicator.set_state(substate_name, "#2d2d44")

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

        self.rtt_display.set_value(self.rtt_ms if self.rtt_ms > 0 else None)
        if self.rtt_ms > 0:
            self.rtt_display.set_color("#00ff88" if self.rtt_ms < 200 else "#ffe66d" if self.rtt_ms < 500 else "#ff6b6b")

        if self.gs_stats:
            self.rx_display.set_value(self.gs_stats.get('rx_fast', 0) + self.gs_stats.get('rx_slow', 0))
            self.crc_display.set_value(self.gs_stats.get('crc_errors', 0))

        if self.gs_status:
            synced = self.gs_status.get('synced')
            self.sync_display.set_value("SYNC" if synced else "UNSYNC")
            self.sync_display.set_color("#00ff88" if synced else "#ff6b6b")

    def closeEvent(self, event):
        self.serial_worker.running = False
        if self.serial_worker.isRunning():
            self.serial_worker.wait()
        event.accept()

def main():
    parser = argparse.ArgumentParser(description='Flight Computer Dashboard v4')
    parser.add_argument('--port', '-p', type=str, help='Serial port')
    args = parser.parse_args()

    # Enable OpenGL context sharing BEFORE creating QApplication
    # This fixes "GLError invalid operation" when creating multiple GLViewWidgets
    QApplication.setAttribute(Qt.AA_ShareOpenGLContexts)

    app = QApplication(sys.argv)
    app.setStyle(QStyleFactory.create('Fusion'))
    DashboardWindow(args.port).show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
