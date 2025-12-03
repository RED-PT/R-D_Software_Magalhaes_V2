#!/usr/bin/env python3
"""
Flight Computer Ground Station Dashboard
==========================================

Connects to Arduino via serial and displays real-time telemetry.
Uses PyQt5 for GUI and pyqtgraph for real-time plotting.

Requirements:
    pip install pyserial pyqt5 pyqtgraph numpy

Usage:
    python dashboard.py --port COM3  # Windows
    python dashboard.py --port /dev/ttyUSB0  # Linux
"""

import sys
import argparse
import struct
import time
from collections import deque
from datetime import datetime
import threading

import serial
import serial.tools.list_ports
import numpy as np

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QComboBox, QGroupBox,
    QTextEdit, QTabWidget, QProgressBar, QFrame, QSplitter,
    QLineEdit, QSpinBox, QDoubleSpinBox, QMessageBox
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QColor, QPalette

import pyqtgraph as pg

# ============================================================================
# Protocol Constants (must match FC/Arduino)
# ============================================================================

PACKET_TYPES = {
    0x01: 'FAST',
    0x02: 'SLOW',
    0x03: 'EVENT',
    0x04: 'COMMAND',
    0x05: 'SYNC'
}

STATE_NAMES = [
    "BOOT", "IDLE", "CONFIGED", "ARMED",
    "TEST_STAND", "FLIGHT", "ABORT", "SAFE"
]

SUBSTATE_NAMES = [
    "NONE",
    "TS_SENSOR_CHECK", "TS_THROTTLE_RAMP",
    "FL_IGNITION", "FL_LIFTOFF_DETECT", "FL_ASCENT", "FL_COAST",
    "FL_DESCENT_BRAKE", "FL_LANDING_FLARE", "FL_TOUCHDOWN", "FL_RECOVERY",
    "ARM_MOTOR_INIT", "ARM_MOTOR_CAL", "ARM_READY"
]

PROFILE_NAMES = ["NONE", "GUTTER_RAMP", "GUTTER_HOLD", "FLIGHT_PARAM"]

EVENT_NAMES = {
    0: "STATE_CHANGE",
    1: "FAULT",
    2: "ABORT_TRIGGERED",
    3: "PROFILE_LOADED",
    4: "CHECKS_GREEN",
    5: "CHECKS_RED",
    6: "BOOT_REPORT",
    7: "ARMED",
    8: "DISARMED",
    9: "LIFTOFF",
    10: "APOGEE",
    11: "LANDING",
    12: "GENERIC_MSG",
    13: "PONG",
    14: "BARO_CALIBRATED",
    15: "MOTOR_ARMED"
}

# Packet sizes
FAST_PKT_SIZE = 38
SLOW_PKT_SIZE = 32
EVENT_PKT_SIZE = 38

# ============================================================================
# Packet Structures
# ============================================================================

def parse_fast_telemetry(data):
    """Parse fast telemetry packet (38 bytes)"""
    if len(data) < FAST_PKT_SIZE:
        return None
    
    fmt = '<BBBBBIBBBB6h4hH'
    unpacked = struct.unpack(fmt, data[:FAST_PKT_SIZE])
    
    return {
        'packet_type': unpacked[0],
        'frame_id': unpacked[1],
        'slot_id': unpacked[2],
        'seq': unpacked[3],
        'flags': unpacked[4],
        'time': unpacked[5],
        'state': unpacked[6],
        'substate': unpacked[7],
        'last_cmd_seq': unpacked[8],
        'last_cmd_status': unpacked[9],
        'accel_x': unpacked[10] / 1000.0,  # g
        'accel_y': unpacked[11] / 1000.0,
        'accel_z': unpacked[12] / 1000.0,
        'gyro_x': unpacked[13] / 100.0,    # dps
        'gyro_y': unpacked[14] / 100.0,
        'gyro_z': unpacked[15] / 100.0,
        'altitude': unpacked[16] / 10.0,   # m
        'vario': unpacked[17] / 100.0,     # m/s
        'pitch': unpacked[18] / 10.0,      # deg
        'roll': unpacked[19] / 10.0,
        'crc': unpacked[20]
    }

def parse_slow_telemetry(data):
    """Parse slow telemetry packet (32 bytes)"""
    if len(data) < SLOW_PKT_SIZE:
        return None
    
    fmt = '<BBBBIiiHBBhhBBHH'
    unpacked = struct.unpack(fmt, data[:SLOW_PKT_SIZE])
    
    return {
        'packet_type': unpacked[0],
        'frame_id': unpacked[1],
        'slot_id': unpacked[2],
        'seq': unpacked[3],
        'time': unpacked[4],
        'latitude': unpacked[5] / 1e7,
        'longitude': unpacked[6] / 1e7,
        'gps_altitude': unpacked[7] / 10.0,
        'gps_lock': unpacked[8],
        'satellites': unpacked[9],
        'pressure': (unpacked[10] / 10.0) + 1000.0,  # mbar
        'temp_baro': unpacked[11] / 10.0,            # C
        'battery_pct': unpacked[12],
        'sd_status': unpacked[13],
        'free_heap': unpacked[14] * 10,              # bytes
        'crc': unpacked[15]
    }

def parse_event(data):
    """Parse event packet (38 bytes)"""
    if len(data) < EVENT_PKT_SIZE:
        return None
    
    fmt = '<BBBBIBBB24sH'
    unpacked = struct.unpack(fmt, data[:EVENT_PKT_SIZE])
    
    return {
        'packet_type': unpacked[0],
        'frame_id': unpacked[1],
        'slot_id': unpacked[2],
        'seq': unpacked[3],
        'time': unpacked[4],
        'event_type': unpacked[5],
        'state': unpacked[6],
        'substate': unpacked[7],
        'payload': unpacked[8],
        'crc': unpacked[9]
    }

# ============================================================================
# Serial Communication Thread
# ============================================================================

class SerialWorker(QObject):
    """Worker thread for serial communication"""
    
    fast_received = pyqtSignal(dict)
    slow_received = pyqtSignal(dict)
    event_received = pyqtSignal(dict)
    raw_received = pyqtSignal(str)
    connected = pyqtSignal(bool)
    error = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.serial = None
        self.running = False
        self.rx_buffer = bytearray()
        
    def connect(self, port, baudrate=115200):
        try:
            self.serial = serial.Serial(port, baudrate, timeout=0.1)
            self.running = True
            self.connected.emit(True)
            return True
        except Exception as e:
            self.error.emit(f"Connection failed: {e}")
            return False
    
    def disconnect(self):
        self.running = False
        if self.serial:
            self.serial.close()
            self.serial = None
        self.connected.emit(False)
    
    def send_command(self, cmd_char):
        """Send a single character command to Arduino"""
        if self.serial and self.serial.is_open:
            self.serial.write(cmd_char.encode())
    
    def run(self):
        """Main receive loop"""
        while self.running:
            if not self.serial or not self.serial.is_open:
                time.sleep(0.1)
                continue
            
            try:
                # Read available data
                data = self.serial.read(256)
                if data:
                    # Forward raw data to console
                    try:
                        text = data.decode('utf-8', errors='replace')
                        self.raw_received.emit(text)
                    except:
                        pass
                    
                    # Add to buffer for packet parsing
                    self.rx_buffer.extend(data)
                    self.process_buffer()
                    
            except Exception as e:
                self.error.emit(f"Read error: {e}")
                time.sleep(0.1)
    
    def process_buffer(self):
        """Process received buffer for complete packets"""
        # Look for packet markers in the Arduino's forwarded data
        # The Arduino sends parsed text, so we mainly use raw_received
        # For binary packets, we'd parse here
        
        # Limit buffer size
        if len(self.rx_buffer) > 1024:
            self.rx_buffer = self.rx_buffer[-512:]

# ============================================================================
# Real-time Plot Widget
# ============================================================================

class RealtimePlot(pg.PlotWidget):
    """Real-time scrolling plot"""
    
    def __init__(self, title="", ylabel="", window_size=500):
        super().__init__()
        
        self.window_size = window_size
        self.data = deque(maxlen=window_size)
        self.timestamps = deque(maxlen=window_size)
        
        self.setTitle(title)
        self.setLabel('left', ylabel)
        self.setLabel('bottom', 'Time', 's')
        self.showGrid(x=True, y=True, alpha=0.3)
        
        self.curve = self.plot(pen=pg.mkPen('c', width=2))
        
    def add_point(self, value, timestamp=None):
        if timestamp is None:
            timestamp = time.time()
        
        self.data.append(value)
        self.timestamps.append(timestamp)
        
        if len(self.data) > 1:
            t0 = self.timestamps[0]
            times = [t - t0 for t in self.timestamps]
            self.curve.setData(times, list(self.data))

class MultiLinePlot(pg.PlotWidget):
    """Plot with multiple lines"""
    
    def __init__(self, title="", ylabel="", lines=3, labels=None, window_size=500):
        super().__init__()
        
        self.window_size = window_size
        self.num_lines = lines
        self.data = [deque(maxlen=window_size) for _ in range(lines)]
        self.timestamps = deque(maxlen=window_size)
        
        self.setTitle(title)
        self.setLabel('left', ylabel)
        self.setLabel('bottom', 'Time', 's')
        self.showGrid(x=True, y=True, alpha=0.3)
        
        colors = ['r', 'g', 'b', 'y', 'c', 'm']
        self.curves = []
        for i in range(lines):
            label = labels[i] if labels else f'Line {i}'
            pen = pg.mkPen(colors[i % len(colors)], width=2)
            curve = self.plot(pen=pen, name=label)
            self.curves.append(curve)
        
        self.addLegend()
    
    def add_points(self, values, timestamp=None):
        if timestamp is None:
            timestamp = time.time()
        
        self.timestamps.append(timestamp)
        for i, v in enumerate(values):
            if i < self.num_lines:
                self.data[i].append(v)
        
        if len(self.timestamps) > 1:
            t0 = self.timestamps[0]
            times = [t - t0 for t in self.timestamps]
            for i, curve in enumerate(self.curves):
                if len(self.data[i]) > 0:
                    curve.setData(times, list(self.data[i]))

# ============================================================================
# Status Indicator Widget
# ============================================================================

class StatusIndicator(QFrame):
    """Colored status indicator with label"""
    
    def __init__(self, label="Status"):
        super().__init__()
        self.setFrameStyle(QFrame.Box | QFrame.Raised)
        self.setLineWidth(2)
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        
        self.label = QLabel(label)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setFont(QFont('Arial', 10, QFont.Bold))
        layout.addWidget(self.label)
        
        self.value_label = QLabel("---")
        self.value_label.setAlignment(Qt.AlignCenter)
        self.value_label.setFont(QFont('Arial', 12))
        layout.addWidget(self.value_label)
        
        self.set_status('unknown')
    
    def set_status(self, status):
        colors = {
            'ok': '#00AA00',
            'warning': '#AAAA00',
            'error': '#AA0000',
            'armed': '#FF6600',
            'flight': '#0066FF',
            'unknown': '#666666'
        }
        color = colors.get(status, colors['unknown'])
        self.setStyleSheet(f"background-color: {color}; border-radius: 5px;")
    
    def set_value(self, value):
        self.value_label.setText(str(value))

# ============================================================================
# Main Dashboard Window
# ============================================================================

class DashboardWindow(QMainWindow):
    """Main application window"""
    
    def __init__(self, port=None):
        super().__init__()
        
        self.setWindowTitle("Flight Computer Ground Station")
        self.setGeometry(100, 100, 1600, 900)
        
        # Serial worker
        self.serial_worker = SerialWorker()
        self.serial_thread = None
        
        # Data storage
        self.last_fast = {}
        self.last_slow = {}
        self.rtt_history = deque(maxlen=100)
        
        # Create UI
        self.init_ui()
        
        # Connect signals
        self.serial_worker.raw_received.connect(self.on_raw_received)
        self.serial_worker.connected.connect(self.on_connection_changed)
        self.serial_worker.error.connect(self.on_error)
        
        # Update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(100)  # 10 Hz update
        
        # Auto-connect if port specified
        if port:
            self.port_combo.setCurrentText(port)
            self.connect_serial()
    
    def init_ui(self):
        """Initialize the user interface"""
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        
        # Left panel - Controls and Status
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_panel.setMaximumWidth(350)
        
        # Connection group
        conn_group = QGroupBox("Connection")
        conn_layout = QGridLayout(conn_group)
        
        self.port_combo = QComboBox()
        self.refresh_ports()
        conn_layout.addWidget(QLabel("Port:"), 0, 0)
        conn_layout.addWidget(self.port_combo, 0, 1)
        
        refresh_btn = QPushButton("🔄")
        refresh_btn.clicked.connect(self.refresh_ports)
        conn_layout.addWidget(refresh_btn, 0, 2)
        
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.connect_serial)
        conn_layout.addWidget(self.connect_btn, 1, 0, 1, 3)
        
        self.status_label = QLabel("Disconnected")
        self.status_label.setAlignment(Qt.AlignCenter)
        conn_layout.addWidget(self.status_label, 2, 0, 1, 3)
        
        left_layout.addWidget(conn_group)
        
        # State indicator
        state_group = QGroupBox("Flight State")
        state_layout = QGridLayout(state_group)
        
        self.state_indicator = StatusIndicator("State")
        state_layout.addWidget(self.state_indicator, 0, 0)
        
        self.substate_indicator = StatusIndicator("Substate")
        state_layout.addWidget(self.substate_indicator, 0, 1)
        
        left_layout.addWidget(state_group)
        
        # Commands group
        cmd_group = QGroupBox("Commands")
        cmd_layout = QGridLayout(cmd_group)
        
        buttons = [
            ("PING", 'P', 0, 0),
            ("Calibrate", 'C', 0, 1),
            ("Profile 1", '1', 1, 0),
            ("Profile 2", '2', 1, 1),
            ("Profile 3", '3', 2, 0),
            ("ARM", 'A', 2, 1),
            ("DISARM", 'D', 3, 0),
            ("TEST", 'T', 3, 1),
            ("LAUNCH", 'L', 4, 0),
            ("ABORT", 'X', 4, 1),
            ("SAFE", 'S', 5, 0),
            ("Reset", 'R', 5, 1),
        ]
        
        for label, cmd, row, col in buttons:
            btn = QPushButton(label)
            btn.clicked.connect(lambda checked, c=cmd: self.send_command(c))
            
            # Color dangerous buttons
            if label in ['LAUNCH', 'ABORT']:
                btn.setStyleSheet("background-color: #AA0000; color: white;")
            elif label == 'ARM':
                btn.setStyleSheet("background-color: #FF6600; color: white;")
            
            cmd_layout.addWidget(btn, row, col)
        
        left_layout.addWidget(cmd_group)
        
        # RTT display
        rtt_group = QGroupBox("Communication")
        rtt_layout = QVBoxLayout(rtt_group)
        
        self.rtt_label = QLabel("RTT: --- ms")
        self.rtt_label.setFont(QFont('Arial', 14, QFont.Bold))
        rtt_layout.addWidget(self.rtt_label)
        
        self.rx_label = QLabel("RX: 0 pkts")
        rtt_layout.addWidget(self.rx_label)
        
        left_layout.addWidget(rtt_group)
        
        # Sensor status
        sensor_group = QGroupBox("Sensors")
        sensor_layout = QGridLayout(sensor_group)
        
        self.altitude_label = QLabel("Alt: --- m")
        self.altitude_label.setFont(QFont('Courier', 11))
        sensor_layout.addWidget(self.altitude_label, 0, 0)
        
        self.vario_label = QLabel("Vario: --- m/s")
        self.vario_label.setFont(QFont('Courier', 11))
        sensor_layout.addWidget(self.vario_label, 0, 1)
        
        self.accel_label = QLabel("Accel: ---g")
        self.accel_label.setFont(QFont('Courier', 11))
        sensor_layout.addWidget(self.accel_label, 1, 0)

        self.temp_label = QLabel("Temp: --- °C")
        self.temp_label.setFont(QFont('Courier', 11))
        sensor_layout.addWidget(self.temp_label, 1, 0)

        self.press_label = QLabel("Press: --- mbar")
        self.press_label.setFont(QFont('Courier', 11))
        sensor_layout.addWidget(self.press_label, 1, 1)
        
        self.gyro_label = QLabel("Gyro: ---°/s")
        self.gyro_label.setFont(QFont('Courier', 11))
        sensor_layout.addWidget(self.gyro_label, 1, 1)
        
        self.pitch_label = QLabel("Pitch: ---°")
        self.pitch_label.setFont(QFont('Courier', 11))
        sensor_layout.addWidget(self.pitch_label, 2, 0)
        
        self.roll_label = QLabel("Roll: ---°")
        self.roll_label.setFont(QFont('Courier', 11))
        sensor_layout.addWidget(self.roll_label, 2, 1)
        
        left_layout.addWidget(sensor_group)
        
        left_layout.addStretch()
        main_layout.addWidget(left_panel)
        
        # Right panel - Plots and Console
        right_panel = QSplitter(Qt.Vertical)
        
        # Plots tab widget
        plot_tabs = QTabWidget()
        
        # Altitude tab
        alt_widget = QWidget()
        alt_layout = QVBoxLayout(alt_widget)
        self.altitude_plot = RealtimePlot("Altitude", "m")
        alt_layout.addWidget(self.altitude_plot)
        plot_tabs.addTab(alt_widget, "Altitude")
        
        # Accelerometer tab
        accel_widget = QWidget()
        accel_layout = QVBoxLayout(accel_widget)
        self.accel_plot = MultiLinePlot("Acceleration", "g", 3, ['X', 'Y', 'Z'])
        accel_layout.addWidget(self.accel_plot)
        plot_tabs.addTab(accel_widget, "Accel")
        
        # Gyroscope tab
        gyro_widget = QWidget()
        gyro_layout = QVBoxLayout(gyro_widget)
        self.gyro_plot = MultiLinePlot("Angular Rate", "°/s", 3, ['X', 'Y', 'Z'])
        gyro_layout.addWidget(self.gyro_plot)
        plot_tabs.addTab(gyro_widget, "Gyro")

        # Temperature tab
        temp_widget = QWidget()
        temp_layout = QVBoxLayout(temp_widget)
        self.temp_plot = RealtimePlot("Temperature", "°C")
        temp_layout.addWidget(self.temp_plot)
        plot_tabs.addTab(temp_widget, "Temp")

        # Pressure tab
        press_widget = QWidget()
        press_layout = QVBoxLayout(press_widget)
        self.press_plot = RealtimePlot("Pressure", "mbar")
        press_layout.addWidget(self.press_plot)
        plot_tabs.addTab(press_widget, "Pressure")
        
        # Orientation tab
        orient_widget = QWidget()
        orient_layout = QVBoxLayout(orient_widget)
        self.orient_plot = MultiLinePlot("Orientation", "°", 2, ['Pitch', 'Roll'])
        orient_layout.addWidget(self.orient_plot)
        plot_tabs.addTab(orient_widget, "Orientation")
        
        right_panel.addWidget(plot_tabs)
        
        # Console
        console_group = QGroupBox("Console")
        console_layout = QVBoxLayout(console_group)
        
        self.console = QTextEdit()
        self.console.setReadOnly(True)
        self.console.setFont(QFont('Courier', 9))
        self.console.setMaximumHeight(250)
        console_layout.addWidget(self.console)
        
        right_panel.addWidget(console_group)
        
        main_layout.addWidget(right_panel, stretch=1)
    
    def refresh_ports(self):
        """Refresh available serial ports"""
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(port.device)
    
    def connect_serial(self):
        """Connect/disconnect serial"""
        if self.serial_worker.serial and self.serial_worker.serial.is_open:
            self.serial_worker.disconnect()
            self.connect_btn.setText("Connect")
        else:
            port = self.port_combo.currentText()
            if port:
                if self.serial_worker.connect(port):
                    self.connect_btn.setText("Disconnect")
                    
                    # Start worker thread
                    self.serial_thread = threading.Thread(target=self.serial_worker.run)
                    self.serial_thread.daemon = True
                    self.serial_thread.start()
    
    def send_command(self, cmd):
        if cmd == 'P':  # assuming 'P' = ping
            self.last_ping_time = time.time()
        """Send command character to Arduino"""
        self.serial_worker.send_command(cmd)
        self.log(f"[TX] Command: {cmd}")
    
    def on_raw_received(self, text):
        """Handle raw serial data"""
        self.console.moveCursor(self.console.textCursor().End)
        self.console.insertPlainText(text)
        
        # Auto-scroll
        scrollbar = self.console.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
        
        # Parse telemetry values from text
        self.parse_arduino_output(text)
    
    def parse_arduino_output(self, text):
        """Parse Arduino's text output to extract telemetry values"""
        lines = text.split('\n')
        for line in lines:
            try:
                # Parse RTT
                if 'RTT:' in line:
                    parts = line.split('RTT:')
                    if len(parts) > 1:
                        rtt_str = parts[1].split()[0].replace('ms', '')
                        rtt = int(rtt_str)
                        self.rtt_history.append(rtt)
                
                # Parse altitude
                if 'Alt:' in line and 'm' in line:
                    parts = line.split('Alt:')
                    if len(parts) > 1:
                        alt_str = parts[1].split('m')[0].strip()
                        try:
                            alt = float(alt_str)
                            self.last_fast['altitude'] = alt
                            self.altitude_plot.add_point(alt)
                        except:
                            pass
                
                # Parse acceleration
                if 'Accel:' in line:
                    # Format: "Accel: X=0.01g Y=0.02g Z=1.00g"
                    try:
                        x = float(line.split('X=')[1].split('g')[0])
                        y = float(line.split('Y=')[1].split('g')[0])
                        z = float(line.split('Z=')[1].split('g')[0])
                        self.last_fast['accel_x'] = x
                        self.last_fast['accel_y'] = y
                        self.last_fast['accel_z'] = z
                        self.accel_plot.add_points([x, y, z])
                    except:
                        pass
                
                # Parse gyro
                if 'Gyro:' in line:
                    try:
                        x = float(line.split('X=')[1].split()[0])
                        y = float(line.split('Y=')[1].split()[0])
                        z = float(line.split('Z=')[1].split()[0])
                        self.last_fast['gyro_x'] = x
                        self.last_fast['gyro_y'] = y
                        self.last_fast['gyro_z'] = z
                        self.gyro_plot.add_points([x, y, z])
                    except:
                        pass
                
                # Parse orientation
                if 'Pitch:' in line and 'Roll:' in line:
                    try:
                        pitch = float(line.split('Pitch:')[1].split('|')[0].strip().replace('°', ''))
                        roll = float(line.split('Roll:')[1].strip().replace('°', ''))
                        self.last_fast['pitch'] = pitch
                        self.last_fast['roll'] = roll
                        self.orient_plot.add_points([pitch, roll])
                    except:
                        pass
                
                # Parse state
                if 'State:' in line and '.' in line:
                    try:
                        state_part = line.split('State:')[1].split('|')[0].strip()
                        if '.' in state_part:
                            state, substate = state_part.split('.')
                            self.last_fast['state_name'] = state.strip()
                            self.last_fast['substate_name'] = substate.strip()
                    except:
                        pass

                # Parse baro temp & pressure
                if 'Temp:' in line and 'Pressure:' in line:
                    try:
                        temp = float(line.split('Temp:')[1].split('C')[0].strip())
                        pressure = float(line.split('Pressure:')[1].split('mbar')[0].strip())
                        self.last_slow['temp_baro'] = temp
                        self.last_slow['pressure'] = pressure
                        self.temp_plot.add_point(temp)
                        self.press_plot.add_point(pressure)
                    except:
                        pass
                
                # Parse vario
                if 'Vario:' in line:
                    try:
                        vario_str = line.split('Vario:')[1].split('m/s')[0].strip()
                        vario = float(vario_str)
                        self.last_fast['vario'] = vario
                    except:
                        pass
                if 'PONG' in line and self.last_ping_time is not None:
                    rtt_ms = int((time.time() - self.last_ping_time) * 1000)
                    self.rtt_history.append(rtt_ms)
                    self.last_ping_time = None
                
            except Exception as e:
                pass  # Ignore parse errors
    
    def on_connection_changed(self, connected):
        """Handle connection state change"""
        if connected:
            self.status_label.setText("Connected")
            self.status_label.setStyleSheet("color: green;")
        else:
            self.status_label.setText("Disconnected")
            self.status_label.setStyleSheet("color: red;")
    
    def on_error(self, error):
        """Handle serial error"""
        self.log(f"[ERROR] {error}")
    
    def log(self, message):
        """Add message to console"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.console.append(f"[{timestamp}] {message}")
    
    def update_display(self):
        """Update display with latest values"""
        # Update RTT
        if self.rtt_history:
            avg_rtt = sum(self.rtt_history) / len(self.rtt_history)
            self.rtt_label.setText(f"RTT: {self.rtt_history[-1]} ms (avg: {avg_rtt:.0f} ms)")
        
        # Update sensor values
        if 'altitude' in self.last_fast:
            self.altitude_label.setText(f"Alt: {self.last_fast['altitude']:.1f} m")
        
        if 'vario' in self.last_fast:
            self.vario_label.setText(f"Vario: {self.last_fast['vario']:.2f} m/s")
        
        if 'accel_x' in self.last_fast:
            ax = self.last_fast['accel_x']
            ay = self.last_fast['accel_y']
            az = self.last_fast['accel_z']
            total = (ax**2 + ay**2 + az**2)**0.5
            self.accel_label.setText(f"Accel: {total:.2f}g")
        
        if 'gyro_x' in self.last_fast:
            gx = self.last_fast['gyro_x']
            gy = self.last_fast['gyro_y']
            gz = self.last_fast['gyro_z']
            total = (gx**2 + gy**2 + gz**2)**0.5
            self.gyro_label.setText(f"Gyro: {total:.1f}°/s")
        
        if 'pitch' in self.last_fast:
            self.pitch_label.setText(f"Pitch: {self.last_fast['pitch']:.1f}°")
        
        if 'roll' in self.last_fast:
            self.roll_label.setText(f"Roll: {self.last_fast['roll']:.1f}°")
        
        if 'temp_baro' in self.last_slow:
            self.temp_label.setText(f"Temp: {self.last_slow['temp_baro']:.1f} °C")
            
        if 'pressure' in self.last_slow:
            self.press_label.setText(f"Press: {self.last_slow['pressure']:.1f} mbar")
        
        # Update state indicators
        if 'state_name' in self.last_fast:
            state = self.last_fast['state_name']
            self.state_indicator.set_value(state)
            
            if state == 'FLIGHT':
                self.state_indicator.set_status('flight')
            elif state == 'ARMED':
                self.state_indicator.set_status('armed')
            elif state in ['ABORT', 'SAFE']:
                self.state_indicator.set_status('error')
            elif state == 'IDLE':
                self.state_indicator.set_status('ok')
            else:
                self.state_indicator.set_status('warning')
        
        if 'substate_name' in self.last_fast:
            self.substate_indicator.set_value(self.last_fast['substate_name'])
    
    def closeEvent(self, event):
        """Clean up on close"""
        self.serial_worker.running = False
        if self.serial_worker.serial:
            self.serial_worker.serial.close()
        event.accept()

# ============================================================================
# Main Entry Point
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description='Flight Computer Ground Station Dashboard')
    parser.add_argument('--port', '-p', type=str, help='Serial port (e.g., COM3 or /dev/ttyUSB0)')
    args = parser.parse_args()
    
    app = QApplication(sys.argv)
    
    # Set dark theme
    app.setStyle('Fusion')
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(53, 53, 53))
    palette.setColor(QPalette.WindowText, Qt.white)
    palette.setColor(QPalette.Base, QColor(25, 25, 25))
    palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
    palette.setColor(QPalette.ToolTipBase, Qt.white)
    palette.setColor(QPalette.ToolTipText, Qt.white)
    palette.setColor(QPalette.Text, Qt.white)
    palette.setColor(QPalette.Button, QColor(53, 53, 53))
    palette.setColor(QPalette.ButtonText, Qt.white)
    palette.setColor(QPalette.BrightText, Qt.red)
    palette.setColor(QPalette.Link, QColor(42, 130, 218))
    palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    palette.setColor(QPalette.HighlightedText, Qt.black)
    app.setPalette(palette)
    
    window = DashboardWindow(args.port)
    window.show()
    
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()