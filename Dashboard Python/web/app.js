/**
 * @file app.js
 * @brief Ground Station Dashboard Client Application
 * @author Tomás Teixeira
 * @date 2025
 * @version 3.0
 *
 * @details
 * Client-side JavaScript for the Magalhães Flight Computer Ground Station Dashboard.
 * Provides real-time telemetry visualization, 3D rocket orientation display,
 * GPS mapping, command interface, and static thrust testing functionality.
 *
 * ## Features
 * - Real-time telemetry charts (altitude, velocity, orientation, etc.)
 * - WebSocket communication with FastAPI backend
 * - 3D rocket visualization using Three.js
 * - GPS tracking with Leaflet maps
 * - Flight state machine status display
 * - Static thrust test controls and data export
 * - Sensor boot status monitoring
 *
 * ## Architecture
 * ```
 * ┌─────────────────────────────────────────────────────────────┐
 * │                    Dashboard Architecture                    │
 * ├─────────────────────────────────────────────────────────────┤
 * │  WebSocket ──► Message Router ──► Update Functions          │
 * │                    │                                         │
 * │                    ├── fast    ──► updateFast()             │
 * │                    ├── slow    ──► updateSlow()             │
 * │                    ├── gs_stats──► updateStats()            │
 * │                    ├── event   ──► handleStaticTestEvent()  │
 * │                    └── pong    ──► RTT Display              │
 * └─────────────────────────────────────────────────────────────┘
 * ```
 *
 * ## Dependencies
 * - Chart.js: Real-time data charting
 * - Three.js: 3D rocket visualization
 * - Leaflet: GPS map display
 *
 * @see server.py for backend WebSocket server
 * @see index.html for HTML structure
 * @ingroup Dashboard
 */

// =============================================================================
// DOM Elements
// =============================================================================

/** @type {NodeListOf<Element>} All page elements for navigation */
const pages = document.querySelectorAll('.page');

/** @type {NodeListOf<Element>} Navigation menu items */
const navItems = document.querySelectorAll('.nav-item');

/** @type {HTMLElement} Sidebar element */
const sidebar = document.getElementById('sidebar');

/** @type {HTMLElement} Menu toggle button */
const menuToggle = document.getElementById('menuToggle');

// Sidebar toggle event listener
menuToggle.addEventListener('click', () => {
  sidebar.classList.toggle('collapsed');
});

// Navigation click handlers - switch between pages
navItems.forEach((item) => {
  item.addEventListener('click', () => {
    navItems.forEach((btn) => btn.classList.remove('active'));
    item.classList.add('active');
    const page = item.getAttribute('data-page');
    pages.forEach((p) => p.classList.remove('active'));
    document.getElementById(`page-${page}`).classList.add('active');
  });
});

// =============================================================================
// Global State Variables
// =============================================================================

/**
 * @type {Object.<string, Chart>}
 * @description Collection of Chart.js instances for telemetry visualization
 */
const charts = {};

/**
 * @type {number}
 * @description Maximum number of data points to display on charts (rolling window)
 */
const maxPoints = 200;

/**
 * @type {number|null}
 * @description Current round-trip time in milliseconds (null if not measured)
 */
let rttMs = null;

/**
 * @type {WebSocket|null}
 * @description WebSocket connection to the backend server
 */
let ws = null;

// =============================================================================
// Rate Calculation Variables
// =============================================================================

/** @type {number} Last total received packet count for rate calculation */
let lastRxTotal = 0;

/** @type {number} Timestamp of last rate calculation */
let lastRateTime = Date.now();

/** @type {number[]} Rolling history of data rates for averaging */
let rateHistory = [];

/** @type {number} Number of rate samples to keep for rolling average (~2 seconds) */
const RATE_HISTORY_SIZE = 4;

// =============================================================================
// Boot Status Tracking
// =============================================================================

/**
 * @type {Object}
 * @description Sensor initialization status from boot report
 * @property {string} imu - IMU sensor status ('ok', 'fail', 'unknown')
 * @property {string} mag - Magnetometer status
 * @property {string} baro - Barometer status
 * @property {string} bno - BNO055 orientation sensor status
 * @property {string} gps - GPS module status
 */
const bootStatus = {
  imu: 'unknown',
  mag: 'unknown',
  baro: 'unknown',
  bno: 'unknown',
  gps: 'unknown'
};

// =============================================================================
// Motor/PWM State
// =============================================================================

/** @type {number} Current motor throttle percentage (0-100) */
let currentThrottle = 0;

// =============================================================================
// Static Thrust Test State
// =============================================================================

/**
 * @type {Array<{time_ms: number, pwm: number, thrust: number}>}
 * @description Collected test data points [{time_ms, pwm, thrust}, ...]
 */
let testData = [];

/** @type {number} Maximum thrust recorded during current test (Newtons) */
let testMaxThrust = 0;

/** @type {number} PWM percentage at which max thrust occurred */
let testMaxPWM = 0;

/** @type {boolean} Whether a static thrust test is currently running */
let testRunning = false;

/** @type {boolean} Whether the motor has been calibrated */
let motorCalibrated = false;

/** @type {number} Timestamp when current test started (for client-side time tracking) */
let testStartTime = 0;

// =============================================================================
// Flight State Machine Constants
// =============================================================================

/**
 * @constant {string[]}
 * @description FSM state names indexed by state ID
 */
const STATE_NAMES = ["BOOT", "IDLE", "CONFIGED", "ARMED", "TEST_STAND", "FLIGHT", "ABORT", "SAFE"];

/**
 * @constant {string[]}
 * @description FSM substate names indexed by substate ID
 */
const SUBSTATE_NAMES = [
  "NONE", "TS_SENSOR_CHECK", "TS_THROTTLE_RAMP",
  "FL_IGNITION", "FL_LIFTOFF_DETECT", "FL_ASCENT", "FL_COAST",
  "FL_DESCENT_BRAKE", "FL_LANDING_FLARE", "FL_TOUCHDOWN", "FL_RECOVERY",
  "ARM_MOTOR_INIT", "ARM_MOTOR_CAL", "ARM_READY"
];

/**
 * @constant {Object.<number, string>}
 * @description Event type ID to name mapping
 */
const EVENT_NAMES = {
  0: "STATE_CHANGE", 1: "FAULT", 2: "ABORT", 3: "PROFILE_LOADED",
  4: "CHECKS_GREEN", 5: "CHECKS_RED", 6: "BOOT_REPORT", 7: "ARMED",
  8: "DISARMED", 9: "LIFTOFF", 10: "APOGEE", 11: "LANDING",
  12: "GENERIC", 13: "PONG", 14: "BARO_CAL", 15: "MOTOR_ARMED",
  16: "MOTOR_CAL_STARTED", 17: "MOTOR_CAL_PHASE2", 18: "MOTOR_CALIBRATED",
  19: "STATIC_TEST_STARTED", 20: "STATIC_TEST_PROGRESS", 21: "STATIC_TEST_COMPLETE", 22: "STATIC_TEST_FAILED"
};

// =============================================================================
// Chart Creation Functions
// =============================================================================

/**
 * Creates a Chart.js line chart with the specified datasets
 *
 * @param {string} id - Canvas element ID for the chart
 * @param {Array<{label: string, color: string}>} datasets - Dataset configurations
 * @returns {Chart} Configured Chart.js instance
 *
 * @example
 * // Create a single-line altitude chart
 * const altChart = createLineChart('altChart', [{ label: 'Alt', color: '#38bdf8' }]);
 *
 * @example
 * // Create a multi-line gyroscope chart
 * const gyroChart = createLineChart('gyroChart', [
 *   { label: 'X', color: '#38bdf8' },
 *   { label: 'Y', color: '#22c55e' },
 *   { label: 'Z', color: '#a855f7' }
 * ]);
 */
function createLineChart(id, datasets) {
  const ctx = document.getElementById(id).getContext('2d');
  return new Chart(ctx, {
    type: 'line',
    data: {
      labels: [],
      datasets: datasets.map((ds) => ({
        label: ds.label,
        data: [],
        borderColor: ds.color,
        borderWidth: 2,
        pointRadius: 0,
        tension: 0.2
      }))
    },
    options: {
      animation: false,
      responsive: true,
      plugins: {
        legend: {
          display: datasets.length > 1
        }
      },
      scales: {
        x: { display: false },
        y: { grid: { color: '#253041' }, ticks: { color: '#9ca3af' } }
      }
    }
  });
}

// Initialize telemetry charts
charts.alt = createLineChart('altChart', [{ label: 'Alt', color: '#38bdf8' }]);
charts.vel = createLineChart('velChart', [{ label: 'Vel', color: '#22c55e' }]);
charts.temp = createLineChart('tempChart', [{ label: 'Temp', color: '#f97316' }]);
charts.press = createLineChart('pressChart', [{ label: 'Press', color: '#a855f7' }]);
charts.gyro = createLineChart('gyroChart', [
  { label: 'X', color: '#38bdf8' },
  { label: 'Y', color: '#22c55e' },
  { label: 'Z', color: '#a855f7' }
]);
charts.accel = createLineChart('accelChart', [
  { label: 'X', color: '#38bdf8' },
  { label: 'Y', color: '#22c55e' },
  { label: 'Z', color: '#a855f7' }
]);
charts.orient = createLineChart('orientChart', [
  { label: 'Pitch', color: '#38bdf8' },
  { label: 'Roll', color: '#22c55e' },
  { label: 'Yaw', color: '#f59e0b' }
]);

/**
 * Creates a scatter/line chart for thrust vs PWM visualization
 *
 * @returns {Chart|null} Chart.js instance or null if canvas not found
 *
 * @description
 * Creates a specialized chart for static thrust testing that plots
 * thrust (Newtons) against PWM percentage. Uses scatter plot with
 * connected lines for better visualization of the thrust curve.
 */
function createThrustChart() {
  const ctx = document.getElementById('thrustChart');
  if (!ctx) return null;
  return new Chart(ctx.getContext('2d'), {
    type: 'scatter',
    data: {
      datasets: [{
        label: 'Thrust vs PWM',
        data: [],
        backgroundColor: '#22c55e',
        borderColor: '#22c55e',
        pointRadius: 4,
        showLine: true,
        tension: 0.2
      }]
    },
    options: {
      animation: false,
      responsive: true,
      plugins: {
        legend: { display: false }
      },
      scales: {
        x: {
          title: { display: true, text: 'PWM (%)', color: '#9ca3af' },
          min: 0,
          max: 100,
          grid: { color: '#253041' },
          ticks: { color: '#9ca3af' }
        },
        y: {
          title: { display: true, text: 'Thrust (N)', color: '#9ca3af' },
          min: 0,
          grid: { color: '#253041' },
          ticks: { color: '#9ca3af' }
        }
      }
    }
  });
}
charts.thrust = createThrustChart();

/** @type {string} Current chart view mode: 'thrust_pwm', 'thrust_time', 'pwm_time' */
let chartViewMode = 'thrust_pwm';

/**
 * Updates the thrust chart to show different data relationships
 *
 * @param {string} mode - Chart mode: 'thrust_pwm', 'thrust_time', or 'pwm_time'
 */
function setChartView(mode) {
  chartViewMode = mode;
  if (!charts.thrust) return;

  // Update axis labels based on mode
  const xAxis = charts.thrust.options.scales.x;
  const yAxis = charts.thrust.options.scales.y;

  switch (mode) {
    case 'thrust_pwm':
      xAxis.title.text = 'PWM (%)';
      xAxis.min = 0;
      xAxis.max = 100;
      yAxis.title.text = 'Thrust (N)';
      yAxis.min = 0;
      charts.thrust.data.datasets[0].label = 'Thrust vs PWM';
      break;
    case 'thrust_time':
      xAxis.title.text = 'Time (s)';
      xAxis.min = 0;
      xAxis.max = undefined;  // Auto scale
      yAxis.title.text = 'Thrust (N)';
      yAxis.min = 0;
      charts.thrust.data.datasets[0].label = 'Thrust vs Time';
      break;
    case 'pwm_time':
      xAxis.title.text = 'Time (s)';
      xAxis.min = 0;
      xAxis.max = undefined;  // Auto scale
      yAxis.title.text = 'PWM (%)';
      yAxis.min = 0;
      yAxis.max = 100;
      charts.thrust.data.datasets[0].label = 'PWM vs Time';
      break;
  }

  // Rebuild chart data from testData based on mode
  const chartData = testData.map(point => {
    switch (mode) {
      case 'thrust_pwm':
        return { x: point.pwm, y: point.thrust };
      case 'thrust_time':
        return { x: point.time_ms / 1000, y: point.thrust };
      case 'pwm_time':
        return { x: point.time_ms / 1000, y: point.pwm };
    }
  });

  charts.thrust.data.datasets[0].data = chartData;
  charts.thrust.update();

  // Update button states
  document.querySelectorAll('.chart-view-btn').forEach(btn => {
    btn.classList.toggle('active', btn.dataset.view === mode);
  });

  appendLog(`[TEST] Chart view: ${mode}`);
}

/**
 * Gets chart point based on current view mode
 * @param {Object} dataPoint - {time_ms, pwm, thrust}
 * @returns {Object} - {x, y} for chart
 */
function getChartPointForMode(dataPoint) {
  switch (chartViewMode) {
    case 'thrust_pwm':
      return { x: dataPoint.pwm, y: dataPoint.thrust };
    case 'thrust_time':
      return { x: dataPoint.time_ms / 1000, y: dataPoint.thrust };
    case 'pwm_time':
      return { x: dataPoint.time_ms / 1000, y: dataPoint.pwm };
    default:
      return { x: dataPoint.pwm, y: dataPoint.thrust };
  }
}

// Setup chart view button handlers - use event delegation for reliability
document.addEventListener('click', (e) => {
  if (e.target.classList.contains('chart-view-btn')) {
    const mode = e.target.dataset.view;
    if (mode) {
      setChartView(mode);
    }
  }
});

/**
 * Pushes new data points to a chart and maintains rolling window
 *
 * @param {Chart} chart - Chart.js instance to update
 * @param {number[]} values - Array of values (one per dataset)
 *
 * @description
 * Adds new data points to all datasets in the chart simultaneously.
 * Removes oldest points when maxPoints limit is reached to maintain
 * a rolling window display. Uses 'none' animation for real-time performance.
 *
 * @example
 * // Update gyroscope chart with X, Y, Z values
 * pushData(charts.gyro, [gyro_x, gyro_y, gyro_z]);
 */
function pushData(chart, values) {
  const label = '';
  chart.data.labels.push(label);
  chart.data.datasets.forEach((ds, i) => {
    ds.data.push(values[i]);
  });
  if (chart.data.labels.length > maxPoints) {
    chart.data.labels.shift();
    chart.data.datasets.forEach((ds) => ds.data.shift());
  }
  chart.update('none');
}

// =============================================================================
// WebSocket Communication
// =============================================================================

/**
 * Establishes WebSocket connection to the backend server
 *
 * @description
 * Connects to the WebSocket endpoint at /ws on the current host.
 * Handles incoming messages by routing to appropriate update functions
 * based on message type. Automatically reconnects on connection close.
 *
 * ## Message Types
 * | Type | Handler | Description |
 * |------|---------|-------------|
 * | fast | updateFast() | High-rate telemetry (8 Hz) |
 * | slow | updateSlow() | Low-rate telemetry (1 Hz) |
 * | gs_stats | updateStats() | Ground station statistics |
 * | gs_status | updateStatus() | TDMA sync status |
 * | event | handleStaticTestEvent() | Flight events |
 * | pong | - | RTT measurement response |
 * | log | appendLog() | Server log messages |
 */
function connectWebSocket() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.addEventListener('open', () => {
    // Reset rate tracking on connect
    lastRxTotal = 0;
    lastRateTime = Date.now();
    rateHistory = [];
  });

  ws.addEventListener('close', () => {
    setTimeout(connectWebSocket, 1000);
  });

  ws.addEventListener('message', (event) => {
    const pkt = JSON.parse(event.data);
    if (pkt.type === 'fast') {
      updateFast(pkt);
    } else if (pkt.type === 'slow') {
      updateSlow(pkt);
    } else if (pkt.type === 'gs_stats') {
      updateStats(pkt);
    } else if (pkt.type === 'gs_status') {
      updateStatus(pkt);
    } else if (pkt.type === 'pong') {
      rttMs = pkt.rtt_ms;
      document.getElementById('rttValue').textContent = `${pkt.rtt_ms} ms`;
      appendLog(`[PONG] RTT=${pkt.rtt_ms}ms`);
    } else if (pkt.type === 'event') {
      const name = EVENT_NAMES[pkt.event_type] || `EVT_${pkt.event_type}`;
      console.log('[DEBUG] Received event:', pkt.event_type, name, 'payload length:', pkt.payload ? pkt.payload.length : 0);
      appendLog(`[EVENT] ${name}`);
      // Parse boot report from CHECKS_GREEN (4) or CHECKS_RED (5)
      if (pkt.event_type === 4 && pkt.payload) {
        parseBootReport(pkt.payload, true);  // All checks passed
      } else if (pkt.event_type === 5 && pkt.payload) {
        parseBootReport(pkt.payload, false); // Some checks failed
      }
      // Handle static test events
      handleStaticTestEvent(pkt.event_type, pkt.payload);
      // Handle motor calibration events
      handleMotorCalEvent(pkt.event_type, pkt.payload);
    } else if (pkt.type === 'log') {
      appendLog(`[GS] ${pkt.message}`);
    }
  });
}

connectWebSocket();

// =============================================================================
// UI Update Functions
// =============================================================================

/**
 * Updates the connection status indicator
 *
 * @param {boolean} connected - Whether the serial connection is active
 */
function setStatus(connected) {
  const el = document.getElementById('statusValue');
  el.textContent = connected ? 'CONNECTED' : 'DISCONNECTED';
  el.style.color = connected ? '#22c55e' : '#ef4444';
}

/**
 * Updates dashboard with fast telemetry data (8 Hz)
 *
 * @param {Object} pkt - Fast telemetry packet
 * @param {number} pkt.altitude - Altitude in meters
 * @param {number} pkt.vario - Vertical velocity in m/s
 * @param {number} pkt.accel_x - X-axis acceleration in g
 * @param {number} pkt.accel_y - Y-axis acceleration in g
 * @param {number} pkt.accel_z - Z-axis acceleration in g
 * @param {number} pkt.gyro_x - X-axis angular rate in deg/s
 * @param {number} pkt.gyro_y - Y-axis angular rate in deg/s
 * @param {number} pkt.gyro_z - Z-axis angular rate in deg/s
 * @param {number} pkt.pitch - Pitch angle in degrees
 * @param {number} pkt.roll - Roll angle in degrees
 * @param {number} [pkt.yaw] - Yaw angle in degrees (optional)
 * @param {number} pkt.state - FSM state ID
 * @param {number} pkt.substate - FSM substate ID
 *
 * @description
 * Updates all fast-rate telemetry displays including:
 * - Altitude and velocity values
 * - Acceleration and gyroscope charts
 * - Orientation chart and 3D rocket model
 * - State machine status display
 */
function updateFast(pkt) {
  document.getElementById('altValue').textContent = `${pkt.altitude.toFixed(1)} m`;
  document.getElementById('velValue').textContent = `${pkt.vario.toFixed(2)} m/s`;

  pushData(charts.alt, [pkt.altitude]);
  pushData(charts.vel, [pkt.vario]);
  pushData(charts.accel, [pkt.accel_x, pkt.accel_y, pkt.accel_z]);
  pushData(charts.gyro, [pkt.gyro_x, pkt.gyro_y, pkt.gyro_z]);

  const yaw = pkt.yaw || 0;
  pushData(charts.orient, [pkt.pitch, pkt.roll, yaw]);

  const state = STATE_NAMES[pkt.state] || `S${pkt.state}`;
  const substate = SUBSTATE_NAMES[pkt.substate] || `SS${pkt.substate}`;
  document.getElementById('stateValue').textContent = `${state} / ${substate}`;

  document.getElementById('accelX').textContent = `${pkt.accel_x.toFixed(2)} g`;
  document.getElementById('accelY').textContent = `${pkt.accel_y.toFixed(2)} g`;
  document.getElementById('accelZ').textContent = `${pkt.accel_z.toFixed(2)} g`;

  document.getElementById('gyroX').textContent = `${pkt.gyro_x.toFixed(2)} deg/s`;
  document.getElementById('gyroY').textContent = `${pkt.gyro_y.toFixed(2)} deg/s`;
  document.getElementById('gyroZ').textContent = `${pkt.gyro_z.toFixed(2)} deg/s`;

  document.getElementById('pitchValue').textContent = `${pkt.pitch.toFixed(1)} deg`;
  document.getElementById('rollValue').textContent = `${pkt.roll.toFixed(1)} deg`;
  document.getElementById('yawValue').textContent = `${yaw.toFixed(1)} deg`;

  document.getElementById('flightPitch').textContent = `${pkt.pitch.toFixed(1)} deg`;
  document.getElementById('flightRoll').textContent = `${pkt.roll.toFixed(1)} deg`;
  document.getElementById('flightYaw').textContent = `${yaw.toFixed(1)} deg`;

  if (rocket) {
    // Convert degrees to radians for Three.js
    targetRotation.x = pkt.roll * Math.PI / 180;
    targetRotation.z = pkt.pitch * Math.PI / 180;
    targetRotation.y = yaw * Math.PI / 180;
  }
}

/**
 * Updates dashboard with slow telemetry data (1 Hz)
 *
 * @param {Object} pkt - Slow telemetry packet
 * @param {number} pkt.temperature - Temperature in Celsius
 * @param {number} pkt.pressure - Pressure in mbar
 * @param {number} pkt.battery - Battery percentage
 * @param {number} pkt.satellites - Number of GPS satellites
 * @param {number} pkt.latitude - GPS latitude in degrees
 * @param {number} pkt.longitude - GPS longitude in degrees
 * @param {number} pkt.gps_altitude - GPS altitude in meters
 * @param {number} pkt.gps_lock - GPS lock type (0=none, 2=2D, 3=3D)
 *
 * @description
 * Updates slow-rate telemetry displays including:
 * - Temperature and pressure values/charts
 * - Battery level
 * - GPS information and map position
 */
function updateSlow(pkt) {
  document.getElementById('tempValue').textContent = `${pkt.temperature.toFixed(1)} C`;
  document.getElementById('pressValue').textContent = `${pkt.pressure.toFixed(1)} mbar`;
  document.getElementById('batteryValue').textContent = `${pkt.battery} %`;
  document.getElementById('satValue').textContent = `${pkt.satellites}`;

  pushData(charts.temp, [pkt.temperature]);
  pushData(charts.press, [pkt.pressure]);

  document.getElementById('gpsLat').textContent = pkt.latitude.toFixed(6);
  document.getElementById('gpsLon').textContent = pkt.longitude.toFixed(6);
  document.getElementById('gpsAlt').textContent = `${pkt.gps_altitude.toFixed(1)} m`;
  document.getElementById('gpsLock').textContent = pkt.gps_lock;
  updateMap(pkt.latitude, pkt.longitude, pkt.gps_lock >= 2);
}

/**
 * Updates ground station statistics display
 *
 * @param {Object} pkt - Statistics packet
 * @param {number} pkt.rx_fast - Fast packets received count
 * @param {number} pkt.rx_slow - Slow packets received count
 * @param {number} pkt.rx_event - Event packets received count
 * @param {number} pkt.tx_cmd - Command packets transmitted count
 * @param {number} pkt.tx_sync - Sync packets transmitted count
 * @param {number} pkt.crc_errors - CRC error count
 * @param {number} pkt.ack_ok - Successful acknowledgments count
 * @param {number} pkt.ack_timeout - Acknowledgment timeout count
 *
 * @description
 * Updates communication statistics display and calculates
 * rolling average data rate in kbps. Signal icon color indicates
 * link quality (green > 0.5 kbps, red otherwise).
 */
function updateStats(pkt) {
  const rxTotal = pkt.rx_fast + pkt.rx_slow + pkt.rx_event;
  const txTotal = pkt.tx_cmd + pkt.tx_sync;
  document.getElementById('rxValue').textContent = rxTotal;
  document.getElementById('txValue').textContent = txTotal;
  document.getElementById('crcValue').textContent = pkt.crc_errors;
  document.getElementById('ackOkValue').textContent = pkt.ack_ok;
  document.getElementById('ackTimeoutValue').textContent = pkt.ack_timeout;

  // Calculate actual data rate with rolling average
  const now = Date.now();
  const dt = (now - lastRateTime) / 1000.0;  // seconds
  if (dt >= 0.5) {
    const delta = rxTotal - lastRxTotal;
    const bytesPerPkt = 32;
    const instantRate = Math.max(0, (delta * bytesPerPkt * 8) / (dt * 1000.0));  // kbps
    console.log(`Rate: delta=${delta}, dt=${dt.toFixed(2)}, instant=${instantRate.toFixed(2)}, history=${rateHistory.length}`);
    rateHistory.push(instantRate);
    if (rateHistory.length > RATE_HISTORY_SIZE) {
      rateHistory.shift();
    }
    lastRxTotal = rxTotal;
    lastRateTime = now;
  }
  const kbps = rateHistory.length > 0 ? rateHistory.reduce((a, b) => a + b, 0) / rateHistory.length : 0;
  const rttText = rttMs ? `${rttMs} ms` : '-- ms';
  document.getElementById('signalValue').textContent = `${kbps.toFixed(1)} kb/s | ${rttText}`;
  const signal_icon = document.getElementById('signalIcon');
  if (kbps > 0.5) {
    signal_icon.style.color = '#22c55e';
  } else {
    signal_icon.style.color = '#ef4444';
  }
}

/**
 * Updates TDMA synchronization status display
 *
 * @param {Object} pkt - Status packet
 * @param {boolean} pkt.synced - Whether TDMA is synchronized
 */
function updateStatus(pkt) {
  document.getElementById('syncValue').textContent = pkt.synced ? 'SYNC' : 'UNSYNC';
}

/**
 * Appends a message to the console log display
 *
 * @param {string} text - Message text to append
 *
 * @description
 * Adds timestamped message to the console log pre element
 * and auto-scrolls to show the latest entry.
 */
function appendLog(text) {
  const log = document.getElementById('consoleLog');
  log.textContent += `${text}\n`;
  log.scrollTop = log.scrollHeight;
}

// =============================================================================
// Boot Status Functions
// =============================================================================

/**
 * Updates a sensor's boot status indicator
 *
 * @param {string} sensor - Sensor name ('imu', 'mag', 'baro', 'bno', 'gps')
 * @param {boolean} status - true if sensor initialized OK, false if failed
 *
 * @description
 * Updates both the bootStatus object and the corresponding DOM element
 * with visual indicator (checkmark for OK, X for failure).
 */
function updateBootStatus(sensor, status) {
  bootStatus[sensor] = status ? 'ok' : 'fail';
  const el = document.getElementById(`boot-${sensor}`);
  if (el) {
    el.className = `sensor-status ${bootStatus[sensor]}`;
    el.querySelector('.indicator').textContent = status ? '✓' : '✗';
  }
}

/**
 * Parses boot report payload and updates sensor status indicators
 *
 * @param {string} payloadHex - Hex-encoded boot report payload
 * @param {boolean} checksOk - Whether all critical checks passed
 *
 * @description
 * Boot report payload format:
 * - Bytes 0: total_checks
 * - Byte 1: passed count
 * - Byte 2: failed count
 * - Byte 3: critical failures
 * - Bytes 4-7: boot_time_ms (uint32)
 * - Bytes 8+: error strings (8 x 16 bytes)
 * - Byte 136: error_count
 *
 * Parses error strings to identify which sensors failed initialization.
 */
function parseBootReport(payloadHex, checksOk) {
  // Boot report payload format:
  // Bytes 0: total_checks, 1: passed, 2: failed, 3: critical
  // Bytes 4-7: boot_time_ms (uint32)
  // Bytes 8+: error strings (8 x 16 bytes)
  // Byte 136: error_count
  if (payloadHex.length < 10) return;

  const total = parseInt(payloadHex.substring(0, 2), 16);
  const passed = parseInt(payloadHex.substring(2, 4), 16);
  const failed = parseInt(payloadHex.substring(4, 6), 16);
  const critical = parseInt(payloadHex.substring(6, 8), 16);

  // Parse error strings to determine which sensors failed
  const errorCount = payloadHex.length >= 274 ? parseInt(payloadHex.substring(272, 274), 16) : 0;

  // Always parse error messages - even CHECKS_GREEN can have non-critical failures
  const errors = [];
  for (let i = 0; i < errorCount && i < 8; i++) {
    const start = 16 + i * 32; // Each error is 16 bytes = 32 hex chars
    if (start + 32 <= payloadHex.length) {
      const errHex = payloadHex.substring(start, start + 32);
      let errStr = '';
      for (let j = 0; j < 32; j += 2) {
        const charCode = parseInt(errHex.substring(j, j + 2), 16);
        if (charCode > 0 && charCode < 128) errStr += String.fromCharCode(charCode);
      }
      if (errStr) errors.push(errStr.trim());
    }
  }

  // Update status based on errors found (if no error for sensor, it's OK)
  updateBootStatus('imu', !errors.some(e => e.includes('IMU')));
  updateBootStatus('mag', !errors.some(e => e.includes('MAG')));
  updateBootStatus('baro', !errors.some(e => e.includes('BARO')));
  updateBootStatus('bno', !errors.some(e => e.includes('BNO')));
  updateBootStatus('gps', !errors.some(e => e.includes('GPS')));

  if (errors.length > 0) {
    appendLog(`[BOOT] Errors: ${errors.join(', ')}`);
  }
  appendLog(`[BOOT] ${passed}/${total} checks passed, ${critical} critical, boot ${checksOk ? 'OK' : 'FAILED'}`);
}

// =============================================================================
// PWM Gauge Functions
// =============================================================================

/**
 * Updates the dashboard PWM gauge display
 *
 * @param {number} percent - Throttle percentage (0-100)
 *
 * @description
 * Updates the SVG arc gauge with stroke-dashoffset animation.
 * Color coding: green (<30%), yellow (30-70%), red (>70%).
 */
function updatePWMGauge(percent) {
  currentThrottle = Math.max(0, Math.min(100, percent));
  const valueEl = document.getElementById('pwmValue');
  const fillEl = document.getElementById('pwmGaugeFill');
  if (valueEl) valueEl.textContent = `${currentThrottle}%`;
  if (fillEl) {
    // Arc length: 157 is full arc (semicircle)
    const arcLength = 157;
    const offset = arcLength - (arcLength * currentThrottle / 100);
    // Use setAttribute for SVG elements
    fillEl.setAttribute('stroke-dashoffset', offset);
    // Color based on throttle: green->yellow->red
    if (currentThrottle < 30) {
      fillEl.setAttribute('stroke', '#22c55e');
    } else if (currentThrottle < 70) {
      fillEl.setAttribute('stroke', '#f59e0b');
    } else {
      fillEl.setAttribute('stroke', '#ef4444');
    }
  }
}

// =============================================================================
// Static Thrust Test Event Handlers
// =============================================================================

/**
 * Handles static thrust test events from the flight computer
 *
 * @param {number} eventType - Event type ID (19-22 for test events)
 * @param {string} payload - Hex-encoded event payload
 *
 * @description
 * Processes static thrust test events:
 * - 19: EVT_STATIC_TEST_STARTED - Initialize test state
 * - 20: EVT_STATIC_TEST_PROGRESS - Update with new data point
 * - 21: EVT_STATIC_TEST_COMPLETE - Finalize test
 * - 22: EVT_STATIC_TEST_FAILED - Handle test failure
 *
 * Progress payload format:
 * - Byte 0: pwm_percent (uint8)
 * - Bytes 1-4: thrust_n (float32, little-endian)
 */
function handleStaticTestEvent(eventType, payload) {
  const statusEl = document.getElementById('staticTestStatus');
  const testStatusEl = document.getElementById('testStatus');
  const testStateEl = document.getElementById('testState');

  switch (eventType) {
    case 19: // EVT_STATIC_TEST_STARTED
      console.log('[DEBUG] Static test STARTED event received');
      testRunning = true;
      testStartTime = Date.now();  // Track when test started for time calculation
      testData = [];
      testMaxThrust = 0;
      testMaxPWM = 0;
      if (statusEl) {
        statusEl.textContent = 'Test running...';
        statusEl.style.color = '#f59e0b';
      }
      if (testStatusEl) {
        testStatusEl.textContent = 'Test in progress...';
        testStatusEl.style.color = '#f59e0b';
      }
      if (testStateEl) testStateEl.textContent = 'RUNNING';
      // Clear chart
      if (charts.thrust) {
        charts.thrust.data.datasets[0].data = [];
        charts.thrust.update('none');
      }
      break;

    case 20: // EVT_STATIC_TEST_PROGRESS
      // Payload: byte 0 = pwm_percent, bytes 1-4 = thrust_n (float, little-endian)
      console.log('[DEBUG] Progress event payload:', payload, 'length:', payload ? payload.length : 0);
      if (payload && payload.length >= 10) {  // 1 + 4 bytes = 5 bytes = 10 hex chars
        const pwmPercent = parseInt(payload.substring(0, 2), 16);

        // Parse float from bytes 1-4 (little-endian IEEE 754)
        const floatBytes = new Uint8Array(4);
        for (let i = 0; i < 4; i++) {
          floatBytes[i] = parseInt(payload.substring(2 + i*2, 4 + i*2), 16);
        }
        const thrustN = new Float32Array(floatBytes.buffer)[0];

        // Calculate time since test started (client-side)
        const timestampMs = Date.now() - testStartTime;

        console.log('[DEBUG] Parsed: Time=', timestampMs, 'ms PWM=', pwmPercent, '% Thrust=', thrustN, 'N');

        // Update displays
        updateAllPWMGauges(pwmPercent);
        const currentPWMEl = document.getElementById('currentPWM');
        const currentThrustEl = document.getElementById('currentThrust');
        const currentTimeEl = document.getElementById('currentTime');
        if (currentPWMEl) currentPWMEl.textContent = `${pwmPercent}%`;
        if (currentThrustEl) currentThrustEl.textContent = `${thrustN.toFixed(2)} N`;
        if (currentTimeEl) currentTimeEl.textContent = `${(timestampMs / 1000).toFixed(1)} s`;

        if (statusEl) {
          statusEl.textContent = `Test: ${(timestampMs/1000).toFixed(1)}s | ${pwmPercent}% | ${thrustN.toFixed(1)}N`;
          statusEl.style.color = '#38bdf8';
        }

        // Add data point with time
        const dataPoint = { time_ms: timestampMs, pwm: pwmPercent, thrust: thrustN };
        testData.push(dataPoint);

        // Track max
        if (thrustN > testMaxThrust) {
          testMaxThrust = thrustN;
          testMaxPWM = pwmPercent;
        }

        // Update chart based on current view mode
        if (charts.thrust) {
          const chartPoint = getChartPointForMode(dataPoint);
          charts.thrust.data.datasets[0].data.push(chartPoint);
          charts.thrust.update('none');
        }

        // Update stats
        const samplesEl = document.getElementById('testSamples');
        const maxThrustEl = document.getElementById('testMaxThrust');
        const maxPWMEl = document.getElementById('testMaxPWM');
        if (samplesEl) samplesEl.textContent = testData.length;
        if (maxThrustEl) maxThrustEl.textContent = `${testMaxThrust.toFixed(2)} N`;
        if (maxPWMEl) maxPWMEl.textContent = `${testMaxPWM}%`;
      }
      break;

    case 21: // EVT_STATIC_TEST_COMPLETE
      testRunning = false;
      if (statusEl) {
        statusEl.textContent = `Test complete! ${testData.length} samples`;
        statusEl.style.color = '#22c55e';
      }
      if (testStatusEl) {
        testStatusEl.textContent = `Complete! ${testData.length} samples. Max: ${testMaxThrust.toFixed(2)}N at ${testMaxPWM}%`;
        testStatusEl.style.color = '#22c55e';
      }
      if (testStateEl) testStateEl.textContent = 'COMPLETE';
      updateAllPWMGauges(0);
      break;

    case 22: // EVT_STATIC_TEST_FAILED
      testRunning = false;
      if (statusEl) {
        statusEl.textContent = 'Test FAILED - check console';
        statusEl.style.color = '#ef4444';
      }
      if (testStatusEl) {
        testStatusEl.textContent = 'Test FAILED!';
        testStatusEl.style.color = '#ef4444';
      }
      if (testStateEl) testStateEl.textContent = 'FAILED';
      updateAllPWMGauges(0);
      break;
  }
}

/**
 * Updates all PWM gauges (dashboard and test page)
 *
 * @param {number} percent - Throttle percentage (0-100)
 */
function updateAllPWMGauges(percent) {
  updatePWMGauge(percent);
  updateTestPWMGauge(percent);
}

/**
 * Updates the test page PWM gauge display
 *
 * @param {number} percent - Throttle percentage (0-100)
 *
 * @description
 * Similar to updatePWMGauge but for the dedicated test page gauge.
 */
function updateTestPWMGauge(percent) {
  const value = Math.max(0, Math.min(100, percent));
  const valueEl = document.getElementById('testPwmValue');
  const fillEl = document.getElementById('testPwmGaugeFill');
  if (valueEl) valueEl.textContent = `${value}%`;
  if (fillEl) {
    const arcLength = 157;
    const offset = arcLength - (arcLength * value / 100);
    // Use setAttribute for SVG elements
    fillEl.setAttribute('stroke-dashoffset', offset);
    if (value < 30) {
      fillEl.setAttribute('stroke', '#22c55e');
    } else if (value < 70) {
      fillEl.setAttribute('stroke', '#f59e0b');
    } else {
      fillEl.setAttribute('stroke', '#ef4444');
    }
  }
}

/**
 * Handles motor calibration events from the flight computer
 *
 * @param {number} eventType - Event type ID (16-18 for calibration events)
 * @param {string} payload - Hex-encoded event payload (unused)
 *
 * @description
 * Processes ESC calibration events:
 * - 16: EVT_MOTOR_CAL_STARTED - Calibration phase 1 (MAX throttle)
 * - 17: EVT_MOTOR_CAL_PHASE2 - Calibration phase 2 (MIN throttle)
 * - 18: EVT_MOTOR_CALIBRATED - Calibration complete
 */
function handleMotorCalEvent(eventType, payload) {
  const consoleStatusEl = document.getElementById('staticTestStatus');
  const testStatusEl = document.getElementById('testStatus');

  switch (eventType) {
    case 16: // EVT_MOTOR_CAL_STARTED
      console.log('[DEBUG] Motor calibration STARTED event received');
      appendLog('[MOTOR] Calibration started - sending MAX throttle');
      if (consoleStatusEl) {
        consoleStatusEl.textContent = 'Motor calibrating... (MAX throttle)';
        consoleStatusEl.style.color = '#f59e0b';
      }
      if (testStatusEl) {
        testStatusEl.textContent = 'Motor calibrating... (MAX throttle)';
        testStatusEl.style.color = '#f59e0b';
      }
      break;
    case 17: // EVT_MOTOR_CAL_PHASE2
      appendLog('[MOTOR] Phase 2 - sending MIN throttle');
      if (consoleStatusEl) {
        consoleStatusEl.textContent = 'Motor calibrating... (MIN throttle)';
        consoleStatusEl.style.color = '#f59e0b';
      }
      if (testStatusEl) {
        testStatusEl.textContent = 'Motor calibrating... (MIN throttle)';
        testStatusEl.style.color = '#f59e0b';
      }
      break;
    case 18: // EVT_MOTOR_CALIBRATED
      console.log('[DEBUG] Motor CALIBRATED event received');
      motorCalibrated = true;
      appendLog('[MOTOR] Calibration complete!');
      if (consoleStatusEl) {
        consoleStatusEl.textContent = 'Motor calibrated - ready for test';
        consoleStatusEl.style.color = '#22c55e';
      }
      if (testStatusEl) {
        testStatusEl.textContent = 'Motor calibrated - ready for test';
        testStatusEl.style.color = '#22c55e';
      }
      break;
  }
}

// =============================================================================
// Command Button Handlers
// =============================================================================

// Setup command buttons with data-cmd attributes
const cmdButtons = document.querySelectorAll('[data-cmd]');
cmdButtons.forEach((btn) => {
  btn.addEventListener('click', () => {
    ws.send(JSON.stringify({ type: 'cmd', cmd: btn.getAttribute('data-cmd') }));
    appendLog(`[TX] ${btn.getAttribute('data-cmd')}`);
  });
});

// Manual command input handler
document.getElementById('sendCmd').addEventListener('click', () => {
  const input = document.getElementById('cmdInput');
  const cmd = input.value.trim();
  if (!cmd) return;
  ws.send(JSON.stringify({ type: 'cmd', cmd }));
  appendLog(`[CMD] ${cmd}`);
  input.value = '';
});

// Static Thrust Test button handler (Console page)
document.getElementById('staticTestBtn').addEventListener('click', () => {
  const pctInput = document.getElementById('staticTestPct');
  const statusEl = document.getElementById('staticTestStatus');
  const pct = parseInt(pctInput.value, 10);

  if (isNaN(pct) || pct < 1 || pct > 100) {
    statusEl.textContent = 'Invalid percentage (1-100)';
    statusEl.style.color = '#ef4444';
    return;
  }

  // Send command in format "EXX" where XX is the throttle percentage
  const cmd = `E${pct}`;
  ws.send(JSON.stringify({ type: 'cmd', cmd }));
  appendLog(`[CMD] Static Test ${pct}%`);
  statusEl.textContent = `Starting test at ${pct}%...`;
  statusEl.style.color = '#f59e0b';
});

// Test page button handlers
document.getElementById('startTestBtn')?.addEventListener('click', () => {
  const pctInput = document.getElementById('testThrottleInput');
  const statusEl = document.getElementById('testStatus');
  const pct = parseInt(pctInput.value, 10);

  if (isNaN(pct) || pct < 1 || pct > 100) {
    statusEl.textContent = 'Invalid percentage (1-100)';
    statusEl.style.color = '#ef4444';
    return;
  }

  const cmd = `E${pct}`;
  ws.send(JSON.stringify({ type: 'cmd', cmd }));
  appendLog(`[CMD] Static Test ${pct}%`);
  statusEl.textContent = `Starting test at ${pct}%...`;
  statusEl.style.color = '#f59e0b';
});

// Clear test data button handler
document.getElementById('clearTestBtn')?.addEventListener('click', () => {
  testData = [];
  testMaxThrust = 0;
  testMaxPWM = 0;
  testStartTime = 0;
  if (charts.thrust) {
    charts.thrust.data.datasets[0].data = [];
    charts.thrust.update('none');
  }
  document.getElementById('testSamples').textContent = '0';
  document.getElementById('testMaxThrust').textContent = '-- N';
  document.getElementById('testMaxPWM').textContent = '--%';
  document.getElementById('currentTime').textContent = '0.0 s';
  document.getElementById('currentPWM').textContent = '0%';
  document.getElementById('currentThrust').textContent = '0.00 N';
  document.getElementById('testStatus').textContent = 'Cleared - Ready for new test';
  document.getElementById('testStatus').style.color = '#888';
  document.getElementById('testState').textContent = 'IDLE';
  appendLog('[TEST] Data cleared');
});

// CSV download button handler
document.getElementById('downloadCsvBtn')?.addEventListener('click', () => {
  if (testData.length === 0) {
    alert('No test data to download. Run a test first.');
    return;
  }

  // Generate CSV content with time column
  let csv = 'time_ms,time_s,pwm_percent,thrust_n\n';
  testData.forEach(point => {
    csv += `${point.time_ms},${(point.time_ms / 1000).toFixed(3)},${point.pwm},${point.thrust.toFixed(3)}\n`;
  });

  // Add summary
  csv += `\n# Test Summary\n`;
  csv += `# Samples: ${testData.length}\n`;
  csv += `# Max Thrust: ${testMaxThrust.toFixed(3)} N at ${testMaxPWM}% PWM\n`;
  csv += `# Test Duration: ${testData.length > 0 ? (testData[testData.length-1].time_ms / 1000).toFixed(1) : 0} s\n`;

  // Create download
  const blob = new Blob([csv], { type: 'text/csv' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = `static_test_pwm_${testMaxPWM}.csv`;
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  URL.revokeObjectURL(url);

  appendLog(`[TEST] Downloaded CSV: ${testData.length} samples`);
});

// =============================================================================
// Serial Port Management
// =============================================================================

/**
 * Loads available serial ports from the server
 *
 * @async
 * @description
 * Fetches list of available COM ports from /api/ports endpoint
 * and populates the port selection dropdown.
 */
async function loadPorts() {
  const select = document.getElementById('portSelect');
  select.innerHTML = '';
  const res = await fetch('/api/ports');
  const data = await res.json();
  data.ports.forEach((p) => {
    const opt = document.createElement('option');
    opt.value = p.device;
    opt.textContent = `${p.device} - ${p.description}`;
    select.appendChild(opt);
  });
}

loadPorts();

document.getElementById('refreshPorts').addEventListener('click', loadPorts);

// Connect button handler
document.getElementById('connectBtn').addEventListener('click', async () => {
  const port = document.getElementById('portSelect').value;
  const res = await fetch('/api/connect', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ port })
  });
  const data = await res.json();
  document.getElementById('connectStatus').textContent = data.ok ? `Connected (${port})` : data.error;
  setStatus(data.ok);
});

// Disconnect button handler
document.getElementById('disconnectBtn').addEventListener('click', async () => {
  await fetch('/api/disconnect', { method: 'POST' });
  document.getElementById('connectStatus').textContent = 'Disconnected';
  setStatus(false);
});

/**
 * Loads help information (commands and profiles) from the server
 *
 * @async
 * @description
 * Fetches command reference and flight profiles from /api/help
 * and renders them in the Help page.
 */
async function loadHelp() {
  const res = await fetch('/api/help');
  const data = await res.json();
  const commands = document.getElementById('helpCommands');
  const profiles = document.getElementById('helpProfiles');
  commands.innerHTML = '';
  profiles.innerHTML = '';
  data.commands.forEach((cmd) => {
    const item = document.createElement('div');
    item.className = 'help-item';
    item.innerHTML = `<div class="cmd">${cmd.cmd}</div><div class="name">${cmd.name}</div><div>${cmd.desc}</div>`;
    commands.appendChild(item);
  });
  data.profiles.forEach((profile) => {
    const item = document.createElement('div');
    item.className = 'help-item';
    const params = profile.params.map((p) => `${p[0]}: ${p[1]}`).join(' | ');
    item.innerHTML = `<div class="cmd">P${profile.id}</div><div class="name">${profile.name}</div><div>${profile.description}<br>${params}</div>`;
    profiles.appendChild(item);
  });
}

/**
 * Loads current connection status from the server
 *
 * @async
 * @description
 * Checks if a serial connection is already established
 * and updates the UI accordingly.
 */
async function loadStatus() {
  const res = await fetch('/api/status');
  const data = await res.json();
  if (data.port) {
    document.getElementById('connectStatus').textContent = `Connected (${data.port})`;
  }
  setStatus(data.connected);
}

// =============================================================================
// Map Functions (Leaflet)
// =============================================================================

/** @type {L.Map|null} Leaflet map instance */
let map = null;

/** @type {L.Marker|null} Rocket position marker */
let marker = null;

/** @type {Array<[number, number]>} GPS track path coordinates */
let path = [];

/** @type {L.Polyline|null} Track path line on map */
let pathLine = null;

/**
 * Initializes the Leaflet map for GPS tracking
 *
 * @description
 * Creates a Leaflet map centered on Lisbon with OpenStreetMap tiles.
 * Adds a marker for rocket position and a polyline for track history.
 */
function initMap() {
  if (map || !window.L) return;
  map = L.map('map').setView([38.7223, -9.1393], 15);
  L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19
  }).addTo(map);
  marker = L.marker([38.7223, -9.1393]).addTo(map);
  pathLine = L.polyline(path, { color: '#22c55e', weight: 2 }).addTo(map);
}

/**
 * Updates the map with new GPS coordinates
 *
 * @param {number} lat - Latitude in degrees
 * @param {number} lon - Longitude in degrees
 * @param {boolean} hasFix - Whether GPS has a valid fix
 *
 * @description
 * Updates marker position and adds point to track path.
 * Maintains a rolling window of 200 track points.
 */
function updateMap(lat, lon, hasFix) {
  if (!map) {
    initMap();
  }
  if (!map || !hasFix) return;
  marker.setLatLng([lat, lon]);
  map.setView([lat, lon]);
  path.push([lat, lon]);
  if (path.length > 200) path.shift();
  pathLine.setLatLngs(path);
}

// =============================================================================
// 3D Visualization Functions (Three.js)
// =============================================================================

/** @type {THREE.Scene|undefined} Three.js scene */
let scene;

/** @type {THREE.PerspectiveCamera|undefined} Three.js camera */
let camera;

/** @type {THREE.WebGLRenderer|undefined} Three.js renderer */
let renderer;

/** @type {THREE.Group|undefined} Rocket 3D model group */
let rocket;

/**
 * @type {{x: number, y: number, z: number}}
 * @description Target rotation for smooth rocket animation (radians)
 */
let targetRotation = { x: 0, y: 0, z: 0 };

/**
 * Initializes the Three.js 3D rocket visualization
 *
 * @description
 * Creates a Three.js scene with:
 * - Perspective camera with 60° FOV
 * - Ambient and directional lighting
 * - Grid helper for reference
 * - Rocket model (cylinder body, cone nose, fins)
 *
 * Animation loop smoothly interpolates rocket rotation towards
 * target values received from telemetry.
 */
function initThree() {
  const container = document.getElementById('threeContainer');
  if (!container) return;
  scene = new THREE.Scene();
  camera = new THREE.PerspectiveCamera(60, container.clientWidth / container.clientHeight, 0.1, 1000);
  renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setSize(container.clientWidth, container.clientHeight);
  renderer.setPixelRatio(window.devicePixelRatio || 1);
  container.appendChild(renderer.domElement);

  const ambient = new THREE.AmbientLight(0xffffff, 0.6);
  scene.add(ambient);

  const keyLight = new THREE.DirectionalLight(0xffffff, 0.9);
  keyLight.position.set(6, 8, 6);
  scene.add(keyLight);

  const fillLight = new THREE.DirectionalLight(0x88ccff, 0.4);
  fillLight.position.set(-6, -4, 4);
  scene.add(fillLight);

  const grid = new THREE.GridHelper(30, 30, 0x243447, 0x182230);
  grid.position.y = -3;
  scene.add(grid);

  const body = new THREE.CylinderGeometry(0.5, 0.5, 4.5, 16);
  const cone = new THREE.ConeGeometry(0.6, 1.2, 16);
  const mat = new THREE.MeshStandardMaterial({ color: 0x22c55e, metalness: 0.25, roughness: 0.5 });
  const bodyMesh = new THREE.Mesh(body, mat);
  const coneMesh = new THREE.Mesh(cone, mat);
  coneMesh.position.y = 2.9;

  rocket = new THREE.Group();
  rocket.add(bodyMesh);
  rocket.add(coneMesh);

  const finMat = new THREE.MeshStandardMaterial({ color: 0x0ea5e9, metalness: 0.2, roughness: 0.6 });
  const fin = new THREE.BoxGeometry(0.15, 1.0, 0.8);
  const finPositions = [
    { x: 0.55, z: 0 },
    { x: -0.55, z: 0 },
    { x: 0, z: 0.55 },
    { x: 0, z: -0.55 }
  ];
  finPositions.forEach((pos) => {
    const finMesh = new THREE.Mesh(fin, finMat);
    finMesh.position.set(pos.x, -1.8, pos.z);
    finMesh.rotation.y = pos.x === 0 ? Math.PI / 2 : 0;
    rocket.add(finMesh);
  });

  scene.add(rocket);

  camera.position.set(0, 1.5, 8);
  camera.lookAt(0, 0, 0);

  /**
   * Animation loop - smoothly interpolates rocket rotation
   */
  function animate() {
    requestAnimationFrame(animate);
    if (rocket) {
      rocket.rotation.x += (targetRotation.x - rocket.rotation.x) * 0.08;
      rocket.rotation.y += (targetRotation.y - rocket.rotation.y) * 0.08;
      rocket.rotation.z += (targetRotation.z - rocket.rotation.z) * 0.08;
    }
    renderer.render(scene, camera);
  }
  animate();

  // Handle container resize
  const resizeObserver = new ResizeObserver(() => {
    const { clientWidth, clientHeight } = container;
    if (clientWidth && clientHeight) {
      renderer.setSize(clientWidth, clientHeight);
      camera.aspect = clientWidth / clientHeight;
      camera.updateProjectionMatrix();
    }
  });
  resizeObserver.observe(container);
}

// =============================================================================
// Initialization
// =============================================================================

window.addEventListener('load', () => {
  initMap();
  initThree();
  loadHelp();
  loadStatus();
  initFullscreenButtons();
  setInterval(() => {
    document.getElementById('timeValue').textContent = new Date().toLocaleTimeString();
  }, 1000);
});

// =============================================================================
// Fullscreen Toggle Functions
// =============================================================================

/**
 * Initializes fullscreen toggle buttons on chart cards
 *
 * @description
 * Adds fullscreen toggle buttons to cards containing canvases,
 * 3D container, or map elements. Skips cards that already have
 * buttons or don't need them.
 */
function initFullscreenButtons() {
  const cards = document.querySelectorAll('.card');
  cards.forEach((card) => {
    // Skip cards that already have buttons or don't need them
    if (card.querySelector('.fullscreen-btn')) return;
    if (!card.querySelector('canvas') && !card.querySelector('#threeContainer') && !card.querySelector('#map')) return;

    const btn = document.createElement('button');
    btn.className = 'fullscreen-btn';
    btn.innerHTML = '<svg viewBox="0 0 24 24"><path d="M7 14H5v5h5v-2H7v-3zm-2-4h2V7h3V5H5v5zm12 7h-3v2h5v-5h-2v3zM14 5v2h3v3h2V5h-5z"/></svg>';
    btn.title = 'Toggle Fullscreen';
    btn.addEventListener('click', (e) => {
      e.stopPropagation();
      toggleCardFullscreen(card);
    });
    card.appendChild(btn);
  });
}

/**
 * Toggles fullscreen mode for a card element
 *
 * @param {HTMLElement} card - Card element to toggle fullscreen
 */
function toggleCardFullscreen(card) {
  if (document.fullscreenElement === card) {
    document.exitFullscreen();
  } else {
    card.requestFullscreen().catch(err => {
      console.log('Fullscreen error:', err);
    });
  }
}

// Handle fullscreen change events - apply styling
document.addEventListener('fullscreenchange', () => {
  const fullscreenElement = document.fullscreenElement;
  if (fullscreenElement) {
    fullscreenElement.style.background = '#0b1117';
    fullscreenElement.style.padding = '20px';
  }
});
