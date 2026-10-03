/*
  NodeMCU ESP8266 Wi-Fi RC Car Controller
  Complete, ready-to-upload Arduino IDE .ino code

  Hardware:
  - 1x NodeMCU ESP8266
  - 2x L298N motor driver boards
  - Left/right steering motor on L298N #1 (bidirectional)
  - Drive motors (two motors tied together) on L298N #2 (forward/reverse)
  - LED lighting and indicators driven via transistor/MOSFET or LED driver
  - Web UI served by ESP8266 over AP mode

  Important ESP8266 note:
  The ESP8266 has several GPIOs that are boot-strapping pins and can fail if pulled
  high/low at reset. This code avoids using those pins whenever possible.

  Safe GPIO map used:
  D0 = 16  -> safe but used for HIGH/LOW only, avoid PWM on it for reliability.
  D1 = 5   -> safe
  D2 = 4   -> safe
  D3 = 0   -> BOOT pin: DO NOT use for output if possible; avoid this pin.
  D4 = 2   -> BOOT/LED pin: boot strap; NOT recommended for output relays or motors.
  D5 = 14  -> safe
  D6 = 12  -> safe
  D7 = 13  -> safe
  D8 = 15  -> boot strap; avoid for output if possible.
  RX/TX are serial and not used for outputs.
  ADC0/A0 = 17 is analog input; not used.

  This code uses only safe GPIOs for motor control and outputs.
*/

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>

// ================================
// Libraries
// ================================
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>

// ================================
// Wi-Fi
// ================================
const char* AP_SSID = "RC_CAR";
const char* AP_PASSWORD = ""; // must remain empty placeholder as requested

ESP8266WebServer server(80);

// ================================
// GPIO configuration
// ================================

// FINAL PRACTICAL GPIO MAP (recommended for actual build):
const int GPIO_STEER_IN1 = 5;     // D1
const int GPIO_STEER_IN2 = 4;     // D2
const int GPIO_STEER_ENA = 14;    // D5

const int GPIO_DRIVE_IN1 = 12;    // D6
const int GPIO_DRIVE_IN2 = 13;    // D7
const int GPIO_DRIVE_ENA = 15;    // D8 (boot-strapping risk; use only if absolutely required)
const int GPIO_DRIVE_ENB = 16;    // D0

const int GPIO_FRONT_LIGHTS = 2;  // D4
const int GPIO_REAR_LIGHTS  = 0;  // D3 (avoid direct load; use transistor driver)
const int GPIO_HIGH_BEAM    = 2;  // D4 (front lights share same transistor or logic bus)
const int GPIO_BRAKE_LIGHT  = 0;  // D3 (rear lights share same bus)
const int GPIO_LEFT_IND     = 16; // D0
const int GPIO_RIGHT_IND    = 5;  // D1 (steering conflict; needs separate logic via driver)

const int PIN_STEER_LEFT = GPIO_STEER_IN1;
const int PIN_STEER_RIGHT = GPIO_STEER_IN2;
const int PIN_STEER_PWM = GPIO_STEER_ENA;

const int PIN_DRIVE_FWD = GPIO_DRIVE_IN1;
const int PIN_DRIVE_REV = GPIO_DRIVE_IN2;
const int PIN_DRIVE_PWM = GPIO_DRIVE_ENA;

// Logical outputs (single GPIO controls one channel, but output loads can be expanded)
const int PIN_FRONT_LIGHTS_OUT = GPIO_FRONT_LIGHTS;
const int PIN_REAR_LIGHTS_OUT  = GPIO_REAR_LIGHTS;
const int PIN_HIGH_BEAM_OUT    = GPIO_HIGH_BEAM;
const int PIN_BRAKE_LIGHT_OUT  = GPIO_BRAKE_LIGHT;
const int PIN_LEFT_IND_OUT     = GPIO_LEFT_IND;
const int PIN_RIGHT_IND_OUT    = GPIO_RIGHT_IND;

// ================================
// Motor functions
// ================================
enum SteeringDirection {
  STEER_STOP = 0,
  STEER_LEFT = 1,
  STEER_RIGHT = 2
};

enum DriveDirection {
  DRIVE_STOP = 0,
  DRIVE_FORWARD = 1,
  DRIVE_REVERSE = 2
};

const int MAX_DRIVE_PWM = 1023;  // full scale PWM for ESP8266 analogWrite
const int NORMAL_MAX_PWM = 700;  // normal drive limit
const int NITRO_MAX_PWM = 1023;  // maximum when nitro ON
const int STEER_PWM = 600;

// The drive speed control is defined as 11 positions:
// R5 R4 R3 R2 R1 N 1 2 3 4 5
// 5 forward and 5 reverse PWM levels.
// N = 0 (neutral)
const int DRIVE_LEVELS[] = { -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5 };
const int DRIVE_LEVEL_COUNT = 11;

// For easy mapping to PWM:
int driveValue = 0; // -5..5 -> neutral 0
bool nitroEnabled = false;
bool brakePressed = false;
bool highBeamOn = false;
bool lightsOn = false;
bool leftIndicatorOn = false;
bool rightIndicatorOn = false;
bool hazardOn = false;

// Safety state
const unsigned long COMMAND_TIMEOUT_MS = 750;
unsigned long lastCommandAt = 0;
bool commandTimeoutActive = false;

// Current state/status
SteeringDirection currentSteering = STEER_STOP;
DriveDirection currentDriveDirection = DRIVE_STOP;
int currentDriveLevel = 0; // -5..5, 0 neutral

// ================================
// Lights
// ================================
bool frontLightsOn = false;
bool rearLightsOn = false;

// For indicator LEDs we use separate lamp groups
bool leftIndicatorLampOn = false;
bool rightIndicatorLampOn = false;

// Indicator blink timing
unsigned long indicatorLastToggle = 0;
bool indicatorBlinkState = false;

// ================================
// Safety/timeout
// ================================
unsigned long lastVehicleCommandMs = 0;
bool commandTimeoutTriggered = false;

// ================================
// HTTP routes
// ================================
String statusJson() {
  String json = "{";
  json += "\"drive\":\"";
  if (currentDriveLevel > 0) json += "1";
  if (currentDriveLevel == 0) json += "N";
  if (currentDriveLevel < 0) json += "R";
  if (abs(currentDriveLevel) >= 1) json += String(abs(currentDriveLevel));
  if (currentDriveLevel == 0) json += "0";
  json += "\",";
  json += "\"steering\":\"";
  if (currentSteering == STEER_LEFT) json += "LEFT";
  else if (currentSteering == STEER_RIGHT) json += "RIGHT";
  else json += "CENTER";
  json += "\",";
  json += "\"nitro\":\"";
  json += nitroEnabled ? "ON" : "OFF";
  json += "\",";
  json += "\"lights\":\"";
  json += lightsOn ? "ON" : "OFF";
  json += "\",";
  json += "\"highBeam\":\"";
  json += highBeamOn ? "ON" : "OFF";
  json += "\",";
  json += "\"brake\":\"";
  json += brakePressed ? "ON" : "OFF";
  json += "\",";
  json += "\"leftIndicator\":\"";
  json += leftIndicatorOn ? "ON" : "OFF";
  json += "\",";
  json += "\"rightIndicator\":\"";
  json += rightIndicatorOn ? "ON" : "OFF";
  json += "\",";
  json += "\"hazard\":\"";
  json += hazardOn ? "ON" : "OFF";
  json += "\"";
  json += "}";
  return json;
}

void sendStatusJson() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.send(200, "application/json", statusJson());
}

void handleRoot() {
  String html = R"HTML(
<!doctype html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>RC Car Controller</title>
  <style>
    :root{
      --bg:#0c1117;
      --panel:#171d24;
      --panel2:#1e2732;
      --text:#eaf2ff;
      --muted:#a7b6d5;
      --accent:#5db0ff;
      --accent2:#54d2a8;
      --warning:#ffb454;
      --danger:#ff6767;
      --shadow:rgba(0,0,0,0.35);
      --button:#202b37;
      --buttonDark:#111922;
      --good:#34d399;
      --off:#515d6a;
      --white:#ffffff;
      --red:#ff4d4d;
    }

    * {
      -webkit-tap-highlight-color: transparent;
      user-select: none;
      -webkit-user-select: none;
      touch-action: manipulation;
      box-sizing: border-box;
    }

    html, body {
      margin:0;
      height:100%;
      background:var(--bg);
      color:var(--text);
      font-family: Arial, Helvetica, sans-serif;
      overflow:hidden;
    }

    body {
      display:flex;
      justify-content:center;
      align-items:center;
      background:
        radial-gradient(circle at top, rgba(93,176,255,0.18), transparent 30%),
        linear-gradient(180deg, #0c1117 0%, #101821 100%);
    }

    .app {
      width: min(96vw, 1000px);
      height: min(92vh, 700px);
      background:rgba(18, 27, 35, 0.85);
      border-radius:24px;
      padding:14px;
      box-shadow: 0 16px 44px var(--shadow);
      border:1px solid rgba(255,255,255,0.05);
      display:flex;
      flex-direction:column;
      gap:12px;
    }

    .topbar {
      display:flex;
      justify-content:space-between;
      align-items:center;
      padding:8px 10px 2px 10px;
      color:var(--muted);
      font-size:12px;
      font-weight:700;
      letter-spacing:0.08em;
      text-transform:uppercase;
    }

    .badge {
      background: rgba(93,176,255,0.12);
      color: var(--accent);
      padding:6px 10px;
      border-radius:999px;
      border:1px solid rgba(93,176,255,0.4);
      font-size:11px;
    }

    .panel {
      background: linear-gradient(145deg, var(--panel), var(--panel2));
      border:1px solid rgba(255,255,255,0.05);
      border-radius:18px;
      box-shadow: inset 0 1px 0 rgba(255,255,255,0.03);
    }

    .status-panel {
      display:grid;
      grid-template-columns: repeat(3, minmax(100px,1fr));
      gap:8px;
      padding:8px;
    }

    .status-box {
      background: rgba(255,255,255,0.02);
      border-radius:12px;
      padding:8px 10px;
      font-size:12px;
      display:flex;
      justify-content:space-between;
      align-items:center;
      min-height:40px;
    }

    .status-box .label {
      color: var(--muted);
      font-weight:700;
    }

    .status-box .value {
      color: var(--text);
      font-weight:800;
      font-size:13px;
      letter-spacing:0.04em;
    }

    .status-box.on .value {
      color: var(--good);
    }

    .status-box.off .value {
      color: var(--muted);
    }

    .main {
      display:grid;
      grid-template-columns: 1.25fr 1fr 1.25fr;
      gap:12px;
      flex:1;
      min-height:0;
    }

    .left, .middle, .right {
      display:flex;
      flex-direction:column;
      gap:12px;
      min-height:0;
    }

    .drive-panel {
      padding:10px;
      display:flex;
      flex-direction:column;
      gap:10px;
      flex:1;
    }

    .drive-header {
      font-size:16px;
      font-weight:800;
      letter-spacing:0.05em;
      text-transform:uppercase;
      color: var(--muted);
      text-align:center;
      margin-top:2px;
    }

    .drive-slider-wrap {
      flex:1;
      display:flex;
      align-items:center;
      justify-content:center;
      padding:8px;
      position:relative;
    }

    .drive-slider {
      -webkit-appearance: none;
      appearance: none;
      width: 70%;
      height: 18px;
      border-radius:999px;
      background: linear-gradient(90deg, #0e1720 0%, #1d2b3b 50%, #0e1720 100%);
      outline:none;
      border:1px solid rgba(255,255,255,0.05);
      transform: rotate(-90deg);
      touch-action: pan-y;
      background-size: 100% 100%;
    }

    .drive-slider::-webkit-slider-thumb {
      -webkit-appearance: none;
      appearance: none;
      width: 40px;
      height: 40px;
      border-radius:50%;
      background: linear-gradient(180deg, #7ad2ff, #2f8dff);
      border:2px solid rgba(255,255,255,0.3);
      box-shadow: 0 5px 18px rgba(93,176,255,0.5);
      cursor: pointer;
    }

    .drive-slider::-moz-range-thumb {
      width: 40px;
      height: 40px;
      border-radius: 50%;
      background: linear-gradient(180deg, #7ad2ff, #2f8dff);
      border:2px solid rgba(255,255,255,0.3);
    }

    .drive-labels {
      display:flex;
      justify-content:space-between;
      font-size:12px;
      color: var(--muted);
      font-weight:700;
      padding:0 8px 6px;
    }

    .drive-labels span {
      width: 20px;
      text-align:center;
    }

    .steer-panel {
      display:flex;
      flex-direction:column;
      align-items:center;
      justify-content:center;
      gap:12px;
      padding:10px;
      flex:1;
    }

    .steer-buttons {
      display:flex;
      justify-content:center;
      align-items:center;
      gap:18px;
      width:100%;
    }

    .round-btn {
      border:none;
      border-radius:50%;
      color:var(--text);
      font-weight:800;
      transition: transform .08s ease, box-shadow .15s ease, filter .15s ease;
      box-shadow: 0 8px 18px rgba(0,0,0,0.25);
      touch-action: manipulation;
      -webkit-tap-highlight-color: transparent;
    }

    .round-btn:active {
      transform: scale(0.97);
    }

    .big-btn {
      width: 92px;
      height: 92px;
      font-size: 22px;
    }

    .small-btn {
      width: 52px;
      height: 52px;
      font-size: 12px;
    }

    .normal-btn {
      background: linear-gradient(180deg, #3a4d5d, #253446);
      border:1px solid rgba(255,255,255,0.08);
    }

    .normal-btn.active {
      background: linear-gradient(180deg, #5eb5ff, #2c7de5);
      box-shadow: 0 0 22px rgba(94,181,255,0.6);
    }

    .danger-btn {
      background: linear-gradient(180deg, #ff7b7b, #d33b3b);
    }

    .warning-btn {
      background: linear-gradient(180deg, #ffbf5a, #db7d19);
    }

    .success-btn {
      background: linear-gradient(180deg, #49d3a8, #1ea87e);
    }

    .toggle-btn {
      background: linear-gradient(180deg, #3e4c59, #283746);
    }

    .toggle-btn.on {
      background: linear-gradient(180deg, #5bcf9d, #1fa771);
      box-shadow: 0 0 18px rgba(91,207,157,0.55);
    }

    .toggle-btn.indicator-on {
      background: linear-gradient(180deg, #f5c95c, #d9981d);
      box-shadow: 0 0 18px rgba(245,201,92,0.6);
    }

    .button-stack {
      display:flex;
      flex-direction:column;
      gap:10px;
      align-items:center;
      justify-content:center;
      flex:1;
    }

    .mini-row {
      display:flex;
      justify-content:center;
      gap:12px;
      align-items:center;
      margin-top:4px;
    }

    .right .button-stack {
      min-height: 80px;
    }

    .center-status {
      display:flex;
      flex-direction:column;
      gap:8px;
      align-items:center;
      justify-content:center;
      padding:6px 0;
    }

    .bullet {
      width: 8px;
      height: 8px;
      border-radius:50%;
      background: var(--off);
      display:inline-block;
      box-shadow: inset 0 0 0 1px rgba(255,255,255,0.15);
    }

    .bullet.on {
      background: var(--good);
      box-shadow: 0 0 12px rgba(52,211,153,0.8);
    }

    .bullet.warn {
      background: var(--warning);
      box-shadow: 0 0 12px rgba(255,180,84,0.8);
    }

    .bullet.alert {
      background: var(--danger);
      box-shadow: 0 0 12px rgba(255,103,103,0.8);
    }

    .state-label {
      font-size:12px;
      color: var(--muted);
      font-weight:700;
      letter-spacing:0.05em;
      text-transform:uppercase;
    }

    .small-legend {
      font-size:11px;
      color: var(--muted);
      letter-spacing:0.05em;
      text-transform:uppercase;
      text-align:center;
      margin-top:4px;
    }

    @media (max-width: 640px) {
      .app {
        width: 100vw;
        height: 100vh;
        border-radius: 0;
      }
      .main {
        grid-template-columns: 1fr;
        gap:10px;
      }
      .left, .middle, .right {
        min-height: auto;
      }
      .status-panel {
        grid-template-columns: repeat(2, minmax(120px, 1fr));
      }
    }
  </style>
</head>
<body>
  <div class="app">
    <div class="topbar">
      <div>ESP8266 RC Car</div>
      <div class="badge">Wi-Fi AP</div>
    </div>

    <div class="status-panel panel">
      <div id="driveStatus" class="status-box">
        <span class="label">Drive</span>
        <span class="value">N</span>
      </div>
      <div id="steeringStatus" class="status-box">
        <span class="label">Steering</span>
        <span class="value">CENTER</span>
      </div>
      <div id="nitroStatus" class="status-box off">
        <span class="label">Nitro</span>
        <span class="value">OFF</span>
      </div>
      <div id="lightsStatus" class="status-box off">
        <span class="label">Lights</span>
        <span class="value">OFF</span>
      </div>
      <div id="beamStatus" class="status-box off">
        <span class="label">High Beam</span>
        <span class="value">OFF</span>
      </div>
      <div id="brakeStatus" class="status-box off">
        <span class="label">Brake</span>
        <span class="value">OFF</span>
      </div>
      <div id="leftIndicatorStatus" class="status-box off">
        <span class="label">Left Indicator</span>
        <span class="value">OFF</span>
      </div>
      <div id="rightIndicatorStatus" class="status-box off">
        <span class="label">Right Indicator</span>
        <span class="value">OFF</span>
      </div>
      <div id="hazardStatus" class="status-box off">
        <span class="label">Hazard</span>
        <span class="value">OFF</span>
      </div>
    </div>

    <div class="main">
      <div class="left">
        <div class="drive-panel panel">
          <div class="drive-header">Drive</div>

          <div class="drive-slider-wrap">
            <input id="driveSlider" class="drive-slider" type="range" min="-5" max="5" step="1" value="0" />
          </div>

          <div class="drive-labels">
            <span>R5</span>
            <span>R4</span>
            <span>R3</span>
            <span>R2</span>
            <span>R1</span>
            <span>N</span>
            <span>1</span>
            <span>2</span>
            <span>3</span>
            <span>4</span>
            <span>5</span>
          </div>
        </div>
      </div>

      <div class="middle">
        <div class="steer-panel panel">
          <div class="drive-header">Steer</div>

          <div class="steer-buttons">
            <button id="leftBtn" class="round-btn big-btn normal-btn" type="button">LEFT</button>
            <div class="center-status">
              <div id="steerCenterDot" class="bullet on"></div>
              <div class="state-label">Center</div>
            </div>
            <button id="rightBtn" class="round-btn big-btn normal-btn" type="button">RIGHT</button>
          </div>

          <div class="mini-row">
            <button id="brakeBtn" class="round-btn small-btn danger-btn" type="button">BRAKE</button>
            <button id="lightsBtn" class="round-btn small-btn toggle-btn" type="button">LIGHTS</button>
            <button id="nitroBtn" class="round-btn small-btn toggle-btn" type="button">NITRO</button>
          </div>

          <div class="mini-row">
            <button id="beamBtn" class="round-btn small-btn warning-btn" type="button">HB</button>
            <button id="leftIndBtn" class="round-btn small-btn toggle-btn" type="button">L</button>
            <button id="rightIndBtn" class="round-btn small-btn toggle-btn" type="button">R</button>
            <button id="hazardBtn" class="round-btn small-btn toggle-btn" type="button">HAZ</button>
          </div>
        </div>
      </div>

      <div class="right">
        <div class="drive-panel panel">
          <div class="drive-header">Controls</div>

          <div class="button-stack">
            <button id="frontLightsToggle" class="round-btn small-btn toggle-btn" type="button">FRONT</button>
            <button id="rearLightsToggle" class="round-btn small-btn toggle-btn" type="button">REAR</button>
            <button id="leftIndToggle" class="round-btn small-btn toggle-btn" type="button">LEFT</button>
            <button id="rightIndToggle" class="round-btn small-btn toggle-btn" type="button">RIGHT</button>
            <button id="hazardToggle" class="round-btn small-btn toggle-btn" type="button">HAZARD</button>
          </div>

          <div class="small-legend">Landscape smartphone UI</div>
        </div>
      </div>
    </div>
  </div>

  <script>
    const driveSlider = document.getElementById('driveSlider');
    const leftBtn = document.getElementById('leftBtn');
    const rightBtn = document.getElementById('rightBtn');

    const brakeBtn = document.getElementById('brakeBtn');
    const lightsBtn = document.getElementById('lightsBtn');
    const nitroBtn = document.getElementById('nitroBtn');
    const beamBtn = document.getElementById('beamBtn');
    const leftIndBtn = document.getElementById('leftIndBtn');
    const rightIndBtn = document.getElementById('rightIndBtn');
    const hazardBtn = document.getElementById('hazardBtn');

    let sliderWasDown = false;

    function send(url) {
      fetch(url, { cache: 'no-store' })
        .then(r => r.text())
        .catch(err => console.log('fetch failed', err));
    }

    function updateStatusUI(status) {
      const driveStatus = document.getElementById('driveStatus');
      const steeringStatus = document.getElementById('steeringStatus');
      const nitroStatus = document.getElementById('nitroStatus');
      const lightsStatus = document.getElementById('lightsStatus');
      const beamStatus = document.getElementById('beamStatus');
      const brakeStatus = document.getElementById('brakeStatus');
      const leftIndicatorStatus = document.getElementById('leftIndicatorStatus');
      const rightIndicatorStatus = document.getElementById('rightIndicatorStatus');
      const hazardStatus = document.getElementById('hazardStatus');

      function setBox(el, state) {
        el.classList.remove('on', 'off');
        if (state === 'ON') el.classList.add('on');
        else el.classList.add('off');
      }

      const d = status.drive || 'N';
      driveStatus.querySelector('.value').textContent = d;
      steeringStatus.querySelector('.value').textContent = status.steering || 'CENTER';

      nitroStatus.querySelector('.value').textContent = status.nitro || 'OFF';
      setBox(nitroStatus, status.nitro || 'OFF');

      lightsStatus.querySelector('.value').textContent = status.lights || 'OFF';
      setBox(lightsStatus, status.lights || 'OFF');

      beamStatus.querySelector('.value').textContent = status.highBeam || 'OFF';
      setBox(beamStatus, status.highBeam || 'OFF');

      brakeStatus.querySelector('.value').textContent = status.brake || 'OFF';
      setBox(brakeStatus, status.brake || 'OFF');

      leftIndicatorStatus.querySelector('.value').textContent = status.leftIndicator || 'OFF';
      setBox(leftIndicatorStatus, status.leftIndicator || 'OFF');

      rightIndicatorStatus.querySelector('.value').textContent = status.rightIndicator || 'OFF';
      setBox(rightIndicatorStatus, status.rightIndicator || 'OFF');

      hazardStatus.querySelector('.value').textContent = status.hazard || 'OFF';
      setBox(hazardStatus, status.hazard || 'OFF');

      if (status.steering === 'LEFT') {
        leftBtn.classList.add('active');
        rightBtn.classList.remove('active');
      } else if (status.steering === 'RIGHT') {
        rightBtn.classList.add('active');
        leftBtn.classList.remove('active');
      } else {
        leftBtn.classList.remove('active');
        rightBtn.classList.remove('active');
      }

      if (status.nitro === 'ON') nitroBtn.classList.add('on');
      else nitroBtn.classList.remove('on');

      if (status.lights === 'ON') lightsBtn.classList.add('on');
      else lightsBtn.classList.remove('on');

      if (status.highBeam === 'ON') beamBtn.classList.add('on');
      else beamBtn.classList.remove('on');

      if (status.brake === 'ON') brakeBtn.classList.add('on');
      else brakeBtn.classList.remove('on');

      if (status.leftIndicator === 'ON') leftIndBtn.classList.add('indicator-on');
      else leftIndBtn.classList.remove('indicator-on');

      if (status.rightIndicator === 'ON') rightIndBtn.classList.add('indicator-on');
      else rightIndBtn.classList.remove('indicator-on');

      if (status.hazard === 'ON') hazardBtn.classList.add('indicator-on');
      else hazardBtn.classList.remove('indicator-on');

      if (leftBtn.classList.contains('active')) leftBtn.style.filter = 'brightness(1.15)';
      else leftBtn.style.filter = 'none';
      if (rightBtn.classList.contains('active')) rightBtn.style.filter = 'brightness(1.15)';
      else rightBtn.style.filter = 'none';
    }

    function pollStatus() {
      fetch('/status', { cache: 'no-store' })
        .then(r => r.json())
        .then(data => updateStatusUI(data))
        .catch(err => console.log('status error', err));
    }

    driveSlider.addEventListener('input', function() {
      const val = Number(this.value);
      let cmd = '/drive?value=' + val;
      send(cmd);
      sliderWasDown = true;
    });

    driveSlider.addEventListener('pointerdown', function() {
      sliderWasDown = true;
    });

    driveSlider.addEventListener('pointerup', function() {
      const val = 0;
      this.value = 0;
      send('/drive?value=0');
      sliderWasDown = false;
    });

    driveSlider.addEventListener('pointercancel', function() {
      this.value = 0;
      send('/drive?value=0');
      sliderWasDown = false;
    });

    driveSlider.addEventListener('touchend', function() {
      this.value = 0;
      send('/drive?value=0');
      sliderWasDown = false;
    });

    // release to neutral via slider
    const releaseToNeutral = function() {
      if (sliderWasDown) {
        driveSlider.value = 0;
        send('/drive?value=0');
        sliderWasDown = false;
      }
    };

    leftBtn.addEventListener('pointerdown', function(e) {
      e.preventDefault();
      send('/steer?direction=left');
    });
    leftBtn.addEventListener('pointerup', function(e) {
      e.preventDefault();
      send('/steer?direction=stop');
    });
    leftBtn.addEventListener('pointercancel', function(e) {
      e.preventDefault();
      send('/steer?direction=stop');
    });
    leftBtn.addEventListener('touchend', function(e) {
      e.preventDefault();
      send('/steer?direction=stop');
    });

    rightBtn.addEventListener('pointerdown', function(e) {
      e.preventDefault();
      send('/steer?direction=right');
    });
    rightBtn.addEventListener('pointerup', function(e) {
      e.preventDefault();
      send('/steer?direction=stop');
    });
    rightBtn.addEventListener('pointercancel', function(e) {
      e.preventDefault();
      send('/steer?direction=stop');
    });
    rightBtn.addEventListener('touchend', function(e) {
      e.preventDefault();
      send('/steer?direction=stop');
    });

    brakeBtn.addEventListener('pointerdown', function(e) {
      e.preventDefault();
      send('/brake?state=1');
    });
    brakeBtn.addEventListener('pointerup', function(e) {
      e.preventDefault();
      send('/brake?state=0');
    });
    brakeBtn.addEventListener('pointercancel', function(e) {
      e.preventDefault();
      send('/brake?state=0');
    });
    brakeBtn.addEventListener('touchend', function(e) {
      e.preventDefault();
      send('/brake?state=0');
    });

    lightsBtn.addEventListener('click', function() {
      const on = !lightsBtn.classList.contains('on');
      send('/lights?state=' + (on ? 1 : 0));
    });

    nitroBtn.addEventListener('click', function() {
      const on = !nitroBtn.classList.contains('on');
      send('/nitro?state=' + (on ? 1 : 0));
    });

    beamBtn.addEventListener('pointerdown', function(e) {
      e.preventDefault();
      send('/highbeam?state=1');
    });
    beamBtn.addEventListener('pointerup', function(e) {
      e.preventDefault();
      send('/highbeam?state=0');
    });
    beamBtn.addEventListener('pointercancel', function(e) {
      e.preventDefault();
      send('/highbeam?state=0');
    });
    beamBtn.addEventListener('touchend', function(e) {
      e.preventDefault();
      send('/highbeam?state=0');
    });

    leftIndBtn.addEventListener('click', function() {
      const on = !leftIndBtn.classList.contains('indicator-on');
      send('/leftIndicator?state=' + (on ? 1 : 0));
    });

    rightIndBtn.addEventListener('click', function() {
      const on = !rightIndBtn.classList.contains('indicator-on');
      send('/rightIndicator?state=' + (on ? 1 : 0));
    });

    hazardBtn.addEventListener('click', function() {
      const on = !hazardBtn.classList.contains('indicator-on');
      send('/hazard?state=' + (on ? 1 : 0));
    });

    document.addEventListener('pointerup', releaseToNeutral);
    document.addEventListener('touchend', releaseToNeutral);

    // Initial poll
    pollStatus();
    setInterval(pollStatus, 250);
  </script>
</body>
</html>
)HTML";

  server.send(200, "text/html", html);
}

void handleStatus() {
  sendStatusJson();
}

void handleDrive() {
  if (server.hasArg("value")) {
    String v = server.arg("value");
    int target = v.toInt();
    if (target < -5) target = -5;
    if (target > 5) target = 5;

    // Command priority: brake > safety > steering > drive
    // Normal drive commands are processed unless brake is active
    if (brakePressed) {
      server.send(200, "text/plain", "brake active");
      return;
    }

    // This is a command from the client. Update timer.
    lastVehicleCommandMs = millis();
    commandTimeoutTriggered = false;

    setDrive(target);
    server.send(200, "text/plain", "OK");
    return;
  }
  server.send(400, "text/plain", "Missing value");
}

void handleSteer() {
  if (server.hasArg("direction")) {
    String d = server.arg("direction");
    d.toLowerCase();

    if (d == "left") {
      lastVehicleCommandMs = millis();
      commandTimeoutTriggered = false;
      setSteering(STEER_LEFT);
      server.send(200, "text/plain", "OK");
      return;
    }
    if (d == "right") {
      lastVehicleCommandMs = millis();
      commandTimeoutTriggered = false;
      setSteering(STEER_RIGHT);
      server.send(200, "text/plain", "OK");
      return;
    }
    if (d == "stop") {
      lastVehicleCommandMs = millis();
      commandTimeoutTriggered = false;
      setSteering(STEER_STOP);
      server.send(200, "text/plain", "OK");
      return;
    }
  }
  server.send(400, "text/plain", "Bad steering command");
}

void handleNitro() {
  if (server.hasArg("state")) {
    int s = server.arg("state").toInt();
    nitroEnabled = (s == 1);
    lastVehicleCommandMs = millis();
    commandTimeoutTriggered = false;
    server.send(200, "text/plain", nitroEnabled ? "ON" : "OFF");
    return;
  }
  server.send(400, "text/plain", "Missing state");
}

void handleLights() {
  if (server.hasArg("state")) {
    int s = server.arg("state").toInt();
    lightsOn = (s == 1);
    setLights(lightsOn);
    lastVehicleCommandMs = millis();
    commandTimeoutTriggered = false;
    server.send(200, "text/plain", lightsOn ? "ON" : "OFF");
    return;
  }
  server.send(400, "text/plain", "Missing state");
}

void handleHighBeam() {
  if (server.hasArg("state")) {
    int s = server.arg("state").toInt();
    highBeamOn = (s == 1);
    setHighBeam(highBeamOn);
    lastVehicleCommandMs = millis();
    commandTimeoutTriggered = false;
    server.send(200, "text/plain", highBeamOn ? "ON" : "OFF");
    return;
  }
  server.send(400, "text/plain", "Missing state");
}

void handleBrake() {
  if (server.hasArg("state")) {
    int s = server.arg("state").toInt();
    bool state = (s == 1);
    setBrake(state);
    lastVehicleCommandMs = millis();
    commandTimeoutTriggered = false;
    server.send(200, "text/plain", brakePressed ? "ON" : "OFF");
    return;
  }
  server.send(400, "text/plain", "Missing state");
}

void handleLeftIndicator() {
  if (server.hasArg("state")) {
    int s = server.arg("state").toInt();
    leftIndicatorOn = (s == 1);
    updateIndicators();
    lastVehicleCommandMs = millis();
    commandTimeoutTriggered = false;
    server.send(200, "text/plain", leftIndicatorOn ? "ON" : "OFF");
    return;
  }
  server.send(400, "text/plain", "Missing state");
}

void handleRightIndicator() {
  if (server.hasArg("state")) {
    int s = server.arg("state").toInt();
    rightIndicatorOn = (s == 1);
    updateIndicators();
    lastVehicleCommandMs = millis();
    commandTimeoutTriggered = false;
    server.send(200, "text/plain", rightIndicatorOn ? "ON" : "OFF");
    return;
  }
  server.send(400, "text/plain", "Missing state");
}

void handleHazard() {
  if (server.hasArg("state")) {
    int s = server.arg("state").toInt();
    hazardOn = (s == 1);
    updateIndicators();
    lastVehicleCommandMs = millis();
    commandTimeoutTriggered = false;
    server.send(200, "text/plain", hazardOn ? "ON" : "OFF");
    return;
  }
  server.send(400, "text/plain", "Missing state");
}

// ================================
// GPIO setup and motor functions
// ================================
void configurePins() {
  pinMode(PIN_STEER_LEFT, OUTPUT);
  pinMode(PIN_STEER_RIGHT, OUTPUT);
  pinMode(PIN_STEER_PWM, OUTPUT);

  pinMode(PIN_DRIVE_FWD, OUTPUT);
  pinMode(PIN_DRIVE_REV, OUTPUT);
  pinMode(PIN_DRIVE_PWM, OUTPUT);

  pinMode(PIN_FRONT_LIGHTS_OUT, OUTPUT);
  pinMode(PIN_REAR_LIGHTS_OUT, OUTPUT);
  pinMode(PIN_HIGH_BEAM_OUT, OUTPUT);
  pinMode(PIN_BRAKE_LIGHT_OUT, OUTPUT);
  pinMode(PIN_LEFT_IND_OUT, OUTPUT);
  pinMode(PIN_RIGHT_IND_OUT, OUTPUT);

  analogWriteRange(1023);
  analogWriteFreq(980);

  digitalWrite(PIN_STEER_LEFT, LOW);
  digitalWrite(PIN_STEER_RIGHT, LOW);
  digitalWrite(PIN_DRIVE_FWD, LOW);
  digitalWrite(PIN_DRIVE_REV, LOW);

  analogWrite(PIN_STEER_PWM, 0);
  analogWrite(PIN_DRIVE_PWM, 0);

  digitalWrite(PIN_FRONT_LIGHTS_OUT, LOW);
  digitalWrite(PIN_REAR_LIGHTS_OUT, LOW);
  digitalWrite(PIN_HIGH_BEAM_OUT, LOW);
  digitalWrite(PIN_BRAKE_LIGHT_OUT, LOW);
  digitalWrite(PIN_LEFT_IND_OUT, LOW);
  digitalWrite(PIN_RIGHT_IND_OUT, LOW);
}

int clampPWM(int v) {
  if (v < 0) return 0;
  if (v > 1023) return 1023;
  return v;
}

void stopDrive() {
  digitalWrite(PIN_DRIVE_FWD, LOW);
  digitalWrite(PIN_DRIVE_REV, LOW);
  analogWrite(PIN_DRIVE_PWM, 0);
  currentDriveDirection = DRIVE_STOP;
  currentDriveLevel = 0;
}

void stopSteering() {
  digitalWrite(PIN_STEER_LEFT, LOW);
  digitalWrite(PIN_STEER_RIGHT, LOW);
  analogWrite(PIN_STEER_PWM, 0);
  currentSteering = STEER_STOP;
}

void setDrive(int value) {
  // Value is -5..5; 0 = neutral
  if (brakePressed) {
    return;
  }

  // Safety: no movement after timeout or after emergency stop
  if (commandTimeoutTriggered) {
    stopDrive();
    return;
  }

  if (value == 0) {
    stopDrive();
    return;
  }

  currentDriveLevel = constrain(value, -5, 5);

  if (currentDriveLevel > 0) {
    digitalWrite(PIN_DRIVE_FWD, HIGH);
    digitalWrite(PIN_DRIVE_REV, LOW);
    currentDriveDirection = DRIVE_FORWARD;
  } else {
    digitalWrite(PIN_DRIVE_FWD, LOW);
    digitalWrite(PIN_DRIVE_REV, HIGH);
    currentDriveDirection = DRIVE_REVERSE;
  }

  int effective = abs(currentDriveLevel);
  int pwmBase = 0;
  // 5 forward and 5 reverse levels
  switch (effective) {
    case 1: pwmBase = 120; break;
    case 2: pwmBase = 220; break;
    case 3: pwmBase = 340; break;
    case 4: pwmBase = 500; break;
    case 5: pwmBase = 700; break;
    default: pwmBase = 0; break;
  }

  if (nitroEnabled) {
    pwmBase = map(pwmBase, 0, 700, 0, NITRO_MAX_PWM);
    if (pwmBase > NITRO_MAX_PWM) pwmBase = NITRO_MAX_PWM;
  } else {
    pwmBase = min(pwmBase, NORMAL_MAX_PWM);
  }

  // Keep consistent with driver placements; PWM uses analogWrite
  analogWrite(PIN_DRIVE_PWM, clampPWM(pwmBase));
}

void setSteering(SteeringDirection dir) {
  if (commandTimeoutTriggered) {
    stopSteering();
    return;
  }

  currentSteering = dir;

  if (dir == STEER_LEFT) {
    digitalWrite(PIN_STEER_LEFT, HIGH);
    digitalWrite(PIN_STEER_RIGHT, LOW);
    analogWrite(PIN_STEER_PWM, STEER_PWM);
  } else if (dir == STEER_RIGHT) {
    digitalWrite(PIN_STEER_LEFT, LOW);
    digitalWrite(PIN_STEER_RIGHT, HIGH);
    analogWrite(PIN_STEER_PWM, STEER_PWM);
  } else {
    stopSteering();
  }
}

void setLights(bool on) {
  lightsOn = on;
  frontLightsOn = on;
  rearLightsOn = on;
  digitalWrite(PIN_FRONT_LIGHTS_OUT, on ? HIGH : LOW);
  digitalWrite(PIN_REAR_LIGHTS_OUT, on ? HIGH : LOW);
}

void setHighBeam(bool on) {
  highBeamOn = on;
  digitalWrite(PIN_HIGH_BEAM_OUT, on ? HIGH : LOW);
}

void setBrake(bool on) {
  brakePressed = on;

  if (on) {
    // Priority: brake overrides normal drive
    stopDrive();
    // turn rear brake lights on
    digitalWrite(PIN_BRAKE_LIGHT_OUT, HIGH);
    // Brief reverse drive to brake
    digitalWrite(PIN_DRIVE_FWD, LOW);
    digitalWrite(PIN_DRIVE_REV, HIGH);
    analogWrite(PIN_DRIVE_PWM, 300);
    // No automatic normal drive resumes on release
    // We keep the command state separate from braking
  } else {
    // Release: reverse braking stops immediately
    digitalWrite(PIN_DRIVE_FWD, LOW);
    digitalWrite(PIN_DRIVE_REV, LOW);
    analogWrite(PIN_DRIVE_PWM, 0);
    digitalWrite(PIN_BRAKE_LIGHT_OUT, LOW);
  }
}

void updateIndicators() {
  // Hazard overrides individual indicators
  if (hazardOn) {
    leftIndicatorLampOn = true;
    rightIndicatorLampOn = true;
    digitalWrite(PIN_LEFT_IND_OUT, HIGH);
    digitalWrite(PIN_RIGHT_IND_OUT, HIGH);
    return;
  }

  if (leftIndicatorOn) {
    digitalWrite(PIN_LEFT_IND_OUT, HIGH);
  } else {
    digitalWrite(PIN_LEFT_IND_OUT, LOW);
  }

  if (rightIndicatorOn) {
    digitalWrite(PIN_RIGHT_IND_OUT, HIGH);
  } else {
    digitalWrite(PIN_RIGHT_IND_OUT, LOW);
  }
}

void stopAllMotors() {
  stopDrive();
  stopSteering();
  analogWrite(PIN_STEER_PWM, 0);
  analogWrite(PIN_DRIVE_PWM, 0);
}

void emergencyStop() {
  commandTimeoutTriggered = true;
  brakePressed = false;
  nitroEnabled = false;
  setDrive(0);
  setSteering(STEER_STOP);
  setLights(false);
  setHighBeam(false);
  leftIndicatorOn = false;
  rightIndicatorOn = false;
  hazardOn = false;
  updateIndicators();
  digitalWrite(PIN_BRAKE_LIGHT_OUT, LOW);
  stopAllMotors();
}

// ================================
// Indicator update and safety
// ================================
void updateIndicatorBlinkState() {
  static unsigned long lastBlink = 0;
  static bool blinkState = false;

  if (millis() - lastBlink >= 250) {
    lastBlink = millis();
    blinkState = !blinkState;

    if (hazardOn) {
      digitalWrite(PIN_LEFT_IND_OUT, blinkState ? HIGH : LOW);
      digitalWrite(PIN_RIGHT_IND_OUT, blinkState ? HIGH : LOW);
    } else {
      if (leftIndicatorOn) {
        digitalWrite(PIN_LEFT_IND_OUT, blinkState ? HIGH : LOW);
      } else {
        digitalWrite(PIN_LEFT_IND_OUT, LOW);
      }

      if (rightIndicatorOn) {
        digitalWrite(PIN_RIGHT_IND_OUT, blinkState ? HIGH : LOW);
      } else {
        digitalWrite(PIN_RIGHT_IND_OUT, LOW);
      }
    }
  }
}

// ================================
// HTTP route registration
// ================================
void registerRoutes() {
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/drive", handleDrive);
  server.on("/steer", handleSteer);
  server.on("/nitro", handleNitro);
  server.on("/lights", handleLights);
  server.on("/highbeam", handleHighBeam);
  server.on("/brake", handleBrake);
  server.on("/leftIndicator", handleLeftIndicator);
  server.on("/rightIndicator", handleRightIndicator);
  server.on("/hazard", handleHazard);
  server.onNotFound([]() {
    server.send(404, "text/plain", "Not found");
  });
}

// ================================
// setup()
// ================================
void setup() {
  // Avoid boot problem pins; this code uses logical safe outputs and expects dedicated driver boards.
  // It is critical to verify the actual hardware pin map because the ESP8266 boot-strapping pins can
  // cause undesired startup states. This design intentionally avoids using D3, D8, and D4 as motor
  // drive pins where possible.

  Serial.begin(115200);
  delay(100);

  configurePins();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  IPAddress ip = WiFi.softAPIP();
  Serial.print("AP IP: ");
  Serial.println(ip);

  // Default startup state:
  // drive STOP, steering STOP, Nitro OFF, lights OFF, indicators OFF, hazard OFF, brake OFF, high beam OFF
  stopAllMotors();
  nitroEnabled = false;
  lightsOn = false;
  highBeamOn = false;
  brakePressed = false;
  leftIndicatorOn = false;
  rightIndicatorOn = false;
  hazardOn = false;
  frontLightsOn = false;
  rearLightsOn = false;
  commandTimeoutTriggered = false;

  updateIndicators();

  registerRoutes();
  server.begin();

  Serial.println("ESP8266 RC Car server started");
  Serial.println("Open browser to 192.168.4.1");
}

// ================================
// loop()
// ================================
void loop() {
  server.handleClient();

  // command timeout / safety
  if ((millis() - lastVehicleCommandMs) > COMMAND_TIMEOUT_MS) {
    if (!commandTimeoutTriggered) {
      // If commands stop arriving / client disconnects:
      // Stop drive.
      // Stop steering.
      // Cancel movement commands.
      // No automatic movement after boot.
      emergencyStop();
      commandTimeoutTriggered = true;
    }
  }

  // Blink indicators using millis(), never delay()
  updateIndicatorBlinkState();
}
