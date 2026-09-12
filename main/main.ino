#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// --- Hardware Pin Configurations ---
#define LED_PIN       2     // ESP32 Diagnostic Status LED
#define SERVO_PIN     26    // Tower Pro SG90 PWM (Vent Flap / Emergency Valve)
#define RELAY_PIN     27    // 5V Relay Actuator (Exhaust Fan Control)
#define MQ2_PIN       34    // MQ-2 ADC Channel (Combustible Gases / Smoke)
#define MQ135_PIN     35    // MQ-135 ADC Channel (Air Quality / CO2 / NH3)

// --- Custom I2C Pins for OLED ---
#define I2C_SDA       33
#define I2C_SCL       32

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1

Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Servo ventServo;

bool oledAvailable = false;

// --- Access Point Credentials ---
const char *AP_SSID = "EcoSense-AIoT";
const char *AP_PASS = "EcoSafe2026";

WebServer server(80);

// --- Tuning & Calibration Constants ---
const int MQ2_MARGIN = 300;
const int MQ135_MARGIN = 250;
const int HYSTERESIS = 50;
const unsigned long SAMPLE_INTERVAL_MS = 100;
const unsigned long WARMUP_DURATION_MS = 30000;
const unsigned long OLED_REFRESH_MS = 250;

// --- Dynamic Calibration & State Variables ---
int liveMQ2 = 0, liveMQ135 = 0;
float baseMQ2 = 0.0, baseMQ135 = 0.0;
int thrsMQ2 = 0, thrsMQ135 = 0;

bool isWarmingUp = true;
int warmupSecondsLeft = 30;
bool isAlarmMQ2 = false;
bool isAlarmMQ135 = false;
bool currentHazardState = false;
bool lastHazardState = false;

// Staggered Actuation Timers
unsigned long hazardTriggerTimestamp = 0;
bool pendingServoMove = false;

// --- Actuator Angle Mapping ---
const int SERVO_CLOSED_ANGLE = 0;
const int SERVO_OPEN_ANGLE = 90;
int currentServoAngle = SERVO_CLOSED_ANGLE;

unsigned long lastSampleTime = 0;
unsigned long lastBlinkTime = 0;
unsigned long lastOledTime = 0;
bool ledState = false;

int getCleanADC(int pin) {
  long sum = 0;
  for (int i = 0; i < 32; i++) {
    sum += analogRead(pin);
    delayMicroseconds(50);
  }
  return (int)(sum / 32);
}

// Visual HUD Engine for 128x64 OLED
void updateOLEDDisplay() {
  if (!oledAvailable) return;

  oled.clearDisplay();

  static bool flashToggle = false;
  flashToggle = !flashToggle;

  // --- 1. HEADER HUD BAR (Y: 0 to 11) ---
  if (currentHazardState && flashToggle) {
    oled.fillRect(0, 0, 128, 12, SSD1306_WHITE);
    oled.setTextColor(SSD1306_BLACK);
    oled.setTextSize(1);
    oled.setCursor(8, 2);
    oled.print("! HAZARD ALERT !");
  } else {
    oled.fillRect(0, 0, 128, 12, SSD1306_WHITE);
    oled.setTextColor(SSD1306_BLACK);
    oled.setTextSize(1);
    oled.setCursor(3, 2);
    oled.print("EcoSense AIoT");
    oled.setCursor(94, 2);
    oled.print(isWarmingUp ? "WARM" : (currentHazardState ? "ALRT" : "SAFE"));
  }

  // --- 2. MAIN CENTER BODY (Y: 15 to 42) ---
  if (isWarmingUp) {
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);
    oled.setCursor(18, 16);
    oled.print("HEATER CALIBRATION");

    // Centered Big Countdown Number
    oled.setTextSize(2);
    oled.setCursor(48, 27);
    if (warmupSecondsLeft < 10) oled.print("0");
    oled.print(warmupSecondsLeft);
    oled.setTextSize(1);
    oled.print("s");

    // Rounded Progress Fill Bar
    oled.drawRoundRect(14, 46, 100, 8, 2, SSD1306_WHITE);
    int barW = map(constrain(30 - warmupSecondsLeft, 0, 30), 0, 30, 0, 96);
    if (barW > 0) {
      oled.fillRect(16, 48, barW, 4, SSD1306_WHITE);
    }
  } 
  else {
    oled.setTextColor(SSD1306_WHITE);

    // Row 1: MQ-2 Gas Readout & Status
    oled.setTextSize(1);
    oled.setCursor(4, 16);
    oled.print("GAS: ");
    oled.setTextSize(1);
    oled.print(liveMQ2);
    oled.print(" / ");
    oled.print(thrsMQ2);
    if (isAlarmMQ2) {
      oled.setCursor(102, 16);
      oled.print("WARN");
    }

    // Row 2: MQ-135 Air Quality Readout & Status
    oled.setCursor(4, 28);
    oled.print("AQI: ");
    oled.print(liveMQ135);
    oled.print(" / ");
    oled.print(thrsMQ135);
    if (isAlarmMQ135) {
      oled.setCursor(102, 28);
      oled.print("POOR");
    }

    // Horizontal Divider
    oled.drawFastHLine(0, 40, 128, SSD1306_WHITE);

    // --- 3. BOTTOM ACTUATOR DECK (Y: 44 to 62) ---
    // Fan Pill
    if (currentHazardState) {
      oled.fillRoundRect(2, 45, 60, 17, 3, SSD1306_WHITE);
      oled.setTextColor(SSD1306_BLACK);
      oled.setCursor(6, 50);
      oled.print("FAN: ON");
    } else {
      oled.setTextColor(SSD1306_WHITE);
      oled.drawRoundRect(2, 45, 60, 17, 3, SSD1306_WHITE);
      oled.setCursor(6, 50);
      oled.print("FAN: OFF");
    }

    // Vent Valve Pill
    if (currentServoAngle > 0) {
      oled.fillRoundRect(66, 45, 60, 17, 3, SSD1306_WHITE);
      oled.setTextColor(SSD1306_BLACK);
      oled.setCursor(70, 50);
      oled.print("VENT: 90");
      oled.print((char)247);
    } else {
      oled.setTextColor(SSD1306_WHITE);
      oled.drawRoundRect(66, 45, 60, 17, 3, SSD1306_WHITE);
      oled.setCursor(70, 50);
      oled.print("VENT:  0");
      oled.print((char)247);
    }
  }

  oled.display();
}

// Complete Responsive Glassmorphism Mobile Dashboard
const char PAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
  <meta name="apple-mobile-web-app-capable" content="yes">
  <meta name="theme-color" content="#030712">
  <title>EcoSense AIoT Safety Station</title>
  <style>
    :root {
      --bg: #030712;
      --card-bg: rgba(17, 24, 39, 0.75);
      --card-border: rgba(255, 255, 255, 0.08);
      --text-main: #f9fafb;
      --text-sub: #9ca3af;
      --accent-cyan: #06b6d4;
      --accent-indigo: #818cf8;
      --danger: #ef4444;
      --warning: #f59e0b;
      --success: #10b981;
      --chart-bg: #0b0f19;
    }
    .light-theme {
      --bg: #f3f4f6;
      --card-bg: rgba(255, 255, 255, 0.85);
      --card-border: rgba(0, 0, 0, 0.08);
      --text-main: #111827;
      --text-sub: #6b7280;
      --accent-cyan: #0891b2;
      --accent-indigo: #6366f1;
      --chart-bg: #e5e7eb;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; -webkit-tap-highlight-color: transparent; }
    body { background: var(--bg); color: var(--text-main); display: flex; justify-content: center; min-height: 100vh; padding: 16px; transition: background 0.3s ease; }
    .hub-container { width: 100%; max-width: 440px; display: flex; flex-direction: column; gap: 14px; }
    .nav-bar { display: flex; justify-content: space-between; align-items: center; padding: 6px 2px; }
    .brand { display: flex; align-items: center; gap: 8px; }
    .brand-icon { width: 12px; height: 12px; border-radius: 50%; background: var(--accent-cyan); box-shadow: 0 0 10px var(--accent-cyan); }
    .brand-title { font-size: 1.15rem; font-weight: 800; letter-spacing: -0.5px; background: linear-gradient(135deg, var(--accent-cyan), var(--accent-indigo)); -webkit-background-clip: text; -webkit-text-fill-color: transparent; }
    .btn-icon { background: var(--card-bg); border: 1px solid var(--card-border); color: var(--text-main); padding: 6px 14px; border-radius: 20px; font-size: 0.75rem; font-weight: 600; cursor: pointer; backdrop-filter: blur(10px); }

    .status-panel { border-radius: 18px; padding: 14px; text-align: center; font-weight: 700; font-size: 0.85rem; letter-spacing: 0.5px; backdrop-filter: blur(12px); border: 1px solid transparent; transition: all 0.3s ease; display: flex; flex-direction: column; gap: 6px; align-items: center; }
    .status-warmup { background: rgba(245, 158, 11, 0.12); color: var(--warning); border-color: rgba(245, 158, 11, 0.3); }
    .status-normal { background: rgba(16, 185, 129, 0.12); color: var(--success); border-color: rgba(16, 185, 129, 0.3); }
    .status-hazard { background: rgba(239, 68, 68, 0.15); color: var(--danger); border-color: rgba(239, 68, 68, 0.4); animation: pulseAlert 1.2s infinite; }
    
    .warmup-bar-wrap { width: 100%; height: 6px; background: rgba(255, 255, 255, 0.1); border-radius: 999px; overflow: hidden; margin-top: 4px; }
    .warmup-bar-fill { height: 100%; width: 0%; background: var(--warning); border-radius: 999px; transition: width 0.3s ease; }

    .grid-metrics { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
    .card { background: var(--card-bg); border: 1px solid var(--card-border); border-radius: 22px; padding: 16px; backdrop-filter: blur(16px); box-shadow: 0 8px 30px rgba(0,0,0,0.12); }
    .card-label { font-size: 0.7rem; font-weight: 700; text-transform: uppercase; color: var(--text-sub); letter-spacing: 0.5px; margin-bottom: 4px; }
    .card-value { font-size: 2.2rem; font-weight: 800; letter-spacing: -1px; line-height: 1.1; margin: 4px 0; }
    .val-mq2 { color: var(--accent-cyan); }
    .val-mq135 { color: var(--accent-indigo); }
    .card-subtext { font-size: 0.7rem; color: var(--text-sub); display: flex; justify-content: space-between; margin-top: 6px; padding-top: 6px; border-top: 1px solid var(--card-border); }

    .actuators-card { display: flex; justify-content: space-between; align-items: center; padding: 12px 18px; }
    .act-item { display: flex; align-items: center; gap: 10px; font-size: 0.78rem; font-weight: 700; }
    .pill { display: inline-block; width: 8px; height: 8px; border-radius: 50%; }
    .pill-active { background: var(--danger); box-shadow: 0 0 8px var(--danger); }
    .pill-idle { background: var(--text-sub); opacity: 0.4; }

    .chart-box { padding: 14px; }
    .chart-head { display: flex; justify-content: space-between; align-items: center; margin-bottom: 8px; }
    .chart-legend { display: flex; gap: 12px; font-size: 0.7rem; font-weight: 700; }
    canvas { width: 100%; height: 110px; background: var(--chart-bg); border-radius: 12px; display: block; }

    .log-card { background: var(--card-bg); border: 1px solid var(--card-border); border-radius: 22px; padding: 14px; }
    .log-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 8px; }
    .log-title { font-size: 0.72rem; font-weight: 700; color: var(--text-sub); text-transform: uppercase; letter-spacing: 0.5px; }
    .export-btn { background: transparent; border: 1px solid var(--card-border); color: var(--accent-cyan); padding: 3px 8px; border-radius: 6px; font-size: 0.7rem; font-weight: 700; cursor: pointer; }
    #logContainer { height: 85px; overflow-y: auto; font-family: monospace; font-size: 0.72rem; color: var(--text-main); display: flex; flex-direction: column-reverse; gap: 3px; }
    .log-entry { border-bottom: 1px solid var(--card-border); padding-bottom: 2px; }
    .log-alert { color: var(--danger); font-weight: bold; }
    .log-safe { color: var(--success); }

    .action-row { display: flex; gap: 10px; }
    .btn-action { width: 100%; padding: 14px; border-radius: 18px; font-size: 0.85rem; font-weight: 700; border: none; cursor: pointer; transition: transform 0.1s ease; }
    .btn-action:active { transform: scale(0.98); }
    .btn-arm { background: linear-gradient(135deg, var(--accent-cyan), var(--accent-indigo)); color: #030712; }
    .btn-mute { background: rgba(239, 68, 68, 0.2); color: var(--danger); border: 1px solid rgba(239, 68, 68, 0.4); display: none; }
    .btn-muted-state { background: var(--card-bg); color: var(--text-sub); border-color: var(--card-border); }

    @keyframes pulseAlert { 0%, 100% { opacity: 1; transform: scale(1); } 50% { opacity: 0.85; } }
  </style>
</head>
<body>
  <div class="hub-container">
    <div class="nav-bar">
      <div class="brand">
        <div class="brand-icon"></div>
        <div class="brand-title">EcoSense AIoT Hub</div>
      </div>
      <button class="btn-icon" onclick="toggleTheme()" id="themeBtn">Light</button>
    </div>

    <div id="statusBadge" class="status-panel status-warmup">
      <span id="statusText">CALIBRATING HEATERS (30s)</span>
      <div id="warmupBarContainer" class="warmup-bar-wrap">
        <div id="warmupBar" class="warmup-bar-fill"></div>
      </div>
    </div>

    <div class="grid-metrics">
      <div class="card">
        <div class="card-label">MQ-2 Gas / Smoke</div>
        <div class="card-value val-mq2" id="valMQ2">--</div>
        <div class="card-subtext">
          <span>Base: <b id="baseMQ2">--</b></span>
          <span>Thr: <b id="thrsMQ2">--</b></span>
        </div>
      </div>
      <div class="card">
        <div class="card-label">MQ-135 Air Quality</div>
        <div class="card-value val-mq135" id="valMQ135">--</div>
        <div class="card-subtext">
          <span>Base: <b id="baseMQ135">--</b></span>
          <span>Thr: <b id="thrsMQ135">--</b></span>
        </div>
      </div>
    </div>

    <div class="card actuators-card">
      <div class="act-item">
        <span id="pillFan" class="pill pill-idle"></span>
        <span>Exhaust Fan (Relay)</span>
      </div>
      <div class="act-item">
        <span id="pillVent" class="pill pill-idle"></span>
        <span id="labelVent">Vent: CLOSED (0°)</span>
      </div>
    </div>

    <div class="card chart-box">
      <div class="chart-head">
        <div class="card-label" style="margin:0;">Live Telemetry Plot</div>
        <div class="chart-legend">
          <span style="color:var(--accent-cyan);">&#9632; MQ-2</span>
          <span style="color:var(--accent-indigo);">&#9632; MQ-135</span>
        </div>
      </div>
      <canvas id="liveChart" width="380" height="110"></canvas>
    </div>

    <div class="log-card">
      <div class="log-header">
        <span class="log-title">System Event Stream</span>
        <button class="export-btn" onclick="exportCSV()">Download CSV</button>
      </div>
      <div id="logContainer">
        <div class="log-entry log-safe">Stabilizing sensors on startup...</div>
      </div>
    </div>

    <div class="action-row">
      <button id="audioBtn" class="btn-action btn-arm" onclick="initAudio()">Tap to Enable Siren</button>
      <button id="muteBtn" class="btn-action btn-mute" onclick="toggleMute()">Mute Siren</button>
    </div>
  </div>

  <script>
    let audioCtx = null, sirenOsc = null, lfoOsc = null, gainNode = null;
    let isMuted = false;
    let lastStateWasAlarm = false;
    let isLightMode = false;
    let lastLoggedWarmupSec = -1;
    const historyMQ2 = new Array(50).fill(0);
    const historyMQ135 = new Array(50).fill(0);
    const csvRecords = [];

    const canvas = document.getElementById('liveChart');
    const ctx = canvas.getContext('2d');

    function toggleTheme() {
      isLightMode = !isLightMode;
      document.body.classList.toggle('light-theme', isLightMode);
      document.getElementById('themeBtn').innerText = isLightMode ? 'Dark' : 'Light';
      drawChart();
    }

    function initAudio() {
      if (!audioCtx) {
        audioCtx = new (window.AudioContext || window.webkitAudioContext)();
        document.getElementById('audioBtn').style.display = 'none';
        document.getElementById('muteBtn').style.display = 'block';
        addLog('Audio alarm system armed & listening', false);
      }
    }

    function toggleMute() {
      isMuted = !isMuted;
      const mBtn = document.getElementById('muteBtn');
      if (isMuted) {
        mBtn.innerText = 'Unmute Siren';
        mBtn.className = 'btn-action btn-mute btn-muted-state';
        stopSiren();
        addLog('Audio alarm muted manually', false);
      } else {
        mBtn.innerText = 'Mute Siren';
        mBtn.className = 'btn-action btn-mute';
        addLog('Audio alarm unmuted', false);
      }
    }

    function addLog(msg, isAlert) {
      const container = document.getElementById('logContainer');
      const time = new Date().toLocaleTimeString();
      const div = document.createElement('div');
      div.className = 'log-entry ' + (isAlert ? 'log-alert' : 'log-safe');
      div.innerText = '[' + time + '] ' + msg;
      container.prepend(div);
      if (container.children.length > 25) container.removeChild(container.lastChild);
    }

    function exportCSV() {
      if (csvRecords.length === 0) { alert('No telemetry recorded yet.'); return; }
      let csvContent = 'data:text/csv;charset=utf-8,Timestamp,MQ2_Raw,MQ2_Base,MQ2_Thrs,MQ135_Raw,MQ135_Base,MQ135_Thrs,Vent_Angle,Alarm_State\n';
      csvRecords.forEach(r => {
        csvContent += `${r.t},${r.mq2},${r.mq2_b},${r.mq2_t},${r.mq135},${r.mq135_b},${r.mq135_t},${r.ang},${r.alm}\n`;
      });
      const link = document.createElement('a');
      link.setAttribute('href', encodeURI(csvContent));
      link.setAttribute('download', `ecosense_telemetry_${Date.now()}.csv`);
      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);
    }

    function setSiren(play, freqRatio) {
      if (!audioCtx || isMuted) {
        if (isMuted) stopSiren();
        return;
      }
      if (play) {
        if (!sirenOsc) {
          sirenOsc = audioCtx.createOscillator();
          lfoOsc = audioCtx.createOscillator();
          const lfoGain = audioCtx.createGain();
          gainNode = audioCtx.createGain();

          sirenOsc.type = 'sawtooth';
          lfoOsc.type = 'sine';
          lfoOsc.frequency.setValueAtTime(2.5, audioCtx.currentTime);
          lfoGain.gain.setValueAtTime(280, audioCtx.currentTime);

          lfoOsc.connect(lfoGain);
          lfoGain.connect(sirenOsc.frequency);
          sirenOsc.connect(gainNode);
          gainNode.connect(audioCtx.destination);
          gainNode.gain.setValueAtTime(0.25, audioCtx.currentTime);

          sirenOsc.start();
          lfoOsc.start();
        }
        let targetFreq = 650 + (freqRatio * 400);
        sirenOsc.frequency.setTargetAtTime(targetFreq, audioCtx.currentTime, 0.05);
      } else {
        stopSiren();
      }
    }

    function stopSiren() {
      if (sirenOsc) {
        try {
          sirenOsc.stop();
          lfoOsc.stop();
          sirenOsc.disconnect();
          lfoOsc.disconnect();
        } catch(e) {}
        sirenOsc = null;
        lfoOsc = null;
      }
    }

    function drawChart() {
      try {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        ctx.strokeStyle = isLightMode ? 'rgba(0,0,0,0.06)' : 'rgba(255,255,255,0.06)';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(0, canvas.height / 2);
        ctx.lineTo(canvas.width, canvas.height / 2);
        ctx.stroke();

        function plotLine(arr, color) {
          ctx.strokeStyle = color;
          ctx.lineWidth = 2;
          ctx.beginPath();
          for (let i = 0; i < arr.length; i++) {
            let x = (i / (arr.length - 1)) * canvas.width;
            let y = canvas.height - ((arr[i] / 4095) * (canvas.height - 10)) - 5;
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
          }
          ctx.stroke();
        }

        plotLine(historyMQ2, isLightMode ? '#0891b2' : '#06b6d4');
        plotLine(historyMQ135, isLightMode ? '#6366f1' : '#818cf8');
      } catch(e) {}
    }

    setInterval(() => {
      fetch('/data').then(r => r.json()).then(d => {
        // Safe DOM Updates
        document.getElementById('valMQ2').innerText = d.mq2;
        document.getElementById('baseMQ2').innerText = d.mq2_b;
        document.getElementById('thrsMQ2').innerText = d.mq2_t;

        document.getElementById('valMQ135').innerText = d.mq135;
        document.getElementById('baseMQ135').innerText = d.mq135_b;
        document.getElementById('thrsMQ135').innerText = d.mq135_t;

        historyMQ2.push(d.mq2); historyMQ2.shift();
        historyMQ135.push(d.mq135); historyMQ135.shift();
        drawChart();

        const badge = document.getElementById('statusBadge');
        const statusText = document.getElementById('statusText');
        const warmupBarContainer = document.getElementById('warmupBarContainer');
        const warmupBar = document.getElementById('warmupBar');
        const pillFan = document.getElementById('pillFan');
        const pillVent = document.getElementById('pillVent');
        const labelVent = document.getElementById('labelVent');

        if (d.warmup) {
          badge.className = 'status-panel status-warmup';
          statusText.innerText = 'CALIBRATING HEATERS (' + d.left + 's remaining)';
          warmupBarContainer.style.display = 'block';
          let progress = Math.min(Math.max(((30 - d.left) / 30) * 100, 0), 100);
          warmupBar.style.width = progress + '%';

          pillFan.className = 'pill pill-idle';
          pillVent.className = 'pill pill-idle';
          labelVent.innerText = 'Vent: CLOSED (' + d.servo_ang + '°)';
          setSiren(false, 0);

          if (d.left % 10 === 0 && d.left !== lastLoggedWarmupSec) {
            addLog('Sensors stabilizing... ' + d.left + 's remaining', false);
            lastLoggedWarmupSec = d.left;
          }
        } else {
          warmupBarContainer.style.display = 'none';

          if (lastLoggedWarmupSec !== 0) {
            addLog('Heater warmup complete. Baselines locked.', false);
            lastLoggedWarmupSec = 0;
          }

          // CSV Telemetry Record
          csvRecords.push({
            t: new Date().toLocaleTimeString(),
            mq2: d.mq2, mq2_b: d.mq2_b, mq2_t: d.mq2_t,
            mq135: d.mq135, mq135_b: d.mq135_b, mq135_t: d.mq135_t,
            ang: d.servo_ang,
            alm: (d.al_mq2 || d.al_mq135) ? 1 : 0
          });
          if (csvRecords.length > 500) csvRecords.shift();

          if (d.al_mq2 || d.al_mq135) {
            badge.className = 'status-panel status-hazard';
            statusText.innerText = d.al_mq2 ? 'HAZARD: COMBUSTIBLE GAS DETECTED' : 'WARNING: CONTAMINATED AIR';
            pillFan.className = 'pill pill-active';
            pillVent.className = 'pill pill-active';
            labelVent.innerText = 'Vent: OPEN (' + d.servo_ang + '°)';

            let ratio = d.al_mq2 ? (d.mq2 - d.mq2_t) / (4095 - d.mq2_t) : (d.mq135 - d.mq135_t) / (4095 - d.mq135_t);
            setSiren(true, Math.min(Math.max(ratio, 0), 1));

            if (!lastStateWasAlarm) {
              addLog(d.al_mq2 ? 'MQ-2 Gas spike threshold breached!' : 'MQ-135 Air contamination breached!', true);
              addLog('Actuators triggered: Relay ON, Vent OPEN (' + d.servo_ang + '°)', true);
              lastStateWasAlarm = true;
            }
          } else {
            badge.className = 'status-panel status-normal';
            statusText.innerText = 'SYSTEM NORMAL - ATMOSPHERE CLEAN';
            pillFan.className = 'pill pill-idle';
            pillVent.className = 'pill pill-idle';
            labelVent.innerText = 'Vent: CLOSED (' + d.servo_ang + '°)';
            setSiren(false, 0);

            if (lastStateWasAlarm) {
              addLog('Atmosphere normalized back below thresholds', false);
              addLog('Actuators idle: Relay OFF, Vent CLOSED (' + d.servo_ang + '°)', false);
              lastStateWasAlarm = false;
            }
          }
        }
      }).catch(() => {});
    }, 150);
  </script>
</body>
</html>
)rawliteral";

void handleRoot() { server.send(200, "text/html", PAGE_HTML); }

void handleData() {
  String j = "{";
  j += "\"mq2\":" + String(liveMQ2) + ",";
  j += "\"mq2_b\":" + String((int)baseMQ2) + ",";
  j += "\"mq2_t\":" + String(thrsMQ2) + ",";
  j += "\"mq135\":" + String(liveMQ135) + ",";
  j += "\"mq135_b\":" + String((int)baseMQ135) + ",";
  j += "\"mq135_t\":" + String(thrsMQ135) + ",";
  j += "\"warmup\":" + String(isWarmingUp ? "true" : "false") + ",";
  j += "\"left\":" + String(warmupSecondsLeft) + ",";
  j += "\"servo_ang\":" + String(currentServoAngle) + ",";
  j += "\"al_mq2\":" + String(isAlarmMQ2 ? "true" : "false") + ",";
  j += "\"al_mq135\":" + String(isAlarmMQ135 ? "true" : "false");
  j += "}";
  server.send(200, "application/json", j);
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(300);

  pinMode(LED_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);

  digitalWrite(LED_PIN, HIGH);
  digitalWrite(RELAY_PIN, LOW);

  // Enable internal pullups & configure stable 100 kHz I2C clock
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  Wire.begin(I2C_SDA, I2C_SCL, 100000);
  Wire.setTimeOut(50);

  // Auto-probe OLED addresses
  if (oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    oledAvailable = true;
    Serial.println(F("[OLED] Display verified at address 0x3C"));
  } else if (oled.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
    oledAvailable = true;
    Serial.println(F("[OLED] Display verified at address 0x3D"));
  } else {
    oledAvailable = false;
    Serial.println(F("[OLED] Warning: Screen not detected on GPIO 33/32. Running headless."));
  }

  if (oledAvailable) {
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);
    oled.setCursor(16, 20);
    oled.println(F("EcoSense AIoT Hub"));
    oled.setCursor(24, 38);
    oled.println(F("SYSTEM ONLINE"));
    oled.display();
  }

  // Continuous Full-Power Wi-Fi SoftAP
  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP);
  delay(100);
  WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 4);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);

  // Servo Setup
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  ventServo.setPeriodHertz(50);
  ventServo.attach(SERVO_PIN, 500, 2400);

  currentServoAngle = SERVO_CLOSED_ANGLE;
  ventServo.write(currentServoAngle);

  analogSetPinAttenuation(MQ2_PIN, ADC_11db);
  analogSetPinAttenuation(MQ135_PIN, ADC_11db);

  Serial.println(F("\n=========================================="));
  Serial.println(F("     EcoSense AIoT Safety Station Hub     "));
  Serial.println(F("=========================================="));
  Serial.print(F("Access Point : ")); Serial.println(AP_SSID);
  Serial.print(F("Dashboard URL: http://")); Serial.println(WiFi.softAPIP());
  Serial.println(F("Telemetry Serial Stream Active (115200 Baud)"));
  Serial.println(F("------------------------------------------"));

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
}

void loop() {
  server.handleClient();
  unsigned long currentMillis = millis();

  // 30-Second Thermal Stabilization Routine
  if (isWarmingUp) {
    if (currentMillis < WARMUP_DURATION_MS) {
      warmupSecondsLeft = (int)((WARMUP_DURATION_MS - currentMillis) / 1000) + 1;
    } else {
      isWarmingUp = false;
      liveMQ2 = getCleanADC(MQ2_PIN);
      liveMQ135 = getCleanADC(MQ135_PIN);
      baseMQ2 = (float)liveMQ2;
      baseMQ135 = (float)liveMQ135;
      thrsMQ2 = (int)baseMQ2 + MQ2_MARGIN;
      thrsMQ135 = (int)baseMQ135 + MQ135_MARGIN;
      Serial.println(F("\n>>> [CALIBRATION COMPLETE] Ambient baselines locked. System armed. <<<"));
    }
  }

  // 10 Hz Telemetry & Safety Control Loop
  if (currentMillis - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = currentMillis;

    liveMQ2 = getCleanADC(MQ2_PIN);
    liveMQ135 = getCleanADC(MQ135_PIN);

    if (!isWarmingUp) {
      // Dynamic baseline drift tracking during normal air
      if (!isAlarmMQ2 && liveMQ2 > 20) {
        baseMQ2 = (0.99 * baseMQ2) + (0.01 * liveMQ2);
        thrsMQ2 = (int)baseMQ2 + MQ2_MARGIN;
      }
      if (!isAlarmMQ135 && liveMQ135 > 20) {
        baseMQ135 = (0.99 * baseMQ135) + (0.01 * liveMQ135);
        thrsMQ135 = (int)baseMQ135 + MQ135_MARGIN;
      }

      // Hysteresis threshold comparison
      if (!isAlarmMQ2 && liveMQ2 > thrsMQ2) isAlarmMQ2 = true;
      else if (isAlarmMQ2 && liveMQ2 < (thrsMQ2 - HYSTERESIS)) isAlarmMQ2 = false;

      if (!isAlarmMQ135 && liveMQ135 > thrsMQ135) isAlarmMQ135 = true;
      else if (isAlarmMQ135 && liveMQ135 < (thrsMQ135 - HYSTERESIS)) isAlarmMQ135 = false;
    }

    currentHazardState = (isAlarmMQ2 || isAlarmMQ135);

    // Staggered Actuation Logic
    if (currentHazardState != lastHazardState) {
      if (currentHazardState) {
        digitalWrite(RELAY_PIN, HIGH);
        hazardTriggerTimestamp = currentMillis;
        pendingServoMove = true;
        Serial.println(F(">>> [EVENT: ALARM] Relay ON. Servo vent move queued (+250ms). <<<"));
      } else {
        digitalWrite(RELAY_PIN, LOW);
        currentServoAngle = SERVO_CLOSED_ANGLE;
        ventServo.write(currentServoAngle);
        pendingServoMove = false;
        Serial.println(F(">>> [EVENT: SAFE] Atmosphere normal. Relay OFF, Vent 0 deg. <<<"));
      }
      lastHazardState = currentHazardState;
    }

    if (pendingServoMove && (currentMillis - hazardTriggerTimestamp >= 250)) {
      currentServoAngle = SERVO_OPEN_ANGLE;
      ventServo.write(currentServoAngle);
      pendingServoMove = false;
      Serial.println(F(">>> [ACTUATOR] Servo Vent Flap rotated to 90 deg. <<<"));
    }

    // Continuous Telemetry Output for Arduino Serial Plotter & Monitor
    Serial.print("MQ2:"); Serial.print(liveMQ2);
    Serial.print(" MQ2_Thrs:"); Serial.print(thrsMQ2);
    Serial.print(" MQ135:"); Serial.print(liveMQ135);
    Serial.print(" MQ135_Thrs:"); Serial.print(thrsMQ135);
    Serial.print(" Vent_Angle:"); Serial.print(currentServoAngle);
    Serial.print(" Warmup_Left:"); Serial.print(isWarmingUp ? warmupSecondsLeft : 0);
    Serial.print(" Alarm:"); Serial.println(currentHazardState ? 1 : 0);
  }

  // Refresh OLED Display at 4 Hz
  if (currentMillis - lastOledTime >= OLED_REFRESH_MS) {
    lastOledTime = currentMillis;
    updateOLEDDisplay();
  }

  // Diagnostic Status LED Heartbeat
  if (isWarmingUp || currentHazardState) {
    if (currentMillis - lastBlinkTime >= 100) {
      lastBlinkTime = currentMillis;
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
    }
  } else {
    if (currentMillis - lastBlinkTime >= 500) {
      lastBlinkTime = currentMillis;
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
    }
  }
}