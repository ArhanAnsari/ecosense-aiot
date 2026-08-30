#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <esp_wifi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// --- Hardware Pin Configurations ---
#define LED_PIN       2     // ESP32 Diagnostic Status LED
#define SERVO_PIN     26    // Tower Pro SG90 PWM (Vent Flap / Emergency Valve)
#define RELAY_PIN     27    // 5V Relay Actuator (Exhaust Fan Control)
#define MQ2_PIN       34    // MQ-2 ADC Channel (Combustible Gases / Smoke)
#define MQ135_PIN     35    // MQ-135 ADC Channel (Air Quality / CO2 / NH3)

Servo ventServo;

// --- Access Point Credentials ---
const char *AP_SSID = "EcoSense-AIoT";
const char *AP_PASS = "EcoSafe2026";

WebServer server(80);

// --- Tuning & Calibration Constants ---
const int MQ2_MARGIN = 300;                     // Delta above baseline for gas hazard
const int MQ135_MARGIN = 250;                   // Delta above baseline for AQI alert
const int HYSTERESIS = 50;                      // Solid deadband buffer to stop rapid cycling
const unsigned long SAMPLE_INTERVAL_MS = 100;   // 10 Hz telemetry loop
const unsigned long WARMUP_DURATION_MS = 30000; // 30s thermal baseline acquisition

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

// Staggered Actuation Timers (Prevents Current Surge)
unsigned long hazardTriggerTimestamp = 0;
bool pendingServoMove = false;

// --- Actuator Angle Mapping ---
const int SERVO_CLOSED_ANGLE = 0;
const int SERVO_OPEN_ANGLE = 90;
int currentServoAngle = SERVO_CLOSED_ANGLE;

unsigned long lastSampleTime = 0;
unsigned long lastBlinkTime = 0;
bool ledState = false;

// 32-sample burst oversampling filter
int getCleanADC(int pin) {
  long sum = 0;
  for (int i = 0; i < 32; i++) {
    sum += analogRead(pin);
    delayMicroseconds(50);
  }
  return (int)(sum / 32);
}

// Modern Glassmorphism Dashboard UI
const char PAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
  <meta name="apple-mobile-web-app-capable" content="yes">
  <meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
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
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; -webkit-tap-highlight-color: transparent; }
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

    .stream-box { padding: 14px; }
    .stream-head { display: flex; justify-content: space-between; align-items: center; margin-bottom: 8px; }
    .export-link { background: none; border: none; color: var(--accent-cyan); font-size: 0.72rem; font-weight: 700; cursor: pointer; text-decoration: underline; }
    #eventStream { height: 75px; overflow-y: auto; display: flex; flex-direction: column-reverse; gap: 4px; font-family: ui-monospace, SFMono-Regular, monospace; font-size: 0.7rem; }
    .stream-row { padding-bottom: 3px; border-bottom: 1px solid var(--card-border); }
    .stream-alert { color: var(--danger); font-weight: bold; }
    .stream-ok { color: var(--success); }
    .stream-info { color: var(--accent-cyan); }

    .action-row { display: flex; gap: 10px; }
    .btn-action { width: 100%; padding: 14px; border-radius: 18px; font-size: 0.85rem; font-weight: 700; border: none; cursor: pointer; transition: transform 0.1s ease; }
    .btn-action:active { transform: scale(0.98); }
    .btn-arm { background: linear-gradient(135deg, var(--accent-cyan), var(--accent-indigo)); color: #030712; }
    .btn-mute { background: rgba(239, 68, 68, 0.2); color: var(--danger); border: 1px solid rgba(239, 68, 68, 0.4); display: none; }
    .btn-muted-active { background: var(--card-bg); color: var(--text-sub); border-color: var(--card-border); }

    @keyframes pulseAlert { 0%, 100% { opacity: 1; transform: scale(1); } 50% { opacity: 0.85; transform: scale(0.995); } }
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
      <canvas id="telemetryCanvas" width="380" height="110"></canvas>
    </div>

    <div class="card stream-box">
      <div class="stream-head">
        <div class="card-label" style="margin:0;">Event Stream</div>
        <button class="export-link" onclick="downloadCSV()">Export CSV</button>
      </div>
      <div id="eventStream">
        <div class="stream-row stream-info">Telemetry link connected...</div>
      </div>
    </div>

    <div class="action-row">
      <button id="armBtn" class="btn-action btn-arm" onclick="armPhoneAudio()">Tap to Arm Siren Alert</button>
      <button id="muteBtn" class="btn-action btn-mute" onclick="toggleAudioMute()">Mute Siren</button>
    </div>
  </div>

  <script>
    let audioCtx = null, sirenOsc = null, lfoOsc = null, gainNode = null;
    let isMuted = false;
    let hadHazard = false;
    let isLight = false;
    let warmupLogged = false;
    const mq2Log = new Array(50).fill(0);
    const mq135Log = new Array(50).fill(0);
    const telemetryRecords = [];

    const canvas = document.getElementById('telemetryCanvas');
    const ctx = canvas.getContext('2d');

    function toggleTheme() {
      isLight = !isLight;
      document.body.classList.toggle('light-theme', isLight);
      document.getElementById('themeBtn').innerText = isLight ? 'Dark' : 'Light';
      renderChart();
    }

    function armPhoneAudio() {
      if (!audioCtx) {
        audioCtx = new (window.AudioContext || window.webkitAudioContext)();
        document.getElementById('armBtn').style.display = 'none';
        document.getElementById('muteBtn').style.display = 'block';
        pushEvent('Phone audio siren armed', 'info');
      }
    }

    function toggleAudioMute() {
      isMuted = !isMuted;
      const btn = document.getElementById('muteBtn');
      if (isMuted) {
        btn.innerText = 'Unmute Siren';
        btn.classList.add('btn-muted-active');
        stopSiren();
        pushEvent('Siren muted by user', 'info');
      } else {
        btn.innerText = 'Mute Siren';
        btn.classList.remove('btn-muted-active');
        pushEvent('Siren unmuted', 'info');
      }
    }

    function pushEvent(msg, type) {
      const stream = document.getElementById('eventStream');
      const time = new Date().toLocaleTimeString();
      const div = document.createElement('div');
      let typeClass = 'stream-ok';
      if (type === 'alert') typeClass = 'stream-alert';
      else if (type === 'info') typeClass = 'stream-info';
      div.className = 'stream-row ' + typeClass;
      div.innerText = '[' + time + '] ' + msg;
      stream.prepend(div);
      if (stream.children.length > 25) stream.removeChild(stream.lastChild);
    }

    function downloadCSV() {
      if (!telemetryRecords.length) return alert('Collecting data...');
      let csv = 'Timestamp,MQ2_Raw,MQ2_Base,MQ2_Thrs,MQ135_Raw,MQ135_Base,MQ135_Thrs,Vent_Angle,Hazard_State\n';
      telemetryRecords.forEach(r => {
        csv += `${r.t},${r.m2},${r.m2b},${r.m2t},${r.m135},${r.m135b},${r.m135t},${r.ang},${r.hz}\n`;
      });
      const a = document.createElement('a');
      a.href = 'data:text/csv;charset=utf-8,' + encodeURI(csv);
      a.download = `ecosense_hub_${Date.now()}.csv`;
      document.body.appendChild(a);
      a.click();
      document.body.removeChild(a);
    }

    function triggerSiren(active, ratio) {
      if (!audioCtx || isMuted) {
        if (isMuted) stopSiren();
        return;
      }
      if (active) {
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
        let baseFreq = 650 + (ratio * 400);
        sirenOsc.frequency.setTargetAtTime(baseFreq, audioCtx.currentTime, 0.05);
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

    function renderChart() {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.strokeStyle = isLight ? 'rgba(0,0,0,0.06)' : 'rgba(255,255,255,0.06)';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(0, canvas.height / 2);
      ctx.lineTo(canvas.width, canvas.height / 2);
      ctx.stroke();

      function drawTrace(arr, col) {
        ctx.strokeStyle = col;
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

      drawTrace(mq2Log, isLight ? '#0891b2' : '#06b6d4');
      drawTrace(mq135Log, isLight ? '#6366f1' : '#818cf8');
    }

    setInterval(() => {
      fetch('/data').then(r => r.json()).then(d => {
        document.getElementById('valMQ2').innerText = d.mq2;
        document.getElementById('baseMQ2').innerText = d.mq2_b;
        document.getElementById('thrsMQ2').innerText = d.mq2_t;

        document.getElementById('valMQ135').innerText = d.mq135;
        document.getElementById('baseMQ135').innerText = d.mq135_b;
        document.getElementById('thrsMQ135').innerText = d.mq135_t;

        mq2Log.push(d.mq2); mq2Log.shift();
        mq135Log.push(d.mq135); mq135Log.shift();
        renderChart();

        if (!d.warmup) {
          telemetryRecords.push({
            t: new Date().toLocaleTimeString(),
            m2: d.mq2, m2b: d.mq2_b, m2t: d.mq2_t,
            m135: d.mq135, m135b: d.mq135_b, m135t: d.mq135_t,
            ang: d.servo_ang,
            hz: (d.al_mq2 || d.al_mq135) ? 1 : 0
          });
          if (telemetryRecords.length > 300) telemetryRecords.shift();
        }

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
          triggerSiren(false, 0);
        } else {
          warmupBarContainer.style.display = 'none';

          if (!warmupLogged) {
            pushEvent('Sensors armed and baseline established', 'info');
            warmupLogged = true;
          }

          if (d.al_mq2 || d.al_mq135) {
            badge.className = 'status-panel status-hazard';
            statusText.innerText = d.al_mq2 ? 'HAZARD: COMBUSTIBLE GAS DETECTED' : 'WARNING: AIR CONTAMINATION';
            pillFan.className = 'pill pill-active';
            pillVent.className = 'pill pill-active';
            labelVent.innerText = 'Vent: OPEN (' + d.servo_ang + '°)';

            let ratio = d.al_mq2 ? (d.mq2 - d.mq2_t) / (4095 - d.mq2_t) : (d.mq135 - d.mq135_t) / (4095 - d.mq135_t);
            triggerSiren(true, Math.min(Math.max(ratio, 0), 1));

            if (!hadHazard) {
              pushEvent(d.al_mq2 ? 'MQ-2 Gas spike threshold breached' : 'MQ-135 Air contamination breached', 'alert');
              pushEvent('Actuators fired: Relay ON, Vent OPEN (' + d.servo_ang + '°)', 'info');
              hadHazard = true;
            }
          } else {
            badge.className = 'status-panel status-normal';
            statusText.innerText = 'SYSTEM NORMAL - ATMOSPHERE CLEAN';
            pillFan.className = 'pill pill-idle';
            pillVent.className = 'pill pill-idle';
            labelVent.innerText = 'Vent: CLOSED (' + d.servo_ang + '°)';
            triggerSiren(false, 0);

            if (hadHazard) {
              pushEvent('Atmosphere returned to safe baseline', 'ok');
              pushEvent('Actuators idle: Relay OFF, Vent CLOSED (' + d.servo_ang + '°)', 'info');
              hadHazard = false;
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
  // Disable brownout detector to prevent sudden resets during relay/servo spikes
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(300);

  pinMode(LED_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);

  digitalWrite(LED_PIN, HIGH);
  digitalWrite(RELAY_PIN, LOW);

  // Configure continuous full-power Wi-Fi AP
  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP);
  delay(100);
  WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 4);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);

  // Servo Timer Allocation
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  ventServo.setPeriodHertz(50);
  ventServo.attach(SERVO_PIN, 500, 2400);

  // Initialize Servo cleanly at 0 degrees
  currentServoAngle = SERVO_CLOSED_ANGLE;
  ventServo.write(currentServoAngle);

  // ADC Attenuation
  analogSetPinAttenuation(MQ2_PIN, ADC_11db);
  analogSetPinAttenuation(MQ135_PIN, ADC_11db);

  Serial.println("\n==========================================");
  Serial.println("     EcoSense AIoT Safety Station Hub     ");
  Serial.println("==========================================");
  Serial.print("Access Point:  ");
  Serial.println(AP_SSID);
  Serial.print("Dashboard URL: http://");
  Serial.println(WiFi.softAPIP());
  Serial.println("------------------------------------------");

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
}

void loop() {
  server.handleClient();
  unsigned long currentMillis = millis();

  // 30-Second Thermal Stabilization
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
      Serial.println("\n>>> SENSORS STABILIZED: BASELINES LOCKED <<<");
    }
  }

  // 10 Hz Telemetry & Safety Control Loop
  if (currentMillis - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = currentMillis;

    liveMQ2 = getCleanADC(MQ2_PIN);
    liveMQ135 = getCleanADC(MQ135_PIN);

    if (!isWarmingUp) {
      // Clean air adaptive baseline drift tracking
      if (!isAlarmMQ2 && liveMQ2 > 20) {
        baseMQ2 = (0.99 * baseMQ2) + (0.01 * liveMQ2);
        thrsMQ2 = (int)baseMQ2 + MQ2_MARGIN;
      }
      if (!isAlarmMQ135 && liveMQ135 > 20) {
        baseMQ135 = (0.99 * baseMQ135) + (0.01 * liveMQ135);
        thrsMQ135 = (int)baseMQ135 + MQ135_MARGIN;
      }

      // Hysteresis threshold logic
      if (!isAlarmMQ2 && liveMQ2 > thrsMQ2) isAlarmMQ2 = true;
      else if (isAlarmMQ2 && liveMQ2 < (thrsMQ2 - HYSTERESIS)) isAlarmMQ2 = false;

      if (!isAlarmMQ135 && liveMQ135 > thrsMQ135) isAlarmMQ135 = true;
      else if (isAlarmMQ135 && liveMQ135 < (thrsMQ135 - HYSTERESIS)) isAlarmMQ135 = false;
    }

    currentHazardState = (isAlarmMQ2 || isAlarmMQ135);

    // Staggered Actuation Logic (Splits Power Draw)
    if (currentHazardState != lastHazardState) {
      if (currentHazardState) {
        digitalWrite(RELAY_PIN, HIGH);
        hazardTriggerTimestamp = currentMillis;
        pendingServoMove = true;
        Serial.println(">>> HAZARD TRIGGERED: RELAY ON (SERVO DELAYED 250ms) <<<");
      } else {
        digitalWrite(RELAY_PIN, LOW);
        currentServoAngle = SERVO_CLOSED_ANGLE;
        ventServo.write(currentServoAngle);
        pendingServoMove = false;
        Serial.println(">>> HAZARD CLEARED: RELAY OFF & SERVO CLOSED <<<");
      }
      lastHazardState = currentHazardState;
    }

    // Execute delayed servo rotation after relay inrush settles
    if (pendingServoMove && (currentMillis - hazardTriggerTimestamp >= 250)) {
      currentServoAngle = SERVO_OPEN_ANGLE;
      ventServo.write(currentServoAngle);
      pendingServoMove = false;
      Serial.println(">>> SERVO OPENED (90 DEG) <<<");
    }

    // Serial Telemetry
    Serial.print("MQ2:"); Serial.print(liveMQ2);
    Serial.print("\tMQ2_Thrs:"); Serial.print(thrsMQ2);
    Serial.print("\tMQ135:"); Serial.print(liveMQ135);
    Serial.print("\tMQ135_Thrs:"); Serial.print(thrsMQ135);
    Serial.print("\tServo_Angle:"); Serial.print(currentServoAngle);
    Serial.print("\tWarmup_Left:"); Serial.print(isWarmingUp ? warmupSecondsLeft : 0);
    Serial.print("\tHazard:"); Serial.println(currentHazardState ? 1 : 0);
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