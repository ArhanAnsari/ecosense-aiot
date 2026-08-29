#include <WiFi.h>
#include <WebServer.h>

// --- Pin Definitions ---
#define LED_PIN      2     // Built-in Blue Status LED
#define BUZZER_PIN   14    // Physical Piezo Buzzer
#define MUTE_BTN_PIN 13    // Physical Mute Pushbutton (Uses Internal Pull-up)
#define RELAY_PIN    27    // 5V Relay for Exhaust Fan
#define MQ2_PIN      34    // Combustible Gas / Smoke (GPIO 34)
#define MQ135_PIN    35    // Air Quality / CO2 / Ammonia (GPIO 35)

// --- Standalone Access Point Credentials ---
const char *AP_SSID = "EcoSense-Hub";
const char *AP_PASS = "12345678";

WebServer server(80);

// --- Tuning & Calibration Constants ---
const int MQ2_MARGIN = 300;                     // Delta above baseline for gas alarm
const int MQ135_MARGIN = 250;                   // Delta above baseline for air quality warning
const int HYSTERESIS = 40;                      // Buffer to prevent rapid alarm flapping
const unsigned long SAMPLE_INTERVAL_MS = 100;   // 10 Hz refresh rate
const unsigned long WARMUP_DURATION_MS = 30000; // 30-second thermal stabilization

// --- Sensor Variables ---
int liveMQ2 = 0, liveMQ135 = 0;
float baseMQ2 = 0.0, baseMQ135 = 0.0;
int thrsMQ2 = 0, thrsMQ135 = 0;

bool isWarmingUp = true;
int warmupSecondsLeft = 30;
bool isAlarmMQ2 = false;
bool isAlarmMQ135 = false;

// --- Physical Button State & Debounce ---
bool isHardwareMuted = false;
int lastBtnReading = HIGH;
int stableBtnState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY_MS = 50;

unsigned long lastSampleTime = 0;
unsigned long lastBlinkTime = 0;
bool ledState = false;

// 32-sample hardware averaging filter for stable analog readings
int getCleanADC(int pin) {
  long sum = 0;
  for (int i = 0; i < 32; i++) {
    sum += analogRead(pin);
    delayMicroseconds(50);
  }
  return (int)(sum / 32);
}

// Complete Responsive Dashboard with Theme Switcher, CSV Logging, PWA Meta & Siren Controls
const char PAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
  <meta name="apple-mobile-web-app-capable" content="yes">
  <meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
  <meta name="apple-mobile-web-app-title" content="EcoSense Hub">
  <meta name="theme-color" content="#0b0f19">
  <title>EcoSense AIoT Hub</title>
  <style>
    :root {
      --bg: #0b0f19;
      --card-bg: #161f30;
      --border: #1e293b;
      --text: #f8fafc;
      --text-muted: #94a3b8;
      --accent-mq2: #38bdf8;
      --accent-mq135: #a78bfa;
      --canvas-bg: #0b0f19;
    }
    .light-theme {
      --bg: #f1f5f9;
      --card-bg: #ffffff;
      --border: #cbd5e1;
      --text: #0f172a;
      --text-muted: #64748b;
      --accent-mq2: #0284c7;
      --accent-mq135: #7c3aed;
      --canvas-bg: #e2e8f0;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; transition: background-color 0.25s, color 0.25s; }
    body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; background: var(--bg); color: var(--text); display: flex; justify-content: center; padding: 14px; }
    .container { width: 100%; max-width: 440px; }
    .header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 12px; }
    h1 { font-size: 1.25rem; font-weight: 700; color: var(--accent-mq2); }
    .theme-btn { background: var(--card-bg); border: 1px solid var(--border); color: var(--text); padding: 6px 12px; border-radius: 8px; font-size: 0.75rem; font-weight: 600; cursor: pointer; }
    .status-badge { display: block; text-align: center; padding: 8px 16px; border-radius: 9999px; font-weight: 700; font-size: 0.85rem; letter-spacing: 0.5px; margin-bottom: 14px; }
    .normal { background: rgba(16, 185, 129, 0.15); color: #10b981; border: 1px solid rgba(16, 185, 129, 0.3); }
    .warmup { background: rgba(234, 179, 8, 0.15); color: #eab308; border: 1px solid rgba(234, 179, 8, 0.3); }
    .alarm { background: rgba(239, 68, 68, 0.2); color: #ef4444; border: 1px solid rgba(239, 68, 68, 0.4); animation: pulse 0.8s infinite; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 14px; }
    .card { background: var(--card-bg); border: 1px solid var(--border); border-radius: 18px; padding: 14px; text-align: center; }
    .val-label { font-size: 0.72rem; color: var(--text-muted); text-transform: uppercase; font-weight: 700; }
    .val-box { font-size: 2.2rem; font-weight: 800; line-height: 1.2; margin: 4px 0; }
    .val-mq2 { color: var(--accent-mq2); }
    .val-mq135 { color: var(--accent-mq135); }
    .sub-meta { font-size: 0.72rem; color: var(--text-muted); }
    .chart-card { background: var(--card-bg); border: 1px solid var(--border); border-radius: 18px; padding: 12px; margin-bottom: 14px; }
    .chart-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 6px; font-size: 0.75rem; font-weight: 700; color: var(--text-muted); }
    .legend { display: flex; gap: 10px; font-size: 0.7rem; }
    .leg-mq2 { color: var(--accent-mq2); }
    .leg-mq135 { color: var(--accent-mq135); }
    canvas { width: 100%; height: 110px; background: var(--canvas-bg); border-radius: 10px; border: 1px solid var(--border); display: block; }
    .log-card { background: var(--card-bg); border: 1px solid var(--border); border-radius: 18px; padding: 12px; margin-bottom: 14px; }
    .log-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 6px; }
    .log-title { font-size: 0.75rem; font-weight: 700; color: var(--text-muted); text-transform: uppercase; }
    .export-btn { background: transparent; border: 1px solid var(--border); color: var(--accent-mq2); padding: 3px 8px; border-radius: 6px; font-size: 0.7rem; font-weight: 700; cursor: pointer; }
    #logContainer { height: 75px; overflow-y: auto; font-family: monospace; font-size: 0.72rem; color: var(--text); display: flex; flex-direction: column-reverse; gap: 3px; }
    .log-entry { border-bottom: 1px solid var(--border); padding-bottom: 2px; }
    .log-alert { color: #ef4444; font-weight: bold; }
    .log-safe { color: #10b981; }
    .btn-row { display: flex; gap: 10px; }
    .btn { background: var(--accent-mq2); color: #0b0f19; border: none; padding: 12px; border-radius: 14px; font-weight: 700; font-size: 0.85rem; width: 100%; cursor: pointer; }
    .btn-mute { background: #ef4444; color: #ffffff; display: none; }
    .btn-muted-state { background: #64748b; color: #f8fafc; }
    @keyframes pulse { 0%, 100% { transform: scale(1); } 50% { transform: scale(1.02); } }
  </style>
</head>
<body>
  <div class="container">
    <div class="header">
      <h1>EcoSense AIoT Hub</h1>
      <button class="theme-btn" onclick="toggleTheme()" id="themeBtn">Light Mode</button>
    </div>

    <div id="statusBadge" class="status-badge warmup">CALIBRATING HEATERS (30s)</div>

    <div class="grid">
      <div class="card">
        <div class="val-label">MQ-2 Gas / Smoke</div>
        <div class="val-box val-mq2" id="mq2Val">--</div>
        <div class="sub-meta">Base: <span id="mq2Base">--</span> | Thr: <span id="mq2Thrs">--</span></div>
      </div>
      <div class="card">
        <div class="val-label">MQ-135 Air Quality</div>
        <div class="val-box val-mq135" id="mq135Val">--</div>
        <div class="sub-meta">Base: <span id="mq135Base">--</span> | Thr: <span id="mq135Thrs">--</span></div>
      </div>
    </div>

    <div class="chart-card">
      <div class="chart-header">
        <span>LIVE TELEMETRY PLOT</span>
        <div class="legend">
          <span class="leg-mq2">&#9632; MQ-2</span>
          <span class="leg-mq135">&#9632; MQ-135</span>
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
        <div class="log-entry">Stabilizing sensors on startup...</div>
      </div>
    </div>

    <div class="btn-row">
      <button id="audioBtn" class="btn" onclick="initAudio()">Tap to Enable Siren</button>
      <button id="muteBtn" class="btn btn-mute" onclick="toggleMute()">Mute Siren</button>
    </div>
  </div>

  <script>
    let audioCtx = null, osc = null, gainNode = null;
    let isMuted = false;
    let lastStateWasAlarm = false;
    let isLightMode = false;
    const historyMQ2 = new Array(50).fill(0);
    const historyMQ135 = new Array(50).fill(0);
    const csvRecords = [];

    const canvas = document.getElementById('liveChart');
    const ctx = canvas.getContext('2d');

    function toggleTheme() {
      isLightMode = !isLightMode;
      document.body.classList.toggle('light-theme', isLightMode);
      document.getElementById('themeBtn').innerText = isLightMode ? 'Dark Mode' : 'Light Mode';
      drawChart();
    }

    function initAudio() {
      if (!audioCtx) {
        audioCtx = new (window.AudioContext || window.webkitAudioContext)();
        document.getElementById('audioBtn').style.display = 'none';
        document.getElementById('muteBtn').style.display = 'block';
        addLog('Audio alarm system armed', false);
      }
    }

    function toggleMute() {
      isMuted = !isMuted;
      const mBtn = document.getElementById('muteBtn');
      if (isMuted) {
        mBtn.innerText = 'Unmute Siren';
        mBtn.className = 'btn btn-mute btn-muted-state';
        if (osc) { osc.stop(); osc.disconnect(); osc = null; }
        addLog('Audio alarm muted manually', false);
      } else {
        mBtn.innerText = 'Mute Siren';
        mBtn.className = 'btn btn-mute';
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
      if (container.children.length > 20) container.removeChild(container.lastChild);
    }

    function exportCSV() {
      if (csvRecords.length === 0) { alert('No data collected yet.'); return; }
      let csvContent = 'data:text/csv;charset=utf-8,Timestamp,MQ2_Raw,MQ2_Base,MQ2_Thrs,MQ135_Raw,MQ135_Base,MQ135_Thrs,Alarm_State\n';
      csvRecords.forEach(r => {
        csvContent += `${r.t},${r.mq2},${r.mq2_b},${r.mq2_t},${r.mq135},${r.mq135_b},${r.mq135_t},${r.alm}\n`;
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
        if (osc && isMuted) { osc.stop(); osc.disconnect(); osc = null; }
        return;
      }
      if (play) {
        if (!osc) {
          osc = audioCtx.createOscillator();
          gainNode = audioCtx.createGain();
          osc.type = 'sawtooth';
          osc.connect(gainNode);
          gainNode.connect(audioCtx.destination);
          gainNode.gain.setValueAtTime(0.2, audioCtx.currentTime);
          osc.start();
        }
        let targetFreq = 440 + (freqRatio * 1000);
        osc.frequency.setTargetAtTime(targetFreq, audioCtx.currentTime, 0.05);
      } else if (osc) {
        osc.stop();
        osc.disconnect();
        osc = null;
      }
    }

    function drawChart() {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.strokeStyle = isLightMode ? '#cbd5e1' : '#1e293b';
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

      plotLine(historyMQ2, isLightMode ? '#0284c7' : '#38bdf8');
      plotLine(historyMQ135, isLightMode ? '#7c3aed' : '#a78bfa');
    }

    setInterval(() => {
      fetch('/data').then(r => r.json()).then(d => {
        document.getElementById('mq2Val').innerText = d.mq2;
        document.getElementById('mq2Base').innerText = d.mq2_b;
        document.getElementById('mq2Thrs').innerText = d.mq2_t;

        document.getElementById('mq135Val').innerText = d.mq135;
        document.getElementById('mq135Base').innerText = d.mq135_b;
        document.getElementById('mq135Thrs').innerText = d.mq135_t;

        historyMQ2.push(d.mq2); historyMQ2.shift();
        historyMQ135.push(d.mq135); historyMQ135.shift();
        drawChart();

        if (!d.warmup) {
          csvRecords.push({
            t: new Date().toLocaleTimeString(),
            mq2: d.mq2, mq2_b: d.mq2_b, mq2_t: d.mq2_t,
            mq135: d.mq135, mq135_b: d.mq135_b, mq135_t: d.mq135_t,
            alm: (d.al_mq2 || d.al_mq135) ? 1 : 0
          });
          if (csvRecords.length > 500) csvRecords.shift();
        }

        const badge = document.getElementById('statusBadge');
        if (d.warmup) {
          badge.className = 'status-badge warmup';
          badge.innerText = 'CALIBRATING HEATERS (' + d.left + 's)';
          setSiren(false, 0);
        } else if (d.al_mq2 || d.al_mq135) {
          badge.className = 'status-badge alarm';
          badge.innerText = d.al_mq2 ? 'HAZARD: GAS SPIKE DETECTED' : 'WARNING: CONTAMINATED AIR';
          let ratio = d.al_mq2 ? (d.mq2 - d.mq2_t) / (4095 - d.mq2_t) : (d.mq135 - d.mq135_t) / (4095 - d.mq135_t);
          setSiren(true, Math.min(Math.max(ratio, 0), 1));
          
          if (!lastStateWasAlarm) {
            addLog(d.al_mq2 ? 'MQ-2 Gas alarm triggered' : 'MQ-135 Air contamination alarm triggered', true);
            lastStateWasAlarm = true;
          }
        } else {
          badge.className = 'status-badge normal';
          badge.innerText = 'ALL SENSORS NORMAL';
          setSiren(false, 0);

          if (lastStateWasAlarm) {
            addLog('Atmosphere normalized back below thresholds', false);
            lastStateWasAlarm = false;
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
  j += "\"al_mq2\":" + String(isAlarmMQ2 ? "true" : "false") + ",";
  j += "\"al_mq135\":" + String(isAlarmMQ135 ? "true" : "false");
  j += "}";
  server.send(200, "application/json", j);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(MUTE_BTN_PIN, INPUT_PULLUP);

  digitalWrite(LED_PIN, HIGH);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(RELAY_PIN, LOW);

  analogSetPinAttenuation(MQ2_PIN, ADC_11db);
  analogSetPinAttenuation(MQ135_PIN, ADC_11db);

  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP);
  delay(100);
  WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 4);

  Serial.println("\n==========================================");
  Serial.println("      EcoSense AIoT Standalone Hub        ");
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

  // Pushbutton Debounce & State Toggle
  int rawReading = digitalRead(MUTE_BTN_PIN);
  if (rawReading != lastBtnReading) {
    lastDebounceTime = currentMillis;
  }
  if ((currentMillis - lastDebounceTime) > DEBOUNCE_DELAY_MS) {
    if (rawReading != stableBtnState) {
      stableBtnState = rawReading;
      if (stableBtnState == LOW) {
        isHardwareMuted = !isHardwareMuted;
        Serial.print("Hardware Buzzer: ");
        Serial.println(isHardwareMuted ? "MUTED" : "UNMUTED");
      }
    }
  }
  lastBtnReading = rawReading;

  // Non-blocking 30-second thermal warmup
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
      Serial.println("\n>>> HEATERS STABILIZED: BASELINES ARMED <<<");
    }
  }

  // 10 Hz Sensor & Logic Processing Loop
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

    // Hardware Alert & Exhaust Automation
    if (isAlarmMQ2 || isAlarmMQ135) {
      digitalWrite(BUZZER_PIN, isHardwareMuted ? LOW : HIGH);
      digitalWrite(RELAY_PIN, HIGH);
    } else {
      digitalWrite(BUZZER_PIN, LOW);
      digitalWrite(RELAY_PIN, LOW);
    }

    // Serial Plotter Output
    Serial.print("MQ2_Gas:"); Serial.print(liveMQ2);
    Serial.print("\tMQ2_Thrs:"); Serial.print(thrsMQ2);
    Serial.print("\tMQ135_Air:"); Serial.print(liveMQ135);
    Serial.print("\tMQ135_Thrs:"); Serial.print(thrsMQ135);
    Serial.print("\tMuted:"); Serial.print(isHardwareMuted ? 1 : 0);
    Serial.print("\tWarmup_Left:"); Serial.print(isWarmingUp ? warmupSecondsLeft : 0);
    Serial.print("\tAlarm:"); Serial.println((isAlarmMQ2 || isAlarmMQ135) ? 1 : 0);
  }

  // Built-in Blue LED Heartbeat / Alarm Strobe
  if (isWarmingUp || isAlarmMQ2 || isAlarmMQ135) {
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