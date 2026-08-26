#define LED_PIN 2       // Built-in blue LED
#define MQ2_PIN 34      // MQ-2 Analog Input Pin

// --- Configuration ---
const int SENSITIVITY_MARGIN = 400; // Rise above baseline required to trigger alert
const int HYSTERESIS = 50;          // Buffer zone to prevent LED flickering
const unsigned long SAMPLE_INTERVAL_MS = 50; // Read sensor every 50ms (20Hz)

int baselineValue = 0;
int alertThreshold = 0;
int smoothGasValue = 0;
bool alertState = false;

unsigned long lastSampleTime = 0;
unsigned long lastBlinkTime = 0;
bool ledStatus = false;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  delay(500);

  // --- Warmup & Auto-Calibration ---
  long sum = 0;
  const int totalSamples = 100;

  for (int i = 0; i < totalSamples; i++) {
    sum += analogRead(MQ2_PIN);
    digitalWrite(LED_PIN, (i % 2 == 0) ? HIGH : LOW);
    delay(30);
  }

  baselineValue = sum / totalSamples;
  alertThreshold = baselineValue + SENSITIVITY_MARGIN;
  smoothGasValue = baselineValue;
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  unsigned long currentMillis = millis();

  // Non-blocking sensor reading & plotting
  if (currentMillis - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = currentMillis;

    // Exponential Moving Average filter (reduces noise spikes)
    int rawRead = analogRead(MQ2_PIN);
    smoothGasValue = (int)(0.2 * rawRead + 0.8 * smoothGasValue);

    // Hysteresis threshold logic
    if (!alertState && smoothGasValue > alertThreshold) {
      alertState = true;
    } else if (alertState && smoothGasValue < (alertThreshold - HYSTERESIS)) {
      alertState = false;
    }

    // Serial Plotter Output (Key:Value format)
    Serial.print("Gas_Level:");
    Serial.print(smoothGasValue);
    Serial.print(" Baseline:");
    Serial.print(baselineValue);
    Serial.print(" Alert_Threshold:");
    Serial.println(alertThreshold);
  }

  // Visual LED handling
  if (alertState) {
    digitalWrite(LED_PIN, HIGH); // Solid ON when gas is detected
  } else {
    // Normal heartbeat blink every 500ms
    if (currentMillis - lastBlinkTime >= 500) {
      lastBlinkTime = currentMillis;
      ledStatus = !ledStatus;
      digitalWrite(LED_PIN, ledStatus);
    }
  }
}