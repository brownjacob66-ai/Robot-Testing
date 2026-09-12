#include <HX711.h>

// ==========================
// Pin configuration
// ==========================
const int LOADCELL_DOUT_PIN = 3;
const int LOADCELL_SCK_PIN  = 2;
const int STOP_OUT_PIN      = 8;

// ==========================
// HX711
// ==========================
HX711 scale;
float calibration_factor = -25.19f;  // keep your validated value

// ==========================
// Filtering / reporting
// ==========================
const uint8_t NUM_READINGS = 16;
float ringBuf[NUM_READINGS];
uint8_t ringIdx = 0;
float ringSum = 0.0f;

unsigned long lastReportMs = 0;
const unsigned long REPORT_INTERVAL_MS = 20; // ~50 Hz report to Python

// ==========================
// Stop trigger behavior
// ==========================
const float PROBE_CONTACT_THRESHOLD = 40.0f;

// pulse shaping / lockout
enum StopState : uint8_t { SS_IDLE, SS_PULSE_HIGH, SS_LOCKOUT };
StopState stopState = SS_IDLE;

unsigned long stateMs = 0;
const unsigned long STOP_HIGH_MS = 80;      // hold HIGH long enough for Mega sampling
const unsigned long LOCKOUT_MS   = 700;     // short lockout to avoid chatter retriggers
const float RELEASE_HYST_FACTOR  = 0.55f;   // allow fast recovery if force drops

float smoothLoad(float x) {
  ringSum -= ringBuf[ringIdx];
  ringBuf[ringIdx] = x;
  ringSum += ringBuf[ringIdx];
  ringIdx = (ringIdx + 1) % NUM_READINGS;
  return ringSum / NUM_READINGS;
}

void setup() {
  Serial.begin(115200);

  pinMode(STOP_OUT_PIN, OUTPUT);
  digitalWrite(STOP_OUT_PIN, LOW);

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(calibration_factor);
  scale.tare();

  for (uint8_t i = 0; i < NUM_READINGS; i++) ringBuf[i] = 0.0f;

  Serial.println("FORCE_UNO_READY");
}

void loop() {
  // Non-blocking read path
  if (scale.is_ready()) {
    float raw = scale.get_units(1);     // single conversion fetch
    float filtered = smoothLoad(raw);

    // stop-line state machine
    switch (stopState) {
      case SS_IDLE:
        if (filtered >= PROBE_CONTACT_THRESHOLD) {
          digitalWrite(STOP_OUT_PIN, HIGH);
          stopState = SS_PULSE_HIGH;
          stateMs = millis();
        }
        break;

      case SS_PULSE_HIGH:
        if (millis() - stateMs >= STOP_HIGH_MS) {
          digitalWrite(STOP_OUT_PIN, LOW);
          stopState = SS_LOCKOUT;
          stateMs = millis();
        }
        break;

      case SS_LOCKOUT:
        // recover by timeout OR by significant unload
        if ((millis() - stateMs >= LOCKOUT_MS) ||
            (filtered < (PROBE_CONTACT_THRESHOLD * RELEASE_HYST_FACTOR))) {
          stopState = SS_IDLE;
        }
        break;
    }

    // periodic force telemetry
    if (millis() - lastReportMs >= REPORT_INTERVAL_MS) {
      lastReportMs = millis();
      Serial.print("FORCE:");
      Serial.println(filtered, 1);
    }
  }
}