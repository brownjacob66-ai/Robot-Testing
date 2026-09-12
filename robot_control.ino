#include <AccelStepper.h>

// ==========================
// Pin configuration
// ==========================
// Limit switches
const int baseLimitSwitch       = 22;
const int shoulderLimitSwitch   = 24;
const int elbowLimitSwitch      = 26;
const int wristPitchLimitSwitch = 28;

// Step/Dir pins
const int baseDirPin = 50,       baseStepPin = 48;
const int shoulderDirPin = 46,   shoulderStepPin = 44;
const int elbowDirPin = 42,      elbowStepPin = 40;
const int wristPitchDirPin = 38, wristPitchStepPin = 36;
const int wristRollDirPin = 34,  wristRollStepPin = 32;

// Hardware STOP from load-cell Uno
const int STOP_PIN = 2;

// ==========================
// Steppers
// ==========================
AccelStepper baseStepper(AccelStepper::DRIVER, baseStepPin, baseDirPin);
AccelStepper shoulderStepper(AccelStepper::DRIVER, shoulderStepPin, shoulderDirPin);
AccelStepper elbowStepper(AccelStepper::DRIVER, elbowStepPin, elbowDirPin);
AccelStepper wristPitchStepper(AccelStepper::DRIVER, wristPitchStepPin, wristPitchDirPin);
AccelStepper wristRollStepper(AccelStepper::DRIVER, wristRollStepPin, wristRollDirPin);

// ==========================
// Homing offsets (from your setup)
// ==========================
const long baseHomeSteps       = 15300;
const long shoulderHomeSteps   = 1075;
const long elbowHomeSteps      = 1203;
const long wristPitchHomeSteps = 35800;
const long wristRollHomeSteps  = 0;

// ==========================
// Speeds/accels
// ==========================
const float baseSpeed = 5000,        baseAccel = 4000;
const float shoulderSpeed = 4500,    shoulderAccel = 3500;
const float elbowSpeed = 10000,      elbowAccel = 8000;
const float wristPitchSpeed = 15000, wristPitchAccel = 10000;
const float wristRollSpeed = 12000,  wristRollAccel = 6000;

// Probe speed scaling
const float PROBE_SPEED_FACTOR = 0.18f;
const float PROBE_ACCEL_FACTOR = 0.55f;

// ==========================
// State machine
// ==========================
enum MotionMode : uint8_t {
  MODE_IDLE,
  MODE_MOVE,
  MODE_PROBE,
  MODE_HOME
};

MotionMode mode = MODE_IDLE;

// Latched emergency stop state
bool stopLatched = false;

// Current command targets
long tgtB = 0, tgtS = 0, tgtE = 0, tgtP = 0, tgtR = 0;

// Probe timeout
unsigned long probeStartMs = 0;
const unsigned long PROBE_TIMEOUT_MS = 25000;

// Periodic position report
unsigned long lastPosReportMs = 0;
const unsigned long POS_REPORT_INTERVAL_MS = 60;

// Serial line buffer
String lineBuf;

// ==========================
// Helpers
// ==========================
inline bool anyAxisMoving() {
  return (baseStepper.distanceToGo() != 0) ||
         (shoulderStepper.distanceToGo() != 0) ||
         (elbowStepper.distanceToGo() != 0) ||
         (wristPitchStepper.distanceToGo() != 0) ||
         (wristRollStepper.distanceToGo() != 0);
}

void setNominalKinematics() {
  baseStepper.setMaxSpeed(baseSpeed);             baseStepper.setAcceleration(baseAccel);
  shoulderStepper.setMaxSpeed(shoulderSpeed);     shoulderStepper.setAcceleration(shoulderAccel);
  elbowStepper.setMaxSpeed(elbowSpeed);           elbowStepper.setAcceleration(elbowAccel);
  wristPitchStepper.setMaxSpeed(wristPitchSpeed); wristPitchStepper.setAcceleration(wristPitchAccel);
  wristRollStepper.setMaxSpeed(wristRollSpeed);   wristRollStepper.setAcceleration(wristRollAccel);
}

void setProbeKinematics() {
  baseStepper.setMaxSpeed(baseSpeed * PROBE_SPEED_FACTOR);
  shoulderStepper.setMaxSpeed(shoulderSpeed * PROBE_SPEED_FACTOR);
  elbowStepper.setMaxSpeed(elbowSpeed * PROBE_SPEED_FACTOR);
  wristPitchStepper.setMaxSpeed(wristPitchSpeed * PROBE_SPEED_FACTOR);
  wristRollStepper.setMaxSpeed(wristRollSpeed * PROBE_SPEED_FACTOR);

  baseStepper.setAcceleration(baseAccel * PROBE_ACCEL_FACTOR);
  shoulderStepper.setAcceleration(shoulderAccel * PROBE_ACCEL_FACTOR);
  elbowStepper.setAcceleration(elbowAccel * PROBE_ACCEL_FACTOR);
  wristPitchStepper.setAcceleration(wristPitchAccel * PROBE_ACCEL_FACTOR);
  wristRollStepper.setAcceleration(wristRollAccel * PROBE_ACCEL_FACTOR);
}

void applySynchronousSpeedScaling(long b, long s, long e, long p, long r) {
  long dB = labs(b - baseStepper.currentPosition());
  long dS = labs(s - shoulderStepper.currentPosition());
  long dE = labs(e - elbowStepper.currentPosition());
  long dP = labs(p - wristPitchStepper.currentPosition());
  long dR = labs(r - wristRollStepper.currentPosition());
  long maxD = max(max(max(dB, dS), max(dE, dP)), dR);

  // keep a minimum factor so tiny moves still run smoothly
  const float minF = 0.20f;

  auto scale = [&](long d) -> float {
    if (maxD <= 0) return 1.0f;
    float f = (float)d / (float)maxD;
    if (f < minF) f = minF;
    return f;
  };

  float fB = scale(dB), fS = scale(dS), fE = scale(dE), fP = scale(dP), fR = scale(dR);

  baseStepper.setMaxSpeed(baseSpeed * fB);             baseStepper.setAcceleration(baseAccel * fB);
  shoulderStepper.setMaxSpeed(shoulderSpeed * fS);     shoulderStepper.setAcceleration(shoulderAccel * fS);
  elbowStepper.setMaxSpeed(elbowSpeed * fE);           elbowStepper.setAcceleration(elbowAccel * fE);
  wristPitchStepper.setMaxSpeed(wristPitchSpeed * fP); wristPitchStepper.setAcceleration(wristPitchAccel * fP);
  wristRollStepper.setMaxSpeed(wristRollSpeed * fR);   wristRollStepper.setAcceleration(wristRollAccel * fR);
}

void moveToAll(long b, long s, long e, long p, long r) {
  tgtB = b; tgtS = s; tgtE = e; tgtP = p; tgtR = r;
  baseStepper.moveTo(tgtB);
  shoulderStepper.moveTo(tgtS);
  elbowStepper.moveTo(tgtE);
  wristPitchStepper.moveTo(tgtP);
  wristRollStepper.moveTo(tgtR);
}

void emergencyStopNow(const char* reasonMsg, bool printStopTriggered) {
  baseStepper.stop();
  shoulderStepper.stop();
  elbowStepper.stop();
  wristPitchStepper.stop();
  wristRollStepper.stop();

  // one immediate run step to service stop commands
  baseStepper.run();
  shoulderStepper.run();
  elbowStepper.run();
  wristPitchStepper.run();
  wristRollStepper.run();

  stopLatched = true;
  mode = MODE_IDLE;

  if (printStopTriggered) {
    Serial.println("STOP_TRIGGERED");
  }
  if (reasonMsg) {
    Serial.println(reasonMsg);
  }
}

void reportPositions() {
  Serial.print("POS:");
  Serial.print(baseStepper.currentPosition()); Serial.print(",");
  Serial.print(shoulderStepper.currentPosition()); Serial.print(",");
  Serial.print(elbowStepper.currentPosition()); Serial.print(",");
  Serial.print(wristPitchStepper.currentPosition()); Serial.print(",");
  Serial.println(wristRollStepper.currentPosition());
}

bool limitActive(int pin) {
  return digitalRead(pin) == HIGH; // per your wiring behavior
}

// Non-blocking homing sub-state machine
enum HomeSubState : uint8_t {
  H_IDLE, H_SHOULDER_SEEK, H_SHOULDER_OFFSET,
  H_BASE_SEEK, H_BASE_OFFSET,
  H_ELBOW_SEEK, H_ELBOW_OFFSET,
  H_PITCH_SEEK, H_PITCH_OFFSET,
  H_ROLL_ZERO, H_DONE
};

HomeSubState homeState = H_IDLE;
unsigned long homeDebounceStart = 0;
bool homeDebounceArmed = false;

void beginHome() {
  setNominalKinematics();
  mode = MODE_HOME;
  homeState = H_SHOULDER_SEEK;
  homeDebounceArmed = false;
  shoulderStepper.moveTo(shoulderStepper.currentPosition() - 120000L);
}

bool debouncedHit(int pin, unsigned long debounceMs = 20) {
  if (limitActive(pin)) {
    if (!homeDebounceArmed) {
      homeDebounceArmed = true;
      homeDebounceStart = millis();
    } else if (millis() - homeDebounceStart >= debounceMs) {
      return true;
    }
  } else {
    homeDebounceArmed = false;
  }
  return false;
}

void serviceHome() {
  // Always run motors to keep motion smooth
  baseStepper.run();
  shoulderStepper.run();
  elbowStepper.run();
  wristPitchStepper.run();
  wristRollStepper.run();

  switch (homeState) {
    case H_SHOULDER_SEEK:
      if (debouncedHit(shoulderLimitSwitch)) {
        shoulderStepper.stop();
        shoulderStepper.setCurrentPosition(0);
        shoulderStepper.moveTo(shoulderHomeSteps);
        homeState = H_SHOULDER_OFFSET;
        homeDebounceArmed = false;
      }
      break;

    case H_SHOULDER_OFFSET:
      if (shoulderStepper.distanceToGo() == 0) {
        baseStepper.moveTo(baseStepper.currentPosition() - 120000L);
        homeState = H_BASE_SEEK;
      }
      break;

    case H_BASE_SEEK:
      if (debouncedHit(baseLimitSwitch)) {
        baseStepper.stop();
        baseStepper.setCurrentPosition(0);
        baseStepper.moveTo(baseHomeSteps);
        homeState = H_BASE_OFFSET;
        homeDebounceArmed = false;
      }
      break;

    case H_BASE_OFFSET:
      if (baseStepper.distanceToGo() == 0) {
        elbowStepper.moveTo(elbowStepper.currentPosition() - 120000L);
        homeState = H_ELBOW_SEEK;
      }
      break;

    case H_ELBOW_SEEK:
      if (debouncedHit(elbowLimitSwitch)) {
        elbowStepper.stop();
        elbowStepper.setCurrentPosition(0);
        elbowStepper.moveTo(elbowHomeSteps);
        homeState = H_ELBOW_OFFSET;
        homeDebounceArmed = false;
      }
      break;

    case H_ELBOW_OFFSET:
      if (elbowStepper.distanceToGo() == 0) {
        wristPitchStepper.moveTo(wristPitchStepper.currentPosition() - 120000L);
        homeState = H_PITCH_SEEK;
      }
      break;

    case H_PITCH_SEEK:
      if (debouncedHit(wristPitchLimitSwitch)) {
        wristPitchStepper.stop();
        wristPitchStepper.setCurrentPosition(0);
        wristPitchStepper.moveTo(wristPitchHomeSteps);
        homeState = H_PITCH_OFFSET;
        homeDebounceArmed = false;
      }
      break;

    case H_PITCH_OFFSET:
      if (wristPitchStepper.distanceToGo() == 0) {
        homeState = H_ROLL_ZERO;
      }
      break;

    case H_ROLL_ZERO:
      wristRollStepper.setCurrentPosition(0);
      wristRollStepper.moveTo(wristRollHomeSteps);
      homeState = H_DONE;
      break;

    case H_DONE:
      if (!anyAxisMoving()) {
        // Re-zero after home offsets as in your workflow
        baseStepper.setCurrentPosition(0);
        shoulderStepper.setCurrentPosition(0);
        elbowStepper.setCurrentPosition(0);
        wristPitchStepper.setCurrentPosition(0);
        wristRollStepper.setCurrentPosition(0);

        mode = MODE_IDLE;
        homeState = H_IDLE;
        Serial.println("HOME_COMPLETE");
      }
      break;

    default:
      break;
  }
}

void beginMove(long b, long s, long e, long p, long r) {
  stopLatched = false;
  mode = MODE_MOVE;
  setNominalKinematics();
  applySynchronousSpeedScaling(b, s, e, p, r);
  moveToAll(b, s, e, p, r);
}

void serviceMove() {
  baseStepper.run();
  shoulderStepper.run();
  elbowStepper.run();
  wristPitchStepper.run();
  wristRollStepper.run();

  if (!anyAxisMoving()) {
    mode = MODE_IDLE;
    Serial.println("MOVE_COMPLETE");
  }
}

void beginProbe(long b, long s, long e, long p, long r) {
  stopLatched = false;
  mode = MODE_PROBE;
  setProbeKinematics();
  moveToAll(b, s, e, p, r);
  probeStartMs = millis();
  Serial.println("STATUS: Autonomous Plunge Probe Initialized...");
}

void serviceProbe() {
  // Hardware stop should immediately terminate probe
  if (digitalRead(STOP_PIN) == HIGH) {
    long hitB = baseStepper.currentPosition();
    long hitS = shoulderStepper.currentPosition();
    long hitE = elbowStepper.currentPosition();
    long hitP = wristPitchStepper.currentPosition();
    long hitR = wristRollStepper.currentPosition();

    emergencyStopNow(nullptr, false);
    Serial.print("PROBE_HIT:");
    Serial.print(hitB); Serial.print(",");
    Serial.print(hitS); Serial.print(",");
    Serial.print(hitE); Serial.print(",");
    Serial.print(hitP); Serial.print(",");
    Serial.println(hitR);
    setNominalKinematics();
    return;
  }

  baseStepper.run();
  shoulderStepper.run();
  elbowStepper.run();
  wristPitchStepper.run();
  wristRollStepper.run();

  if (!anyAxisMoving()) {
    mode = MODE_IDLE;
    setNominalKinematics();
    Serial.println("STATUS: PROBE FAILED (Reached target without touching bed)");
    return;
  }

  if (millis() - probeStartMs > PROBE_TIMEOUT_MS) {
    emergencyStopNow("STATUS: PROBE FAILED (Timeout)", false);
    setNominalKinematics();
  }
}

void serviceSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      lineBuf.trim();
      if (lineBuf.length() == 0) {
        lineBuf = "";
        return;
      }

      // Commands
      if (lineBuf.startsWith("MOVE")) {
        long b, s, e, p, r;
        if (sscanf(lineBuf.c_str(), "MOVE %ld %ld %ld %ld %ld", &b, &s, &e, &p, &r) == 5) {
          beginMove(b, s, e, p, r);
        } else {
          Serial.println("ERROR: Bad MOVE format");
        }
      }
      else if (lineBuf.startsWith("PROBE")) {
        long b, s, e, p, r;
        if (sscanf(lineBuf.c_str(), "PROBE %ld %ld %ld %ld %ld", &b, &s, &e, &p, &r) == 5) {
          beginProbe(b, s, e, p, r);
        } else {
          Serial.println("ERROR: Bad PROBE format");
        }
      }
      else if (lineBuf == "HOME") {
        beginHome();
      }
      else if (lineBuf == "ABORT") {
        emergencyStopNow("ABORT_COMPLETE", false);
        reportPositions();
      }
      else if (lineBuf == "CLEAR_STOP") {
        stopLatched = false;
        mode = MODE_IDLE;
        setNominalKinematics();
        Serial.println("STOP_CLEARED");
      }
      else if (lineBuf == "POS?") {
        reportPositions();
      }

      lineBuf = "";
    } else {
      if (lineBuf.length() < 140) lineBuf += c;
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(baseLimitSwitch, INPUT_PULLUP);
  pinMode(shoulderLimitSwitch, INPUT_PULLUP);
  pinMode(elbowLimitSwitch, INPUT_PULLUP);
  pinMode(wristPitchLimitSwitch, INPUT_PULLUP);
  pinMode(STOP_PIN, INPUT);

  setNominalKinematics();

  lineBuf.reserve(160);
  Serial.println("MOTION_MEGA_READY");
}

void loop() {
  // Highest-priority async hardware stop (except during homing you may decide to ignore/handle differently)
  if (digitalRead(STOP_PIN) == HIGH && !stopLatched && mode != MODE_PROBE) {
    emergencyStopNow(nullptr, true);
    reportPositions();
  }

  // Mode service
  if (!stopLatched) {
    switch (mode) {
      case MODE_MOVE:  serviceMove();  break;
      case MODE_PROBE: serviceProbe(); break;
      case MODE_HOME:  serviceHome();  break;
      case MODE_IDLE:
      default:
        // keep run() active even idle for smooth decel completion
        baseStepper.run();
        shoulderStepper.run();
        elbowStepper.run();
        wristPitchStepper.run();
        wristRollStepper.run();
        break;
    }
  }

  // Optional periodic POS streaming while active
  if (mode != MODE_IDLE && millis() - lastPosReportMs >= POS_REPORT_INTERVAL_MS) {
    lastPosReportMs = millis();
    reportPositions();
  }

  serviceSerial();
}