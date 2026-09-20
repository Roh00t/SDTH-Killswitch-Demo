/*
 * Killswitch — ESP32-S3 Actuator Firmware
 *
 * Drives a 2-axis SG90 gimbal and a proxy effector (LED) from newline-
 * terminated ASCII commands on the UART bridge.
 *
 * THIS FIRMWARE IS THE SAFETY AUTHORITY. The host proposes; this decides.
 * It independently enforces the deadman timeout, the burn ceiling, arming,
 * and actuator bounds. It never trusts the host to do any of those.
 *
 * The effector on the bench is an eye-safe LED. Every guardrail here is sized
 * for a hazardous effector on purpose: the architecture claims this brain
 * re-hosts onto a real one, and that claim is only credible if the safety
 * logic was built for it from the start.
 *
 * Board:   ESP32-S3-WROOM-1
 * Library: ESP32Servo (Kevin Harrington)
 * Port:    UART bridge (CP2102/CH340) — NOT native USB-CDC. The bridge stays
 *          enumerated across ESP32 resets; native USB re-enumerates and the
 *          host's serial handle dies.
 *
 * Wiring:
 *   GPIO 5  -> Pan servo signal     (5V + GND from a SEPARATE supply)
 *   GPIO 6  -> Tilt servo signal    (common ground with the ESP32)
 *   GPIO 7  -> Effector gate        (220R -> LED -> GND, 10k pulldown to GND)
 *
 * The 10k pulldown on GPIO 7 is mandatory. Between power-on and the first line
 * of setup(), every ESP32 GPIO is a floating input. A floating gate is an
 * undefined effector state during boot, reflash, brownout and crash. Software
 * cannot fix this; the resistor can.
 */

#include <ESP32Servo.h>

// ---------------------------------------------------------------- pins ----
static const int PIN_SERVO_PAN  = 5;
static const int PIN_SERVO_TILT = 6;
static const int PIN_EFFECTOR   = 7;

// ------------------------------------------------------------ constants ----
static const unsigned long BAUD               = 921600UL;

// Actuator bounds. Also enforced host-side; twice is deliberate.
static const float PAN_MIN   = 0.0f;
static const float PAN_MAX   = 180.0f;
static const float TILT_MIN  = 45.0f;
static const float TILT_MAX  = 135.0f;
static const float STOW_PAN  = 90.0f;
static const float STOW_TILT = 90.0f;

// SG90 pulse widths. Calibrate per servo if travel is short or it buzzes at rest.
static const int SERVO_MIN_US = 500;
static const int SERVO_MAX_US = 2400;

// Effector cut if no valid command arrives inside this window.
static const unsigned long DEADMAN_TIMEOUT_MS = 250UL;
// Hard ceiling on one burn. Enforced here even if the host commands longer.
static const unsigned long MAX_BURN_MS        = 2000UL;

static const unsigned long STATUS_INTERVAL_MS = 100UL;  // 10 Hz telemetry
static const unsigned long SERVO_UPDATE_MS    = 20UL;   // 50 Hz slew update

// Max degrees per servo update -> ~250 deg/s. Limits inrush current and stops
// the large-step commands that stall an SG90 and brown out a shared rail.
static const float SLEW_RATE_DEG = 5.0f;
// Below the SG90 deadband, writing only produces buzz and current draw.
static const float DEADBAND_DEG  = 0.5f;

static const size_t RX_BUFFER_SIZE = 64;

// --------------------------------------------------------------- state ----
Servo servoPan;
Servo servoTilt;

static float targetPan   = STOW_PAN;
static float targetTilt  = STOW_TILT;
static float currentPan  = STOW_PAN;
static float currentTilt = STOW_TILT;

static bool armed        = false;
static bool effectorOn   = false;
static bool estopLatched = true;   // Boot latched. An explicit arm is required.
static bool deadmanFired = false;

static unsigned long lastCommandMs = 0;
static unsigned long burnStartMs   = 0;
static unsigned long lastStatusMs  = 0;
static unsigned long lastServoMs   = 0;

static char   rxBuffer[RX_BUFFER_SIZE];
static size_t rxLength = 0;

// ------------------------------------------------------------- effector ----

/* De-energise unconditionally. Safe to call from anywhere, any number of times.
 * Writes the pin before touching any flag: if a later line faults, the hardware
 * is already safe. */
static void killEffector() {
  digitalWrite(PIN_EFFECTOR, LOW);
  effectorOn  = false;
  burnStartMs = 0;
}

/* Kill the effector AND drop arming. Used for faults and e-stop. */
static void safeState() {
  killEffector();
  armed = false;
}

static void emitError(const char* code, const char* detail) {
  Serial.print("ERR ");
  Serial.print(code);
  Serial.print(' ');
  Serial.println(detail);
}

// ------------------------------------------------------------- checksum ----

/* XOR of the body bytes, matching helper/hardware/protocol.py::checksum. */
static uint8_t computeChecksum(const char* body, size_t length) {
  uint8_t acc = 0;
  for (size_t i = 0; i < length; i++) acc ^= (uint8_t)body[i];
  return acc;
}

/* Split "BODY*XX" and verify.
 *
 * Asymmetric by design: only hazard-INCREASING commands (M1 arm, L1 fire) are
 * checksummed. Hazard-DECREASING commands (M0, L0, Z) are always accepted, so a
 * corrupted byte can never block a shutdown. Combined with two-key arming, it
 * takes two independently valid checksummed commands to energise the effector.
 */
static bool verifyChecksum(char* line, size_t length, size_t* bodyLength) {
  char* star = nullptr;
  for (size_t i = 0; i < length; i++) {
    if (line[i] == '*') { star = &line[i]; break; }
  }
  if (star == nullptr) return false;

  *bodyLength = (size_t)(star - line);
  if (length - *bodyLength - 1 < 2) return false;

  char hex[3] = { star[1], star[2], '\0' };
  uint8_t supplied = (uint8_t)strtoul(hex, nullptr, 16);
  return supplied == computeChecksum(line, *bodyLength);
}

// -------------------------------------------------------------- parsing ----

static float clampf(float value, float lo, float hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

/* Parse "A<pan>,<tilt>". Returns false on any malformation. */
static bool parseAngles(const char* body, size_t length, float* pan, float* tilt) {
  char work[RX_BUFFER_SIZE];
  if (length >= RX_BUFFER_SIZE) return false;
  memcpy(work, body, length);
  work[length] = '\0';

  char* comma = strchr(work + 1, ',');
  if (comma == nullptr) return false;
  *comma = '\0';

  char* endPan  = nullptr;
  char* endTilt = nullptr;
  float p = strtod(work + 1, &endPan);
  float t = strtod(comma + 1, &endTilt);

  if (endPan == work + 1 || endTilt == comma + 1) return false;
  if (isnan(p) || isnan(t) || isinf(p) || isinf(t)) return false;

  *pan  = p;
  *tilt = t;
  return true;
}

static void handleCommand(char* line, size_t length) {
  if (length == 0) return;

  const char verb = line[0];

  // Z (e-stop) is processed ahead of everything, including its own validation.
  if (verb == 'Z') {
    safeState();
    estopLatched = true;
    lastCommandMs = millis();
    Serial.println("OK Z");
    return;
  }

  size_t bodyLength = length;
  bool checksummed  = verifyChecksum(line, length, &bodyLength);

  switch (verb) {
    case 'P':  // ping — refreshes the deadman and nothing else
      lastCommandMs = millis();
      deadmanFired = false;
      Serial.println("OK P");
      return;

    case 'S':
      lastCommandMs = millis();
      lastStatusMs = 0;  // force a status frame on the next loop
      return;

    case 'A': {
      float pan, tilt;
      if (!parseAngles(line, bodyLength, &pan, &tilt)) {
        emitError("E02", "angle parse failed");
        return;
      }
      // Clamp, never wrap: 190 becomes 180, never 10.
      targetPan  = clampf(pan,  PAN_MIN,  PAN_MAX);
      targetTilt = clampf(tilt, TILT_MIN, TILT_MAX);
      lastCommandMs = millis();
      deadmanFired = false;
      Serial.println("OK A");
      return;
    }

    case 'M': {
      if (bodyLength < 2) { emitError("E02", "M needs 0 or 1"); return; }
      const bool wantArmed = (line[1] == '1');

      if (!wantArmed) {  // disarm is always honoured
        safeState();
        lastCommandMs = millis();
        Serial.println("OK M0");
        return;
      }
      if (!checksummed) { emitError("E05", "M1 requires checksum"); return; }

      estopLatched = false;  // an explicit checksummed arm clears the latch
      armed = true;
      lastCommandMs = millis();
      deadmanFired = false;
      Serial.println("OK M1");
      return;
    }

    case 'L': {
      if (bodyLength < 2) { emitError("E02", "L needs 0 or 1"); return; }
      const bool wantOn = (line[1] == '1');

      if (!wantOn) {  // de-energise is always honoured
        killEffector();
        lastCommandMs = millis();
        Serial.println("OK L0");
        return;
      }
      if (!checksummed)  { emitError("E05", "L1 requires checksum");      return; }
      if (estopLatched)  { emitError("E08", "e-stop latched");            return; }
      if (!armed)        { emitError("E04", "not armed; send M1 first");  return; }

      // Two independently valid checksummed commands were required to reach
      // this line: M1 to arm, then L1 to fire.
      digitalWrite(PIN_EFFECTOR, HIGH);
      effectorOn  = true;
      burnStartMs = millis();
      lastCommandMs = burnStartMs;
      deadmanFired = false;
      Serial.println("OK L1");
      return;
    }

    default:
      emitError("E01", "unknown verb");
      return;
  }
}

// --------------------------------------------------------------- safety ----

/* Host silence for longer than the window means the host is gone: crashed,
 * unplugged, blocked on the GIL, or stalled in GC. Cut the effector without
 * the host participating. */
static void checkDeadman() {
  if (millis() - lastCommandMs <= DEADMAN_TIMEOUT_MS) return;
  if (effectorOn || armed) {
    safeState();
    if (!deadmanFired) {
      emitError("E07", "deadman timeout: effector cut, disarmed");
      deadmanFired = true;
    }
  }
}

/* Independent burn ceiling. The host is not trusted to time its own burn. */
static void checkBurnCeiling() {
  if (!effectorOn || burnStartMs == 0) return;
  if (millis() - burnStartMs >= MAX_BURN_MS) {
    killEffector();
    emitError("E06", "burn ceiling reached: effector cut by firmware");
  }
}

// --------------------------------------------------------------- motion ----

/* Step current angles toward target at a bounded rate.
 *
 * SG90s have no position feedback, so "current" is the commanded angle and
 * nothing more. Rate limiting keeps inrush current down and avoids the large
 * steps that stall the servo. Never present these values as measured telemetry.
 */
static void updateServos() {
  const unsigned long now = millis();
  if (now - lastServoMs < SERVO_UPDATE_MS) return;
  lastServoMs = now;

  const float dPan  = targetPan  - currentPan;
  const float dTilt = targetTilt - currentTilt;

  if (fabsf(dPan) > DEADBAND_DEG) {
    currentPan += clampf(dPan, -SLEW_RATE_DEG, SLEW_RATE_DEG);
    servoPan.write((int)roundf(clampf(currentPan, PAN_MIN, PAN_MAX)));
  }
  if (fabsf(dTilt) > DEADBAND_DEG) {
    currentTilt += clampf(dTilt, -SLEW_RATE_DEG, SLEW_RATE_DEG);
    servoTilt.write((int)roundf(clampf(currentTilt, TILT_MIN, TILT_MAX)));
  }
}

static void sendStatus() {
  const unsigned long now = millis();
  if (now - lastStatusMs < STATUS_INTERVAL_MS) return;
  lastStatusMs = now;

  Serial.print("ST ");
  Serial.print(currentPan, 1);   Serial.print(',');
  Serial.print(currentTilt, 1);  Serial.print(',');
  Serial.print(effectorOn ? '1' : '0'); Serial.print(',');
  Serial.print(armed ? '1' : '0');      Serial.print(',');
  Serial.println(now);
}

static void pumpSerial() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (rxLength > 0) {
        rxBuffer[rxLength] = '\0';
        handleCommand(rxBuffer, rxLength);
        rxLength = 0;
      }
      continue;
    }
    if (rxLength < RX_BUFFER_SIZE - 1) {
      rxBuffer[rxLength++] = c;
    } else {
      // Overlong line: discard and resynchronise on the next newline rather
      // than acting on a truncated command.
      rxLength = 0;
      emitError("E02", "rx overflow, line discarded");
    }
  }
}

// ---------------------------------------------------------------- entry ----

void setup() {
  // FIRST. Before serial, before servos, before anything that can block or
  // fault. The effector is de-energised as the very first action.
  pinMode(PIN_EFFECTOR, OUTPUT);
  digitalWrite(PIN_EFFECTOR, LOW);

  Serial.begin(BAUD);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servoPan.setPeriodHertz(50);
  servoTilt.setPeriodHertz(50);
  servoPan.attach(PIN_SERVO_PAN,   SERVO_MIN_US, SERVO_MAX_US);
  servoTilt.attach(PIN_SERVO_TILT, SERVO_MIN_US, SERVO_MAX_US);

  servoPan.write((int)STOW_PAN);
  servoTilt.write((int)STOW_TILT);

  safeState();
  estopLatched  = true;
  lastCommandMs = millis();

  Serial.println("OK BOOT killswitch-actuator v1");
}

void loop() {
  pumpSerial();
  checkDeadman();      // before motion: safety decisions precede actuation
  checkBurnCeiling();
  updateServos();
  sendStatus();
}
