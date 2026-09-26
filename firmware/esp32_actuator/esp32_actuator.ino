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
 * The effector on the bench is a KY-008 650 nm laser module — a low-power
 * stand-in, but a real emitter, not an indicator LED. Every guardrail here is
 * sized for a hazardous effector on purpose: the architecture claims this brain
 * re-hosts onto a real one, and that claim is only credible if the safety
 * logic was built for it from the start.
 *
 * NO FIRMWARE CHANGE was needed when the effector moved from an LED to the
 * laser module. The gate is a plain digitalWrite on the same pin either way;
 * only what hangs off it changed. Recorded here so the next reader does not go
 * looking for a driver that does not exist.
 *
 * Board:   ESP32-S3-N16R8 (16 MB flash, 8 MB PSRAM)
 * Library: ESP32Servo (Kevin Harrington)
 * Port:    UART bridge (CH343 on this board; CP2102/CH340 on others) — NOT
 *          native USB-CDC. The bridge stays enumerated across ESP32 resets;
 *          native USB re-enumerates and the host's serial handle dies.
 *
 * CAMERA: the OV5640 on this board's camera connector streams MJPEG over Wi-Fi
 * (KS_CAMERA below). The connector uses the ESP32-S3-EYE layout, which puts
 * SIOC on GPIO 5, VSYNC on 6 and HREF on 7. The actuator used to sit on those
 * three pins, and HREF on the laser gate would pulse the effector at line rate,
 * so the actuator moved and a compile-time check refuses any overlap. Camera,
 * Wi-Fi and the stream server run on core 0; the safety loop runs on core 1
 * and shares nothing with them. See CLAUDE.md.
 *
 * Wiring:
 *   GPIO 14 -> Pan servo signal     (5V + GND from a SEPARATE supply)
 *   GPIO 21 -> Tilt servo signal    (common ground with the ESP32)
 *   GPIO 1  -> Effector gate        (1k -> 2N2222 base; collector sinks the
 *                                    KY-008 '-' terminal; 10k pulldown to GND)
 *   KY-008 S -> the board's 5V pin  (USB 5 V; middle pin unconnected; emitter
 *                                    to GND). S on the collector and '-' on GND
 *                                    puts no supply across the diode: it never
 *                                    lights, yet serial_probe still passes.
 *
 * The 10k pulldown on GPIO 1 is mandatory. Between power-on and the first line
 * of setup(), every ESP32 GPIO is a floating input. A floating gate is an
 * undefined effector state during boot, reflash, brownout and crash. Software
 * cannot fix this; the resistor can.
 */

#include <ESP32Servo.h>

// ---------------------------------------------------------------- pins ----
// Off GPIO 4-18: the camera connector owns those (see the header).
static constexpr int PIN_SERVO_PAN  = 14;
static constexpr int PIN_SERVO_TILT = 21;
static constexpr int PIN_EFFECTOR   = 1;

// 1 = stream the OV5640 over Wi-Fi. 0 = actuator only, as before.
#ifndef KS_CAMERA
#define KS_CAMERA 1
#endif

#if KS_CAMERA
#include <atomic>
#include "esp_camera.h"
#include "esp_http_server.h"
#include <ESPmDNS.h>
#include <WiFi.h>
#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"   // WIFI_SSID, WIFI_PASSWORD. Gitignored.
#else
#error "Copy wifi_secrets.example.h to wifi_secrets.h in this folder and put your hotspot name and password in it."
#endif

// ESP32-S3-EYE camera connector (Freenove ESP32-S3-WROOM CAM and its clones).
// From Espressif's CameraWebServer camera_pins.h, CAMERA_MODEL_ESP32S3_EYE.
static constexpr int CAM_PIN_XCLK  = 15;
static constexpr int CAM_PIN_SIOD  = 4;
static constexpr int CAM_PIN_SIOC  = 5;
static constexpr int CAM_PIN_D0    = 11;
static constexpr int CAM_PIN_D1    = 9;
static constexpr int CAM_PIN_D2    = 8;
static constexpr int CAM_PIN_D3    = 10;
static constexpr int CAM_PIN_D4    = 12;
static constexpr int CAM_PIN_D5    = 18;
static constexpr int CAM_PIN_D6    = 17;
static constexpr int CAM_PIN_D7    = 16;
static constexpr int CAM_PIN_VSYNC = 6;
static constexpr int CAM_PIN_HREF  = 7;
static constexpr int CAM_PIN_PCLK  = 13;

static constexpr int CAM_PINS[] = {
  CAM_PIN_XCLK, CAM_PIN_SIOD, CAM_PIN_SIOC, CAM_PIN_D0, CAM_PIN_D1, CAM_PIN_D2,
  CAM_PIN_D3, CAM_PIN_D4, CAM_PIN_D5, CAM_PIN_D6, CAM_PIN_D7, CAM_PIN_VSYNC,
  CAM_PIN_HREF, CAM_PIN_PCLK,
};
static constexpr bool usesCameraPin(int pin, size_t i = 0) {
  return i < sizeof(CAM_PINS) / sizeof(CAM_PINS[0]) &&
         (CAM_PINS[i] == pin || usesCameraPin(pin, i + 1));
}
// A camera signal on an actuator pin drives it from the sensor, outside every
// interlock in this file. Refuse to build rather than find out on the bench.
static_assert(!usesCameraPin(PIN_EFFECTOR),   "effector gate shares a camera pin");
static_assert(!usesCameraPin(PIN_SERVO_PAN),  "pan servo shares a camera pin");
static_assert(!usesCameraPin(PIN_SERVO_TILT), "tilt servo shares a camera pin");

static const char* CAM_HOSTNAME = "killswitch-cam";   // http://killswitch-cam.local/stream
#endif

// ------------------------------------------------------------ constants ----
static const unsigned long BAUD               = 921600UL;

// Actuator bounds. Also enforced host-side; twice is deliberate.
static const float PAN_MIN   = 0.0f;
static const float PAN_MAX   = 180.0f;
static const float TILT_MIN  = 45.0f;
static const float TILT_MAX  = 135.0f;
static const float STOW_PAN  = 90.0f;
static const float STOW_TILT = 90.0f;

// The rig's pan SG90 turns ANTICLOCKWISE, seen from above, as its angle rises.
// Everything upstream means CLOCKWISE: cue geometry, the dashboard radar, the
// control law and serial_probe's `a`. writePan() mirrors the angle at the one
// place it becomes PWM, so every angle above that line (bounds, ST frames, the
// host) keeps that convention. PAN_MIN/PAN_MAX are symmetric about 90, so the
// mirrored angle is always inside them. false = a servo that already turns
// clockwise.
static constexpr bool PAN_REVERSED = true;

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
// ESP32Servo::attach() returns the LEDC channel, or 0 on failure. Unchecked,
// a failed attach produces firmware that answers every command correctly and
// never emits a PWM pulse — silent servos with no error anywhere.
static int  panChannel   = 0;
static int  tiltChannel  = 0;
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

/* The only place a pan angle reaches the servo. Clamps, then mirrors when
 * PAN_REVERSED. Loop thread (core 1) and setup() only. */
static void writePan(float degrees) {
  const float clamped = clampf(degrees, PAN_MIN, PAN_MAX);
  servoPan.write((int)roundf(PAN_REVERSED ? (PAN_MIN + PAN_MAX) - clamped : clamped));
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

    case 'D': {  // diagnostics — is PWM actually being generated?
      lastCommandMs = millis();
      Serial.print("DIAG pan_pin="); Serial.print(PIN_SERVO_PAN);
      Serial.print(" pan_ch=");      Serial.print(panChannel);
      Serial.print(" pan_attached="); Serial.print(servoPan.attached() ? 1 : 0);
      Serial.print(" tilt_pin=");    Serial.print(PIN_SERVO_TILT);
      Serial.print(" tilt_ch=");     Serial.print(tiltChannel);
      Serial.print(" tilt_attached=");Serial.print(servoTilt.attached() ? 1 : 0);
      Serial.print(" us_range=");    Serial.print(SERVO_MIN_US);
      Serial.print("-");             Serial.println(SERVO_MAX_US);
      return;
    }

    case 'T': {  // raw sweep, bypassing slew limiting and the deadband
      lastCommandMs = millis();
      Serial.println("OK T raw sweep starting");
      for (int angle = 20; angle <= 160; angle += 10) {
        writePan(angle);
        servoTilt.write(constrain(angle, (int)TILT_MIN, (int)TILT_MAX));
        delay(120);
      }
      writePan(STOW_PAN);
      servoTilt.write((int)STOW_TILT);
      currentPan = targetPan = STOW_PAN;
      currentTilt = targetTilt = STOW_TILT;
      lastCommandMs = millis();
      Serial.println("OK T raw sweep done");
      return;
    }

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
    writePan(currentPan);
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

// --------------------------------------------------------------- camera ----
#if KS_CAMERA
// Written by the camera task (core 0), read by loop() (core 1). Only loop()
// prints, so camera lines can never interleave with an ST frame on the UART.
static volatile int  cameraState = 0;    // 0 starting, 1 streaming, -1 init failed
static volatile int  cameraError = 0;
static volatile int  cameraPid   = 0;    // sensor product id: 0x5640 on an OV5640
static volatile int  cameraStalls = 0;   // sensor went silent; camera restarted
static httpd_handle_t streamServer = nullptr;

// No frame for this long while a viewer is connected: the sensor has stalled.
static constexpr uint32_t STREAM_STALL_MS = 3000;
static esp_err_t startCamera();

// Wi-Fi diagnostics, same rule: core 0 writes, loop() prints. Without them a
// hotspot that refuses the board looks exactly like a URL that scrolled past.
static constexpr int SCAN_PENDING = -100;
static std::atomic<int> wifiScanCount{SCAN_PENDING};  // published last
static volatile int  wifiSeenRssi    = 0;    // 0: our SSID was not in the scan
static volatile int  wifiSeenChannel = 0;
static char          wifiNearby[4][33] = {}; // a few SSIDs the scan did hear
static volatile int  wifiDropReason  = 0;    // last STA_DISCONNECTED reason

#define STREAM_BOUNDARY "killswitchframe"

/* One MJPEG client at a time: the handler owns the server task while it
 * streams. Runs on the HTTP server task, pinned to core 0.
 *
 * A viewer's departure is only noticed when a send fails, and nothing is sent
 * while the sensor is silent. Waiting for frames forever therefore held the
 * server's only task for good, and every later viewer hung: on the rig, 20
 * minutes of timeouts with the board still up. After STREAM_STALL_MS without
 * a frame the handler hangs up and restarts the camera for the next viewer. */
static esp_err_t streamHandler(httpd_req_t* req) {
  httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=" STREAM_BOUNDARY);
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  char part[96];
  uint32_t lastFrameMs = millis();
  while (true) {
    camera_fb_t* fb = esp_camera_fb_get();   // itself waits up to ~4 s
    if (fb == nullptr) {
      if (millis() - lastFrameMs > STREAM_STALL_MS) {
        esp_camera_deinit();
        const esp_err_t err = startCamera();
        cameraError = (int)err;
        cameraState = err == ESP_OK ? 1 : -1;
        cameraStalls = cameraStalls + 1;
        return ESP_FAIL;   // closes this viewer; the host reconnects
      }
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    lastFrameMs = millis();
    const int headerLength = snprintf(
        part, sizeof(part),
        "\r\n--" STREAM_BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
        (unsigned)fb->len);
    esp_err_t res = httpd_resp_send_chunk(req, part, headerLength);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) return res;   // client went away
  }
}

static esp_err_t startCamera() {
  camera_config_t c = {};
  // The servos own LEDC timers 0 and 1 (allocateTimer in setup). XCLK takes
  // timer 3 and the last channel so it can never retime a servo.
  c.ledc_timer   = LEDC_TIMER_3;
  c.ledc_channel = LEDC_CHANNEL_7;
  c.pin_d0 = CAM_PIN_D0;  c.pin_d1 = CAM_PIN_D1;  c.pin_d2 = CAM_PIN_D2;
  c.pin_d3 = CAM_PIN_D3;  c.pin_d4 = CAM_PIN_D4;  c.pin_d5 = CAM_PIN_D5;
  c.pin_d6 = CAM_PIN_D6;  c.pin_d7 = CAM_PIN_D7;
  c.pin_xclk = CAM_PIN_XCLK;  c.pin_pclk = CAM_PIN_PCLK;
  c.pin_vsync = CAM_PIN_VSYNC;  c.pin_href = CAM_PIN_HREF;
  c.pin_sccb_sda = CAM_PIN_SIOD;  c.pin_sccb_scl = CAM_PIN_SIOC;
  c.pin_pwdn = -1;  c.pin_reset = -1;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.grab_mode = CAMERA_GRAB_LATEST;   // never hand out a stale frame
  if (psramFound()) {
    c.frame_size = FRAMESIZE_VGA;  c.jpeg_quality = 12;
    c.fb_count = 2;                c.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    c.frame_size = FRAMESIZE_QVGA; c.jpeg_quality = 12;
    c.fb_count = 1;                c.fb_location = CAMERA_FB_IN_DRAM;
  }
  return esp_camera_init(&c);
}

static void startStreamServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.core_id = 0;               // keep the stream off the safety core
  config.lru_purge_enable = true;   // a dead client frees its slot
  httpd_uri_t stream = {};
  stream.uri = "/stream";
  stream.method = HTTP_GET;
  stream.handler = streamHandler;
  if (httpd_start(&streamServer, &config) == ESP_OK) {
    httpd_register_uri_handler(streamServer, &stream);
  }
}

/* Arduino's Wi-Fi event task. Records why the hotspot dropped or refused us;
 * loop() turns it into words. Never prints: only loop() owns the UART. */
static void onWifiDisconnected(WiFiEvent_t, WiFiEventInfo_t info) {
  wifiDropReason = info.wifi_sta_disconnected.reason;
}

/* Core 0, before WiFi.begin(). Records whether the hotspot is audible at all.
 * The S3's radio is 2.4 GHz only, so a 5 GHz hotspot never appears here. */
static void scanForHotspot() {
  const int n = WiFi.scanNetworks();   // blocks ~2 s, on core 0 only
  int nearby = 0;
  for (int i = 0; i < n; ++i) {
    const String ssid = WiFi.SSID(i);
    if (ssid == WIFI_SSID) {
      wifiSeenRssi = WiFi.RSSI(i);
      wifiSeenChannel = WiFi.channel(i);
    } else if (nearby < 4 && ssid.length() > 0) {
      strlcpy(wifiNearby[nearby++], ssid.c_str(), sizeof(wifiNearby[0]));
    }
  }
  WiFi.scanDelete();
  wifiScanCount.store(n, std::memory_order_release);   // after every write above
}

/* Core 0. Brings up the camera, the stream server and Wi-Fi, then exits.
 * setup() has already made the effector safe and started the servos, and
 * loop() is running on core 1 while this works, so a slow or failed camera
 * never delays a status frame or the deadman. */
static void cameraTask(void*) {
  WiFi.setHostname(CAM_HOSTNAME);   // before mode(): applied when STA starts
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);             // modem sleep adds tens of ms per frame
  cameraError = (int)startCamera();
  if (cameraError == ESP_OK) {
    sensor_t* sensor = esp_camera_sensor_get();
    cameraPid = sensor != nullptr ? sensor->id.PID : 0;
    startStreamServer();
    cameraState = 1;
  } else {
    cameraState = -1;
  }
  WiFi.onEvent(onWifiDisconnected, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  scanForHotspot();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);   // non-blocking; loop() reports the IP
  vTaskDelete(nullptr);
}

/* ESP-IDF wifi_err_reason_t, by number so it builds on IDF 4.4 and 5.x. */
static const char* wifiReasonText(int reason) {
  switch (reason) {
    case 0:   return "still connecting";
    case 2:   return "authentication timed out: weak signal or wrong password";
    case 15:
    case 204: return "password rejected: check WIFI_PASSWORD, capitals included";
    case 201: return "hotspot not found: set its band to 2.4 GHz and check WIFI_SSID";
    case 202: return "authentication failed: wrong password, or a WPA3-only hotspot (use WPA2)";
    case 203:
    case 205: return "hotspot refused the connection: too many devices on it?";
    case 210:
    case 211: return "security mode not accepted: set the hotspot to WPA2";
    default:  return "see the reason number";
  }
}

/* loop(), core 1. Says once whether the boot scan heard the hotspot. */
static void reportWifiScan(int networks) {
  Serial.print("CAM wifi \"");
  Serial.print(WIFI_SSID);
  if (networks < 0) {
    Serial.println("\": scan failed, joining anyway");
  } else if (wifiSeenRssi != 0) {
    Serial.print("\" heard on channel ");
    Serial.print(wifiSeenChannel);
    Serial.print(" at ");
    Serial.print(wifiSeenRssi);
    Serial.println(" dBm, joining");
  } else {
    Serial.print("\" NOT FOUND among ");
    Serial.print(networks);
    Serial.println(" networks: the S3 hears 2.4 GHz only (iPhone: Maximise"
                   " Compatibility), and the name is case-sensitive");
    if (wifiNearby[0][0] != '\0') {
      Serial.print("CAM wifi heard instead:");
      for (int i = 0; i < 4 && wifiNearby[i][0] != '\0'; ++i) {
        Serial.print(" \"");
        Serial.print(wifiNearby[i]);
        Serial.print('"');
      }
      Serial.println();
    }
  }
}

/* loop(), core 1. Prints the stream URL whenever Wi-Fi gains an address, and
 * while it has none, why not: once for the scan, then every 10 s. */
static void announceCamera() {
  static unsigned long lastCheckMs = 0;
  static unsigned long lastWifiNoteMs = 0;
  static int reportedState = 0;
  static bool scanReported = false;
  static IPAddress announced;
  static bool mdnsStarted = false;

  const unsigned long now = millis();
  if (now - lastCheckMs < 500) return;
  lastCheckMs = now;

  if (cameraState != reportedState) {
    reportedState = cameraState;
    if (cameraState < 0) {
      Serial.print("CAM FAIL camera init error 0x");
      Serial.print(cameraError, HEX);
      Serial.println(" - check the ribbon and the PSRAM setting");
    } else if (cameraState > 0) {
      Serial.print("CAM camera ok, sensor 0x");
      Serial.print(cameraPid, HEX);
      Serial.println(psramFound() ? ", 640x480"
                                  : ", 320x240 - no PSRAM: set Tools > PSRAM to OPI PSRAM");
    }
  }
  static int reportedStalls = 0;
  const int stalls = cameraStalls;
  if (stalls != reportedStalls) {
    reportedStalls = stalls;
    Serial.print("CAM sensor stalled, camera restarted (");
    Serial.print(stalls);
    Serial.println(stalls == 1 ? " time)" : " times)");
  }
  const int networks = wifiScanCount.load(std::memory_order_acquire);
  if (!scanReported && networks != SCAN_PENDING) {
    scanReported = true;
    lastWifiNoteMs = now;
    reportWifiScan(networks);
  }
  if (WiFi.status() == WL_CONNECTED) {
    const IPAddress ip = WiFi.localIP();
    if (ip != announced) {
      announced = ip;
      if (!mdnsStarted && MDNS.begin(CAM_HOSTNAME)) {
        MDNS.addService("http", "tcp", 80);
        mdnsStarted = true;
      }
      Serial.print("CAM http://");
      Serial.print(ip);
      Serial.println(cameraState > 0 ? "/stream" : "/stream (camera not ready)");
    }
  } else if (announced != IPAddress()) {
    announced = IPAddress();
    lastWifiNoteMs = now;
    Serial.println("CAM wifi lost");
  } else if (scanReported && now - lastWifiNoteMs >= 10000) {
    lastWifiNoteMs = now;
    const int reason = wifiDropReason;
    Serial.print("CAM wifi not joined: ");
    Serial.print(wifiReasonText(reason));
    Serial.print(" (reason ");
    Serial.print(reason);
    Serial.println(")");
  }
}
#endif

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
  panChannel  = servoPan.attach(PIN_SERVO_PAN,   SERVO_MIN_US, SERVO_MAX_US);
  tiltChannel = servoTilt.attach(PIN_SERVO_TILT, SERVO_MIN_US, SERVO_MAX_US);

  writePan(STOW_PAN);
  servoTilt.write((int)STOW_TILT);

  safeState();
  estopLatched  = true;
  lastCommandMs = millis();

  // Report PWM attach state at boot. A failed attach is otherwise invisible:
  // every command still succeeds, no pulses are ever emitted.
  Serial.print("OK BOOT killswitch-actuator v3.3 pan_ch=");
  Serial.print(panChannel);
  Serial.print(" tilt_ch=");
  Serial.print(tiltChannel);
  if (panChannel == 0 || tiltChannel == 0) {
    Serial.println(" SERVO ATTACH FAILED - NO PWM WILL BE GENERATED");
  } else {
    Serial.println(" pwm=ok");
  }

#if KS_CAMERA
  // Only now, with the effector safe and the servos stowed. Core 0.
  xTaskCreatePinnedToCore(cameraTask, "camera", 8192, nullptr, 1, nullptr, 0);
#endif
}

void loop() {
  pumpSerial();
  checkDeadman();      // before motion: safety decisions precede actuation
  checkBurnCeiling();
  updateServos();
  sendStatus();
#if KS_CAMERA
  announceCamera();
#endif
}
