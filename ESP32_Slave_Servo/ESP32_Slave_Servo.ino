// =======================
// SERVO SLAVE: ESP32
// Receives PKT_CMD {angle_deg, open_pct} and moves a hobby servo
// Uses Arduino Servo library for simplicity
// =======================

#include <WiFi.h>
#include <esp_now.h>
#include <Servo.h>
#include <math.h>

// ----------------------- Servo config -----------------------
// HS-422 typical: about 1000-2000 us for 0-180 deg.
// If you see clipping or not reaching endpoints, widen to 900-2100 or 500-2500 carefully.
static const int   SERVO_PIN = 23;
static const int   SERVO_MIN_US = 1000;  // us at 0 deg (adjust if needed)
static const int   SERVO_MAX_US = 2000;  // us at 180 deg (adjust if needed)
static const bool  ENABLE_RATE_LIMIT = true;
static const float MAX_DEG_STEP = 10.0f; // max degrees per command step if rate limiting

Servo valve;

// ----------------------- Packets -----------------------
enum : uint8_t { PKT_REQ = 1, PKT_RESP = 2, PKT_CMD = 3 };

typedef struct __attribute__((packed)) {
  uint8_t  type;
  uint32_t seq;
  uint8_t  node_id;
} MsgHeader;

typedef struct __attribute__((packed)) {
  MsgHeader h;
  float     angle_deg;  // 0..180 requested by master
} CmdMsg;

// ----------------------- Command handling -----------------------
static float g_last_deg = 0.0f;

static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

void setServoAngle(float deg) {
  // clamp to physical range
  float d = clampf(deg, 0.0f, 180.0f);

  // optional simple rate limiting
  if (ENABLE_RATE_LIMIT) {
    if (d > g_last_deg + MAX_DEG_STEP) d = g_last_deg + MAX_DEG_STEP;
    if (d < g_last_deg - MAX_DEG_STEP) d = g_last_deg - MAX_DEG_STEP;
  }

  valve.write((int)lroundf(d));  // because we attached with min/max us, this maps 0..180 to those pulse bounds
  g_last_deg = d;
}

// ----------------------- ESP-NOW callbacks -----------------------
void onDataSent(const uint8_t*, esp_now_send_status_t) {}

void onDataRecv(const uint8_t* mac, const uint8_t* payload, int len) {
  if (len < (int)sizeof(MsgHeader)) return;
  const MsgHeader* h = (const MsgHeader*)payload;

  if (h->type == PKT_CMD && len >= (int)sizeof(CmdMsg)) {
    const CmdMsg* c = (const CmdMsg*)payload;
    setServoAngle(c->angle_deg);  // servo closes loop internally
    // no ACK by design
  }
}

// ----------------------- Setup -----------------------
void setup() {
  Serial.begin(115200);
  delay(100);

  // Attach servo with explicit microsecond bounds for better calibration
  valve.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  setServoAngle(0.0f);  // park at 0 deg on boot

  // ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) { delay(1000); }
  }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  Serial.println("Servo slave ready (Servo library).");
}

void loop() {
  // Nothing to do - waits for PKT_CMD and moves the servo
  vTaskDelay(pdMS_TO_TICKS(200));
}
