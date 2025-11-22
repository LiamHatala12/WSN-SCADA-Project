// =======================
// SENSOR SLAVE: ESP32
// Responds to PKT_REQ with PKT_RESP {flow_rate_lpm, level_cm}
// =======================

#include <WiFi.h>
#include <esp_now.h>

// ----------------------- Pins (example) -----------------------
// Ultrasonic HC-SR04 style
const int TRIG_PIN = 5;
const int ECHO_PIN = 18;

// Flow sensor (e.g., YF-S201) on GPIO 19 (interrupt-capable)
const int FLOW_PIN = 19;

// ----------------------- Flow counting -----------------------
volatile uint32_t g_flowPulses = 0;
void IRAM_ATTR flowISR() {
  g_flowPulses++;
}

// Convert pulses to L/min (adjust constants for your sensor)
float pulsesToLpm(uint32_t pulses, float dt_s) {
  // Example: YF-S201 ~ 450 pulses/L at nominal spec -> 7.5 pulses/sec per L/min
  // L/min = pulses / (7.5 * dt_s)
  const float K = 7.5f;
  return (dt_s > 1e-6f) ? (pulses / (K * dt_s)) : 0.0f;
}

// ----------------------- Ultrasonic -----------------------
float readUltrasonicCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);  // microseconds
  // distance in cm: duration * 0.034 / 2
  float cm = (duration * 0.034f) / 2.0f;
  return cm;
}

// ----------------------- Packets -----------------------
enum : uint8_t { PKT_REQ = 1,
                 PKT_RESP = 2,
                 PKT_CMD = 3 };

typedef struct __attribute__((packed)) {
  float flow_rate_lpm;
  float level_cm;
} SensorData;

typedef struct __attribute__((packed)) {
  uint8_t type;
  uint32_t seq;
  uint8_t node_id;
} MsgHeader;

typedef struct __attribute__((packed)) {
  MsgHeader h;
  uint32_t request_mask;
} ReqMsg;

typedef struct __attribute__((packed)) {
  MsgHeader h;
  SensorData data;
} RespMsg;

static SemaphoreHandle_t sendMutex;

void onDataSent(const uint8_t *, esp_now_send_status_t) {}

void onDataRecv(const uint8_t *mac, const uint8_t *payload, int len) {
  if (len < (int)sizeof(MsgHeader)) return;
  const MsgHeader *h = (const MsgHeader *)payload;

  if (h->type == PKT_REQ && len >= (int)sizeof(ReqMsg)) {
    static uint32_t lastCount = 0;
    static uint32_t lastMicros = micros();

    uint32_t nowUs = micros();
    float dt_s = (nowUs - lastMicros) / 1e6f;
    if (dt_s < 0) dt_s = 0;
    uint32_t cnt = g_flowPulses;
    uint32_t dcnt = cnt - lastCount;

    float level_cm = readUltrasonicCm();
    float lpm = pulsesToLpm(dcnt, dt_s);

    lastMicros = nowUs;
    lastCount = cnt;

    RespMsg r{};
    r.h.type = PKT_RESP;
    r.h.seq = h->seq;  // mirror seq
    r.h.node_id = h->node_id;
    r.data.flow_rate_lpm = lpm;
    r.data.level_cm = level_cm;

    if (xSemaphoreTakeFromISR(sendMutex, nullptr) == pdTRUE) {
      esp_now_send(mac, (uint8_t *)&r, sizeof(r));
      xSemaphoreGiveFromISR(sendMutex, nullptr);
    } else {
      esp_now_send(mac, (uint8_t *)&r, sizeof(r));
    }
  }
}

bool ensurePeer(const uint8_t mac[6]) {
  esp_now_peer_info_t p{};
  memcpy(p.peer_addr, mac, 6);
  p.channel = 0;
  p.encrypt = false;
  if (!esp_now_is_peer_exist(mac)) {
    if (esp_now_add_peer(&p) != ESP_OK) return false;
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // sensors
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(FLOW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flowISR, RISING);

  // esp-now
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW fail");
    while (1) { delay(1000); }
  }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  sendMutex = xSemaphoreCreateMutex();

  Serial.println("Sensor slave ready.");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(500));
}
