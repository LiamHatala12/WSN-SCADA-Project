// =======================
// MASTER: ESP32 + ESP-NOW + FreeRTOS
// Encoder -> Setpoint (cm), Sensor PV -> PID -> Servo Angle Command
// LCD shows SP, PV, Flow, OUT
// =======================

#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ----------------------- LCD -----------------------
#define LCD_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

// ----------------------- ESP-NOW Peers -----------------------
static const uint8_t SLAVES[][6] = {
  // index 0: SENSOR SLAVE MAC (ultrasonic + flow)
  { 0x24, 0x6F, 0x28, 0x00, 0x00, 0x01 }, // TODO: get correct MAC
  // index 1: SERVO SLAVE MAC (valve)
  { 0x24, 0x6F, 0x28, 0x00, 0x00, 0x02 }, // TODO: get correct MAC
};
static const size_t NUM_SLAVES = sizeof(SLAVES) / 6;

// identify indices within SLAVES MAC address array
static const uint8_t IDX_SENSOR = 0;
static const uint8_t IDX_SERVO = 1;

// ----------------------- Timing -----------------------
static const TickType_t POLL_TIMEOUT = pdMS_TO_TICKS(200);
static const uint8_t MAX_RETRIES = 2;
static const TickType_t GAP_BETWEEN_TX = pdMS_TO_TICKS(40);
static const float PID_TS_S = 0.050f;     // 50 ms PID period
static const uint32_t SERVO_CMD_HZ = 10;  // send at 10 Hz
static const TickType_t LCD_UPDATE_MS = pdMS_TO_TICKS(250);

// ----------------------- Packets -----------------------
enum : uint8_t { PKT_REQ = 1,
                 PKT_RESP = 2,
                 PKT_CMD = 3 };

typedef struct __attribute__((packed)) {
  float flow_rate_lpm;  // L/min (example)
  float level_cm;       // cm
} SensorData;

typedef struct __attribute__((packed)) {
  uint8_t type;
  uint32_t seq;
  uint8_t node_id;
} MsgHeader;

typedef struct __attribute__((packed)) {
  MsgHeader h;
  uint32_t request_mask;  // reserved
} ReqMsg;

typedef struct __attribute__((packed)) {
  MsgHeader h;
  SensorData data;
} RespMsg;

typedef struct __attribute__((packed)) {
  MsgHeader h;
  float angle_deg;  // 0..180
  float open_pct;   // 0..100
} CmdMsg;

// ----------------------- System Config -----------------------
struct SystemConfig {
  // level setpoint mapping from encoder counts
  float enc_counts_to_cm;  // cm per count
  float min_sp_cm;
  float max_sp_cm;

  // PID
  float Kc;  // proportional gain
  float Ti;  // integral time [s]
  float Td;  // derivative time [s]

  // output mapping (controller out to servo)
  float out_min_pct;  // 0%
  float out_max_pct;  // 100%
  float pct_to_deg;   // e.g., 180/100

  // LCD text scaling/format
  uint8_t pv_slave_index;  // which peer provides PV (ultrasonic)
} CFG = {
  .enc_counts_to_cm = 0.10f,  // cm / count (adjust)
  .min_sp_cm = 0.0f,
  .max_sp_cm = 80.0f,  // tank height
  .Kc = 1.0f,
  .Ti = 0.7f,
  .Td = 0.02f,
  .out_min_pct = 0.0f,
  .out_max_pct = 100.0f,
  .pct_to_deg = 1.8f,  // 100% -> 180 deg
  .pv_slave_index = IDX_SENSOR
};

// ----------------------- Encoder pins -----------------------
static const int ENC_A_PIN = 32;
static const int ENC_B_PIN = 33;

// ----------------------- Globals -----------------------
static volatile long g_enc_counts = 0;

static QueueHandle_t rxQueue;
static SemaphoreHandle_t sendMutex;
static SemaphoreHandle_t stateMutex;

static volatile uint32_t g_seq = 1;

// shared state
static float g_sp_cm = 0.0f;  // setpoint (cm)
static float g_pv_cm = 0.0f;  // PV level (cm)
static float g_flow = 0.0f;   // flow (L/min)
static float g_u_pct = 0.0f;  // controller output (% open)
static float g_u_deg = 0.0f;  // controller output (deg)

static bool g_have_pv = false;

// ----------------------- Encoder ISRs -----------------------
static inline int readB() {
  return digitalRead(ENC_B_PIN);
}
void IRAM_ATTR isrA() {
  g_enc_counts += (readB() ? -1 : +1);
}
void IRAM_ATTR isrB() {
  g_enc_counts += (digitalRead(ENC_A_PIN) ? +1 : -1);
}

// ----------------------- Helpers -----------------------
float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

float encoderCountsToCm(long counts) {
  float cm = counts * CFG.enc_counts_to_cm;
  return clampf(cm, CFG.min_sp_cm, CFG.max_sp_cm);
}

void lcdPrintStats() {
  static TickType_t last = 0;
  TickType_t now = xTaskGetTickCount();
  if (now - last < LCD_UPDATE_MS) return;
  last = now;

  lcd.clear();
  // Line 1: SP:xx.x PV:yy.y
  lcd.setCursor(0, 0);
  lcd.print("SP:");
  lcd.print(g_sp_cm, 1);
  lcd.print(" PV:");
  lcd.print(g_pv_cm, 1);

  // Line 2: F:aa.a O:bbb
  lcd.setCursor(0, 1);
  lcd.print("F:");
  lcd.print(g_flow, 1);
  lcd.print(" O:");
  lcd.print((int)roundf(g_u_pct));
}

// ----------------------- ESP-NOW plumbing -----------------------
void onDataSent(const uint8_t*, esp_now_send_status_t) {}

void onDataRecv(const uint8_t* mac, const uint8_t* payload, int len) {
  if (len < (int)sizeof(MsgHeader)) return;
  const MsgHeader* h = (const MsgHeader*)payload;

  if (h->type == PKT_RESP && len >= (int)sizeof(RespMsg)) {
    const RespMsg* r = (const RespMsg*)payload;

    if (h->node_id == CFG.pv_slave_index) {
      // update PV / flow
      if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        g_pv_cm = r->data.level_cm;
        g_flow = r->data.flow_rate_lpm;
        g_have_pv = true;
        xSemaphoreGive(stateMutex);
      }
    }

    // also forward to queue (optional)
    // (not strictly needed; PID reads from shared state)
    // we keep this if you'd like to log later
  }
}

bool ensurePeer(const uint8_t mac[6]) {
  esp_now_peer_info_t p{};
  memcpy(p.peer_addr, mac, 6);
  p.channel = 0;
  p.encrypt = false;
  if (!esp_now_is_peer_exist(mac)) {
    if (esp_now_add_peer(&p) != ESP_OK) {
      Serial.println("add_peer failed");
      return false;
    }
  }
  return true;
}

bool sendPacket(const uint8_t mac[6], const uint8_t* data, size_t n) {
  if (xSemaphoreTake(sendMutex, pdMS_TO_TICKS(200)) != pdTRUE) return false;
  bool ok = (esp_now_send(mac, data, n) == ESP_OK);
  xSemaphoreGive(sendMutex);
  return ok;
}

// ----------------------- Tasks -----------------------
void taskPoll(void*) {
  size_t idx = 0;
  while (true) {
    if (NUM_SLAVES == 0) {
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    // Only poll the SENSOR slave
    if (idx == IDX_SENSOR) {
      const uint8_t* mac = SLAVES[idx];
      ensurePeer(mac);

      ReqMsg req{};
      req.h.type = PKT_REQ;
      req.h.seq = g_seq++;
      req.h.node_id = idx;
      req.request_mask = 0;

      for (uint8_t attempt = 0; attempt <= MAX_RETRIES; attempt++) {
        sendPacket(mac, (uint8_t*)&req, sizeof(req));
        // We do not block here waiting — callback updates state when resp arrives
        vTaskDelay(GAP_BETWEEN_TX);
      }
    }

    // move to next peer
    idx = (idx + 1) % NUM_SLAVES;
    vTaskDelay(GAP_BETWEEN_TX);
  }
}

void taskPID(void*) {
  float e_prev = 0.0f;
  float Iacc = 0.0f;
  TickType_t last = xTaskGetTickCount();

  uint32_t servoTick = 0;
  const uint32_t servoEvery = (uint32_t)roundf(1.0f / PID_TS_S / SERVO_CMD_HZ);  // sends per PID loops

  while (true) {
    // 1) Read SP from encoder, clamp to tank height
    float sp = encoderCountsToCm(g_enc_counts);

    // 2) Snapshot PV (hold last if not updated)
    float pv, flow;
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      pv = g_pv_cm;
      flow = g_flow;
      g_sp_cm = sp;
      xSemaphoreGive(stateMutex);
    }

    // 3) PID (parallel form)
    float e = sp - pv;
    float Iterm = (CFG.Ti > 1e-6f) ? (Iacc + (PID_TS_S / CFG.Ti) * e) : Iacc;
    float Dterm = (CFG.Td > 1e-6f) ? ((e - e_prev) / PID_TS_S) : 0.0f;

    float u = CFG.Kc * (e + Iterm + CFG.Td * Dterm);  // unconstrained controller output in "percent-equivalent"
    // clamp to [0..100]% with simple back-calc anti-windup
    float u_pct = u;
    if (u_pct > CFG.out_max_pct) {
      if (CFG.Kc > 1e-6f) Iterm -= (u_pct - CFG.out_max_pct) / CFG.Kc;
      u_pct = CFG.out_max_pct;
    } else if (u_pct < CFG.out_min_pct) {
      if (CFG.Kc > 1e-6f) Iterm += (CFG.out_min_pct - u_pct) / CFG.Kc;
      u_pct = CFG.out_min_pct;
    }

    Iacc = Iterm;
    e_prev = e;

    // 4) Convert to servo angle
    float angle = u_pct * CFG.pct_to_deg;  // 0..100 -> 0..180 deg

    // 5) Save shared output + LCD
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
      g_u_pct = u_pct;
      g_u_deg = angle;
      xSemaphoreGive(stateMutex);
    }
    lcdPrintStats();

    // 6) Periodically send command to servo slave
    if (servoTick++ >= servoEvery) {
      servoTick = 0;
      CmdMsg cmd{};
      cmd.h.type = PKT_CMD;
      cmd.h.seq = g_seq++;
      cmd.h.node_id = IDX_SERVO;
      cmd.angle_deg = angle;
      cmd.open_pct = u_pct;
      ensurePeer(SLAVES[IDX_SERVO]);
      sendPacket(SLAVES[IDX_SERVO], (uint8_t*)&cmd, sizeof(cmd));
    }

    vTaskDelayUntil(&last, pdMS_TO_TICKS((int)(PID_TS_S * 1000)));
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WSN Master PID");
  lcd.setCursor(0, 1);
  lcd.print("Init...");

  // Encoder
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), isrB, CHANGE);

  // WiFi + ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    lcd.clear();
    lcd.print("ESP-NOW FAIL");
    while (1) { delay(1000); }
  }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);
  for (size_t i = 0; i < NUM_SLAVES; i++) ensurePeer(SLAVES[i]);

  // RTOS
  rxQueue = xQueueCreate(8, sizeof(RespMsg));  // reserved if you want logging later
  sendMutex = xSemaphoreCreateMutex();
  stateMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(taskPoll, "Poll", 4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(taskPID, "PID", 4096, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(taskHMI, "PID", 4096, nullptr, 3, nullptr, 1);


  lcd.clear();
  lcd.print("Ready");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
