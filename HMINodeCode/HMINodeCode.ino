#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>
#include <vector>
#include <new>

/* ESP-NOW config */
#define ESPNOW_WIFI_IFACE       WIFI_IF_STA
#define ESPNOW_WIFI_CHANNEL     4
#define ESPNOW_COMM_INTERVAL_MS 100
#define ESPNOW_PEER_COUNT       1

#define ESPNOW_EXAMPLE_PMK "pmk1234567890123"
#define ESPNOW_EXAMPLE_LMK "lmk1234567890123"

/* Role priorities */
#define PRIORITY_HMI   0
#define PRIORITY_HEAD  5

/* Message types and struct (shared) */
enum MsgType : uint8_t {
  MSG_DISCOVERY       = 0,
  MSG_POLL_SENSOR     = 1,
  MSG_SENSOR_DATA     = 2,
  MSG_POLL_HMI        = 3,
  MSG_HMI_SETPOINT    = 4,
  MSG_CONTROL_COMMAND = 5,
  MSG_CONTROL_STATUS  = 6,
  MSG_SERVO_SETPOINT  = 7,
  MSG_SERVO_COMMAND   = 8
};

/* Shared data struct on all nodes */
// HMI / Sensor / Servo / Pump
typedef struct {
  uint8_t  msg_type;
  uint8_t  rsvd[3];
  uint32_t count;
  uint32_t priority;
  int32_t  data;
  int32_t  data2;
  int32_t  data3;
  int32_t  data4;
  bool     ready;
} __attribute__((packed)) esp_now_data_t;

/* TFT */
#define TFT_CS   5
#define TFT_DC   2
#define TFT_RST  4
#define BL_PIN   32

/* Encoder */
#define CLK_PIN  21
#define DT_PIN   22
#define SW_PIN   19

#define MAX_CM    24
#define MAX_SERVO 90  // servo disturbance range

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

/* Modes */
enum EditMode {
  MODE_SETPOINT = 0,
  MODE_P        = 1,
  MODE_I        = 2,
  MODE_D        = 3,
  MODE_SERVO    = 4,
  MODE_COUNT    = 5
};

/* HMI state */
int setpoint          = 12;
int setpointPercent   = 0;
int waterLevel        = 0;
int waterLevelPercent = 0;
int volume            = 0;
int servoPosition     = 45;   // disturbance angle
int pumpPower         = 0;

// PID parameters scaled by 10
int Kp_x10 = 20;
int Ki_x10 = 5;
int Kd_x10 = 10;

/* Encoder */
volatile int counter         = 0;
volatile int lastEncoded     = 0;
int          lastCounterValue = 0;

/* Mode/button */
volatile EditMode currentMode = MODE_SETPOINT;
volatile bool     buttonPressed = false;
unsigned long     lastButtonTime = 0;
const unsigned long DEBOUNCE_DELAY = 200;

/* Display tracking */
int      lastSetpoint          = -1;
int      lastSetpointPercent   = -1;
int      lastWaterLevel        = -1;
int      lastWaterLevelPercent = -1;
int      lastVolume            = -1;
int      lastServoPosition     = -1;
int      lastPumpPower         = -1;
int      lastKp_x10            = -1;
int      lastKi_x10            = -1;
int      lastKd_x10            = -1;
EditMode lastMode              = MODE_COUNT;

/* ESP-NOW globals */
uint32_t self_priority      = PRIORITY_HMI;
uint8_t  current_peer_count = 0;
bool     device_is_master   = false;
bool     master_decided     = false;
uint32_t sent_msg_count     = 0;
uint32_t recv_msg_count     = 0;
esp_now_data_t new_msg;

/* Peer class */
class ESP_NOW_Network_Peer : public ESP_NOW_Peer {
public:
  uint32_t priority;
  bool     peer_is_master = false;
  bool     peer_ready     = false;

  ESP_NOW_Network_Peer(const uint8_t *mac_addr,
                       uint32_t priority = 0,
                       const uint8_t *lmk = (const uint8_t *)ESPNOW_EXAMPLE_LMK)
    : ESP_NOW_Peer(mac_addr, ESPNOW_WIFI_CHANNEL, ESPNOW_WIFI_IFACE, lmk),
      priority(priority) {}

  bool begin() {
    if (!add()) {
      log_e("Failed to register peer");
      return false;
    }
    return true;
  }

  bool send_message(const uint8_t *data, size_t len) {
    if (!data || len == 0) return false;
    return send(data, len);
  }

  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    Serial.printf("[HMI_RX] len=%d, broadcast=%d\n", len, broadcast);
    
    esp_now_data_t *msg = (esp_now_data_t *)data;

    if (!peer_ready && msg->ready) {
      Serial.printf("Peer " MACSTR " reported ready\n", MAC2STR(addr()));
      peer_ready = true;
    }

    if (broadcast) {
      Serial.println("[HMI_RX] Ignoring broadcast");
      return;
    }

    recv_msg_count++;
    Serial.printf("[HMI_RX] msg_type=%d, peer_is_master=%d\n", msg->msg_type, peer_is_master);

    if (!peer_is_master) {
      Serial.println("[HMI_RX] Ignoring - sender not master");
      return;
    }

    switch (msg->msg_type) {
      case MSG_CONTROL_STATUS:
        Serial.println("[HMI_RX] Processing CONTROL_STATUS");
        // data  = waterLevel
        // data2 = pumpPower
        // data3 = servoPosition (optional)
        waterLevel = (int)msg->data;
        pumpPower  = (int)msg->data2;
        // Optional feedback of servo, but we keep local user value as source of truth
        Serial.printf("Status from HEAD " MACSTR ": Level=%d cm, Pump=%d %%\n",
                      MAC2STR(addr()), waterLevel, pumpPower);
        break;

      case MSG_POLL_HMI: {
        Serial.println("[HMI_RX] Processing POLL_HMI - sending response");
        // Head polling for setpoint, PID, and servo disturbance
        esp_now_data_t reply;
        memset(&reply, 0, sizeof(reply));
        reply.msg_type = MSG_HMI_SETPOINT;
        reply.priority = self_priority;
        reply.count    = ++sent_msg_count;
        reply.data     = (int32_t)setpoint;
        reply.data2    = (int32_t)Kp_x10;
        reply.data3    = (int32_t)Ki_x10;
        reply.data4    = (int32_t)Kd_x10;
        reply.ready    = true;

        if (!send_message((const uint8_t *)&reply, sizeof(reply))) {
          Serial.println("[HMI_TX] Failed to send HMI_SETPOINT to HEAD");
        } else {
          Serial.printf("[HMI_TX] Sent SP=%d, Kp_x10=%d, Ki_x10=%d, Kd_x10=%d to HEAD\n",
                        setpoint, Kp_x10, Ki_x10, Kd_x10);
        }

        // Send servo disturbance separately
        esp_now_data_t reply2;
        memset(&reply2, 0, sizeof(reply2));
        reply2.msg_type = MSG_SERVO_SETPOINT;
        reply2.priority = self_priority;
        reply2.count    = ++sent_msg_count;
        reply2.data     = (int32_t)servoPosition;
        reply2.ready    = true;

        if (!send_message((const uint8_t *)&reply2, sizeof(reply2))) {
          Serial.println("[HMI_TX] Failed to send SERVO_SETPOINT to HEAD");
        } else {
          Serial.printf("[HMI_TX] Sent Servo=%d deg to HEAD\n", servoPosition);
        }
        break;
      }

      default:
        Serial.printf("[HMI_RX] Unknown msg_type=%d\n", msg->msg_type);
        break;
    }
  }

  void onSent(bool success) {
    log_i("Message sent %s", success ? "successfully" : "failed");
  }
};

std::vector<ESP_NOW_Network_Peer *> peers;
ESP_NOW_Network_Peer broadcast_peer(ESP_NOW.BROADCAST_ADDR, 0, nullptr);
ESP_NOW_Network_Peer *master_peer = nullptr;

/* Helpers */
void IRAM_ATTR updateEncoder() {
  int MSB = digitalRead(CLK_PIN);
  int LSB = digitalRead(DT_PIN);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) counter++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) counter--;

  lastEncoded = encoded;
}

void IRAM_ATTR buttonISR() {
  buttonPressed = true;
}

void fail_reboot() {
  Serial.println("ERROR: Rebooting in 5 seconds...");
  tft.fillScreen(ST77XX_RED);
  tft.setCursor(10, 150);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE);
  tft.println("ERROR!");
  tft.println("Rebooting...");
  delay(5000);
  ESP.restart();
}

uint32_t check_highest_priority() {
  uint32_t highest_priority = 0;
  for (auto &peer : peers) {
    if (peer->priority > highest_priority) highest_priority = peer->priority;
  }
  return std::max(highest_priority, self_priority);
}

bool check_all_peers_ready() {
  for (auto &peer : peers) {
    if (!peer->peer_ready) return false;
  }
  return true;
}

int calculateVolume(int level_cm) {
  const float TANK_RADIUS = 10.75f;
  float volume_ml = 3.14159f * TANK_RADIUS * TANK_RADIUS * level_cm;
  return (int)volume_ml;
}

/* UI drawing */
void drawGUI() {
  tft.fillScreen(ST77XX_BLACK);
  tft.drawRect(0, 20, 240, 300, ST77XX_WHITE);

  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE);

  tft.setCursor(10, 30);
  tft.println("SETPOINT");
  tft.drawLine(5, 95, 235, 95, ST77XX_WHITE);

  tft.setCursor(10, 105);
  tft.println("PID TUNING");
  tft.drawLine(5, 145, 235, 145, ST77XX_WHITE);

  tft.setCursor(10, 155);
  tft.println("WATER LEVEL");
  tft.drawLine(5, 220, 235, 220, ST77XX_WHITE);

  tft.setCursor(10, 230);
  tft.println("ACTUATORS");
  tft.setCursor(10, 255);
  tft.print("Servo:");
  tft.setCursor(130, 255);
  tft.print("Pump:");
}

void updateModeIndicator() {
  if (currentMode != lastMode) {
    tft.fillRect(5, 5, 230, 10, ST77XX_BLACK);
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(10, 5);
    tft.print("MODE: ");
    switch (currentMode) {
      case MODE_SETPOINT: tft.print("Setpoint"); break;
      case MODE_P:        tft.print("Proportional"); break;
      case MODE_I:        tft.print("Integral"); break;
      case MODE_D:        tft.print("Derivative"); break;
      case MODE_SERVO:    tft.print("Servo Position"); break;
      default: break;
    }
    lastMode = currentMode;
  }
}

/* Value helpers */
int getCurrentValue() {
  switch (currentMode) {
    case MODE_SETPOINT: return setpoint;
    case MODE_P:        return Kp_x10;
    case MODE_I:        return Ki_x10;
    case MODE_D:        return Kd_x10;
    case MODE_SERVO:    return servoPosition / 5;  // step of 5 degrees
    default:            return 0;
  }
}

void setCurrentValue(int value) {
  switch (currentMode) {
    case MODE_SETPOINT:
      setpoint = constrain(value, 0, MAX_CM);
      break;
    case MODE_P:
      Kp_x10 = constrain(value, 0, 1000);
      break;
    case MODE_I:
      Ki_x10 = constrain(value, 0, 1000);
      break;
    case MODE_D:
      Kd_x10 = constrain(value, 0, 1000);
      break;
    case MODE_SERVO:
      servoPosition = constrain(value * 5, 0, MAX_SERVO);
      break;
    default:
      break;
  }
}

/* UI update sections */
void updateSetpointDisplay() {
  if (setpoint != lastSetpoint) {
    setpointPercent = (setpoint * 100) / MAX_CM;

    tft.fillRect(20, 55, 200, 30, ST77XX_BLACK);

    uint16_t color = (currentMode == MODE_SETPOINT) ? ST77XX_YELLOW : ST77XX_CYAN;

    tft.setTextSize(4);
    tft.setTextColor(color);
    tft.setCursor(20, 55);
    tft.print(setpoint);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.print(" cm");

    tft.setTextSize(3);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(140, 60);
    tft.print(setpointPercent);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.print("%");

    lastSetpoint = setpoint;
  }
}

void updatePIDDisplay() {
  if (Kp_x10 != lastKp_x10 || Ki_x10 != lastKi_x10 || Kd_x10 != lastKd_x10 || currentMode != lastMode) {
    tft.fillRect(10, 120, 220, 20, ST77XX_BLACK);
    tft.setTextSize(2);

    tft.setCursor(10, 125);
    tft.setTextColor(ST77XX_WHITE);
    tft.print("P:");
    uint16_t colorP = (currentMode == MODE_P) ? ST77XX_YELLOW : ST77XX_CYAN;
    tft.setTextColor(colorP);
    tft.print(Kp_x10 / 10);
    tft.print(".");
    tft.print(Kp_x10 % 10);

    tft.setTextColor(ST77XX_WHITE);
    tft.print(" I:");
    uint16_t colorI = (currentMode == MODE_I) ? ST77XX_YELLOW : ST77XX_GREEN;
    tft.setTextColor(colorI);
    tft.print(Ki_x10 / 10);
    tft.print(".");
    tft.print(Ki_x10 % 10);

    tft.setTextColor(ST77XX_WHITE);
    tft.print(" D:");
    uint16_t colorD = (currentMode == MODE_D) ? ST77XX_YELLOW : ST77XX_MAGENTA;
    tft.setTextColor(colorD);
    tft.print(Kd_x10 / 10);
    tft.print(".");
    tft.print(Kd_x10 % 10);

    lastKp_x10 = Kp_x10;
    lastKi_x10 = Ki_x10;
    lastKd_x10 = Kd_x10;
  }
}

void updateWaterLevelDisplay() {
  if (waterLevel != lastWaterLevel) {
    waterLevelPercent = (waterLevel * 100) / MAX_CM;
    volume = calculateVolume(waterLevel);

    tft.fillRect(20, 175, 200, 35, ST77XX_BLACK);

    tft.setTextSize(4);
    tft.setTextColor(ST77XX_GREEN);
    tft.setCursor(20, 180);
    tft.print(waterLevel);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.print(" cm");

    tft.setTextSize(3);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(140, 185);
    tft.print(waterLevelPercent);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.print("%");

    tft.fillRect(70, 210, 150, 20, ST77XX_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(20, 210);
    tft.print("Vol:");
    tft.setTextSize(3);
    tft.setTextColor(ST77XX_MAGENTA);
    tft.setCursor(70, 210);
    tft.print(volume);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.print(" mL");

    lastWaterLevel = waterLevel;
  }
}

void updateActuatorsDisplay() {
  // Servo
  if (servoPosition != lastServoPosition || currentMode != lastMode) {
    tft.fillRect(10, 275, 105, 25, ST77XX_BLACK);
    tft.setTextSize(3);
    uint16_t servoColor = (currentMode == MODE_SERVO) ? ST77XX_YELLOW : ST77XX_ORANGE;
    tft.setTextColor(servoColor);
    tft.setCursor(10, 275);
    tft.print(servoPosition);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.print(" deg");
    lastServoPosition = servoPosition;
  }

  // Pump
  if (pumpPower != lastPumpPower) {
    tft.fillRect(130, 275, 90, 25, ST77XX_BLACK);
    tft.setTextSize(3);
    tft.setTextColor(ST77XX_RED);
    tft.setCursor(130, 275);
    tft.print(pumpPower);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.print("%");
    lastPumpPower = pumpPower;
  }
}

void updateDisplay() {
  updateModeIndicator();
  updateSetpointDisplay();
  updatePIDDisplay();
  updateWaterLevelDisplay();
  updateActuatorsDisplay();
}

/* Encoder and button handling */
void handleModeChange() {
  if (buttonPressed) {
    unsigned long now = millis();
    if (now - lastButtonTime > DEBOUNCE_DELAY) {
      lastButtonTime = now;
      currentMode = (EditMode)((currentMode + 1) % MODE_COUNT);
      counter = getCurrentValue();
      lastCounterValue = counter;

      lastSetpoint       = -1;
      lastKp_x10         = -1;
      lastKi_x10         = -1;
      lastKd_x10         = -1;
      lastServoPosition  = -1;
    }
    buttonPressed = false;
  }
}

void handleEncoderChange() {
  int newValue = counter;
  if (newValue != lastCounterValue) {
    setCurrentValue(newValue);
    lastCounterValue = newValue;
  }
}

/* New peer callback */
void register_new_peer(const esp_now_recv_info_t *info,
                       const uint8_t *data,
                       int len,
                       void *arg) {
  esp_now_data_t *msg = (esp_now_data_t *)data;
  int priority = (int)msg->priority;

  if ((uint32_t)priority == self_priority) {
    Serial.println("ERROR: duplicate priority");
    fail_reboot();
  }

  if (current_peer_count >= ESPNOW_PEER_COUNT) return;

  Serial.printf("New peer found: " MACSTR " with priority %d\n",
                MAC2STR(info->src_addr), priority);

  ESP_NOW_Network_Peer *new_peer =
    new (std::nothrow) ESP_NOW_Network_Peer(info->src_addr, priority);

  if (new_peer == nullptr || !new_peer->begin()) {
    Serial.println("Failed to register peer");
    delete new_peer;
    return;
  }

  peers.push_back(new_peer);
  current_peer_count++;

  if ((uint32_t)priority > self_priority) {
    new_peer->peer_is_master = true;
    master_peer = new_peer;
    Serial.println("Peer identified as HEAD master");
  }

  if (current_peer_count == ESPNOW_PEER_COUNT) {
    Serial.println("All peers found");
    new_msg.ready = true;
  }
}

/* Setup */
void setup() {
  pinMode(BL_PIN, OUTPUT);
  digitalWrite(BL_PIN, HIGH);

  Serial.begin(115200);
  delay(100);
  Serial.println("=== Water Tank HMI with PID + Servo disturbance ===");

  tft.init(240, 320);
  tft.setRotation(0);
  drawGUI();

  pinMode(CLK_PIN, INPUT_PULLUP);
  pinMode(DT_PIN, INPUT_PULLUP);
  pinMode(SW_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(CLK_PIN), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DT_PIN), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SW_PIN), buttonISR, FALLING);

  counter = setpoint;
  lastCounterValue = counter;

  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) delay(10);

  self_priority = PRIORITY_HMI;

  Serial.println("WiFi initialized");
  Serial.printf("MAC: %s\n", WiFi.macAddress().c_str());
  Serial.printf("Channel: %d\n", ESPNOW_WIFI_CHANNEL);
  Serial.printf("Priority: %lu (HMI)\n", self_priority);

  tft.setCursor(5, 5);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_GREEN);
  tft.print(WiFi.macAddress());
  tft.print(" | Ch:");
  tft.print(ESPNOW_WIFI_CHANNEL);
  tft.drawLine(0, 15, 240, 15, ST77XX_WHITE);

  if (!ESP_NOW.begin((const uint8_t *)ESPNOW_EXAMPLE_PMK)) {
    Serial.println("ESP-NOW init failed");
    fail_reboot();
  }

  Serial.printf("ESP-NOW version: %d, max data length: %d\n",
                ESP_NOW.getVersion(), ESP_NOW.getMaxDataLen());

  if (!broadcast_peer.begin()) {
    Serial.println("Broadcast peer init failed");
    fail_reboot();
  }

  ESP_NOW.onNewPeer(register_new_peer, nullptr);

  memset(&new_msg, 0, sizeof(new_msg));
  new_msg.msg_type = MSG_DISCOVERY;
  new_msg.priority = self_priority;
  new_msg.ready    = false;

  Serial.println("HMI setup complete");
}

/* Loop */
/* Loop */
void loop() {
  static unsigned long lastCommTime = 0;
  unsigned long now = millis();

  if (!master_decided) {
    if (now - lastCommTime >= ESPNOW_COMM_INTERVAL_MS) {
      lastCommTime = now;
      // Update ready flag if all peers found
      if (current_peer_count >= ESPNOW_PEER_COUNT) {
        new_msg.ready = true;
      }
      broadcast_peer.send_message((const uint8_t *)&new_msg, sizeof(new_msg));

      if (current_peer_count == ESPNOW_PEER_COUNT && check_all_peers_ready()) {
        master_decided = true;
        uint32_t highest_priority = check_highest_priority();
        device_is_master = (highest_priority == self_priority);

        if (device_is_master) {
          Serial.println(">>> HMI became master (unexpected)");
        } else {
          Serial.println(">>> HMI is slave, HEAD is master");
          for (auto &peer : peers) {
            if (peer->priority == highest_priority) {
              peer->peer_is_master = true;
              master_peer = peer;
              break;
            }
          }
        }
      }
    }
  }

  handleModeChange();
  handleEncoderChange();
  updateDisplay();

  delay(20);
}
