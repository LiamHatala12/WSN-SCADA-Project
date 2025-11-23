#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>
#include <vector>
#include <new>

/* Definitions */
#define ESPNOW_WIFI_IFACE       WIFI_IF_STA
#define ESPNOW_WIFI_CHANNEL     4
#define ESPNOW_COMM_INTERVAL_MS 100
#define ESPNOW_PEER_COUNT       1
#define REPORT_INTERVAL         5

#define ESPNOW_EXAMPLE_PMK "pmk1234567890123"
#define ESPNOW_EXAMPLE_LMK "lmk1234567890123"

#define PRIORITY_HMI  0
#define PRIORITY_HEAD 5

// TFT Screen pins
#define TFT_CS   5
#define TFT_DC   2
#define TFT_RST  4
#define BL_PIN   32

// Rotary Encoder pins
#define CLK_PIN  21
#define DT_PIN   22

#define MAX_CM      24        // Maximum water level
#define TANK_RADIUS 10.75     // Tank radius in cm

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

/* Message types (must match head) */
enum MsgType : uint8_t {
  MSG_DISCOVERY       = 0,
  MSG_POLL_SENSOR     = 1,
  MSG_SENSOR_DATA     = 2,
  MSG_POLL_HMI        = 3,
  MSG_HMI_SETPOINT    = 4,
  MSG_CONTROL_COMMAND = 5,
  MSG_CONTROL_STATUS  = 6
};

/* Shared ESP-NOW data struct */
typedef struct {
  uint8_t  msg_type;   // MsgType
  uint8_t  rsvd[3];
  uint32_t count;
  uint32_t priority;
  int32_t  data;
  int32_t  data2;
  int32_t  data3;
  bool     ready;
} __attribute__((packed)) esp_now_data_t;

/* Global Variables */

volatile int setpoint = 0;          // cm, from encoder
int setpointPercent   = 0;          // %
uint32_t waterLevel   = 0;          // cm, from head
int waterLevelPercent = 0;          // %
int volume            = 0;          // mL
int servoPosition     = 0;          // degrees, from head
int pumpPower         = 0;          // %, from head

// Track last displayed values
int lastSetpoint         = -1;
int lastSetpointPercent  = -1;
int lastWaterLevel       = -1;
int lastWaterLevelPercent = -1;
int lastVolume           = -1;
int lastServoPosition    = -1;
int lastPumpPower        = -1;

// Rotary Encoder
volatile int counter      = 0;
volatile int lastEncoded  = 0;

uint32_t self_priority      = PRIORITY_HMI;
uint8_t  current_peer_count = 0;
bool     device_is_master   = false;
bool     master_decided     = false;
uint32_t sent_msg_count     = 0;
uint32_t recv_msg_count     = 0;
esp_now_data_t new_msg;
std::vector<uint32_t> last_data(5, 0);

/* Classes */
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

  ~ESP_NOW_Network_Peer() {}

  bool begin() {
    if (!add()) {
      log_e("Failed to initialize ESP-NOW or register the peer");
      return false;
    }
    return true;
  }

  bool send_message(const uint8_t *data, size_t len) {
    if (data == nullptr || len == 0) {
      log_e("Data to be sent is NULL or has a length of 0");
      return false;
    }
    return send(data, len);
  }

  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    esp_now_data_t *msg = (esp_now_data_t *)data;

    if (!peer_ready && msg->ready) {
      Serial.printf("Peer " MACSTR " reported ready\n", MAC2STR(addr()));
      peer_ready = true;
    }

    if (broadcast) {
      return;
    }

    recv_msg_count++;

    // For HMI, the only peer is expected to be the head node (master)
    if (peer_is_master) {
      if (msg->msg_type == MSG_CONTROL_STATUS) {
        // Head sends status: data = water level, data2 = pump power, data3 = servo angle
        waterLevel    = (uint32_t)msg->data;
        pumpPower     = (int)msg->data2;
        servoPosition = (int)msg->data3;

        Serial.printf("Received CONTROL_STATUS from master " MACSTR "\n", MAC2STR(addr()));
        Serial.printf("  Level: %lu cm, Pump: %d %%, Servo: %d deg\n",
                      (unsigned long)waterLevel, pumpPower, servoPosition);

        last_data.push_back(waterLevel);
        last_data.erase(last_data.begin());
      } else if (msg->msg_type == MSG_POLL_HMI) {
        // Head is polling HMI for current setpoint
        esp_now_data_t reply;
        memset(&reply, 0, sizeof(reply));
        reply.msg_type = MSG_HMI_SETPOINT;
        reply.priority = self_priority;
        reply.count    = ++sent_msg_count;
        reply.data     = (int32_t)setpoint;
        reply.ready    = true;

        Serial.printf("Polled by master %s, sending setpoint %d cm\n",
                      WiFi.macAddress().c_str(), setpoint);

        if (!send_message((const uint8_t *)&reply, sizeof(reply))) {
          Serial.println("Failed to send HMI_SETPOINT to master");
        }
      } else {
        Serial.printf("Received unexpected msg_type %d from master\n", msg->msg_type);
      }
    } else if (device_is_master) {
      Serial.println("Received a message as master (unexpected for HMI)");
    } else {
      Serial.printf("Peer " MACSTR " says: data=%ld\n", MAC2STR(addr()), (long)msg->data);
    }
  }

  void onSent(bool success) {
    bool broadcast = memcmp(addr(), ESP_NOW.BROADCAST_ADDR, ESP_NOW_ETH_ALEN) == 0;
    if (broadcast) {
      log_i("Broadcast message reported as sent %s", success ? "successfully" : "unsuccessfully");
    } else {
      log_i("Unicast message reported as sent %s to peer " MACSTR,
            success ? "successfully" : "unsuccessfully", MAC2STR(addr()));
    }
  }
};

/* Peers */
std::vector<ESP_NOW_Network_Peer *> peers;
ESP_NOW_Network_Peer  broadcast_peer(ESP_NOW.BROADCAST_ADDR, 0, nullptr);
ESP_NOW_Network_Peer *master_peer = nullptr;

/* Helper functions */
void IRAM_ATTR updateEncoder() {
  int MSB = digitalRead(CLK_PIN);
  int LSB = digitalRead(DT_PIN);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) counter++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) counter--;
  lastEncoded = encoded;
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
    if (peer->priority > highest_priority) {
      highest_priority = peer->priority;
    }
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
  float volume_ml = 3.14159f * TANK_RADIUS * TANK_RADIUS * level_cm;
  return (int)volume_ml;
}

void drawGUI() {
  tft.fillScreen(ST77XX_BLACK);
  tft.drawRect(0, 20, 240, 300, ST77XX_WHITE);

  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE);

  tft.setCursor(10, 30);
  tft.println("SETPOINT");
  tft.drawLine(5, 95, 235, 95, ST77XX_WHITE);

  tft.setCursor(10, 105);
  tft.println("WATER LEVEL");
  tft.drawLine(5, 200, 235, 200, ST77XX_WHITE);

  tft.setCursor(10, 210);
  tft.println("ACTUATORS");
  tft.drawLine(5, 295, 235, 295, ST77XX_WHITE);
}

void updateSetpointDisplay() {
  int newSetpoint = counter;
  if (newSetpoint < 0) newSetpoint = 0;
  if (newSetpoint > MAX_CM) newSetpoint = MAX_CM;

  if (newSetpoint != lastSetpoint) {
    setpoint = newSetpoint;
    setpointPercent = (setpoint * 100) / MAX_CM;

    tft.fillRect(20, 55, 200, 30, ST77XX_BLACK);
    tft.setTextSize(4);
    tft.setTextColor(ST77XX_CYAN);
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

void updateWaterLevelDisplay() {
  if ((int)waterLevel != lastWaterLevel) {
    waterLevelPercent = (waterLevel * 100) / MAX_CM;
    volume = calculateVolume((int)waterLevel);

    tft.fillRect(20, 130, 200, 30, ST77XX_BLACK);
    tft.setTextSize(4);
    tft.setTextColor(ST77XX_GREEN);
    tft.setCursor(20, 130);
    tft.print(waterLevel);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.print(" cm");

    tft.setTextSize(3);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(140, 135);
    tft.print(waterLevelPercent);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.print("%");

    tft.fillRect(70, 168, 150, 25, ST77XX_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(20, 170);
    tft.print("Vol:");
    tft.setTextSize(3);
    tft.setTextColor(ST77XX_MAGENTA);
    tft.setCursor(70, 168);
    tft.print(volume);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.print(" mL");

    lastWaterLevel = (int)waterLevel;
  }
}

void updateActuatorsDisplay() {
  if (servoPosition != lastServoPosition) {
    tft.fillRect(10, 260, 105, 25, ST77XX_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(10, 240);
    tft.print("Servo:");
    tft.setTextSize(3);
    tft.setTextColor(ST77XX_ORANGE);
    tft.setCursor(10, 260);
    tft.print(servoPosition);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.print(" deg");
    lastServoPosition = servoPosition;
  }

  if (pumpPower != lastPumpPower) {
    tft.fillRect(130, 260, 90, 25, ST77XX_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(130, 240);
    tft.print("Pump:");
    tft.setTextSize(3);
    tft.setTextColor(ST77XX_RED);
    tft.setCursor(130, 260);
    tft.print(pumpPower);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.print("%");
    lastPumpPower = pumpPower;
  }
}

void updateDisplay() {
  updateSetpointDisplay();
  updateWaterLevelDisplay();
  updateActuatorsDisplay();
}

/* Callbacks */
void register_new_peer(const esp_now_recv_info_t *info,
                       const uint8_t *data,
                       int len,
                       void *arg) {
  esp_now_data_t *msg = (esp_now_data_t *)data;
  int priority        = msg->priority;

  if ((uint32_t)priority == self_priority) {
    Serial.println("ERROR! Device has the same priority as this device. Unsupported behavior.");
    fail_reboot();
  }

  if (current_peer_count < ESPNOW_PEER_COUNT) {
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
      Serial.println("Peer identified as master");
    }

    if (current_peer_count == ESPNOW_PEER_COUNT) {
      Serial.println("All peers found");
      new_msg.ready = true;
    }
  }
}

void setup() {
  pinMode(BL_PIN, OUTPUT);
  digitalWrite(BL_PIN, HIGH);

  Serial.begin(115200);
  delay(100);
  Serial.println("=== Water Tank HMI (polled) ===");

  // Initialize display
  tft.init(240, 320);
  tft.setRotation(0);
  drawGUI();
  Serial.println("Display initialized");

  // Setup rotary encoder
  pinMode(CLK_PIN, INPUT_PULLUP);
  pinMode(DT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CLK_PIN), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DT_PIN), updateEncoder, CHANGE);
  Serial.println("Rotary encoder initialized");

  // ESP-NOW setup
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) delay(10);

  self_priority = PRIORITY_HMI;  // HMI has lowest priority

  Serial.println("  WiFi initialized");
  Serial.printf("  MAC: %s\n", WiFi.macAddress().c_str());
  Serial.printf("  Channel: %d\n", ESPNOW_WIFI_CHANNEL);
  Serial.printf("  Priority: %lu (HMI)\n", self_priority);

  tft.setCursor(5, 5);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_GREEN);
  tft.print(WiFi.macAddress());
  tft.print(" | Ch:");
  tft.print(ESPNOW_WIFI_CHANNEL);
  tft.drawLine(0, 15, 240, 15, ST77XX_WHITE);

  if (!ESP_NOW.begin((const uint8_t *)ESPNOW_EXAMPLE_PMK)) {
    Serial.println("Failed to initialize ESP-NOW");
    fail_reboot();
  }
  Serial.printf("ESP-NOW version: %d, max data length: %d\n",
                ESP_NOW.getVersion(), ESP_NOW.getMaxDataLen());

  if (!broadcast_peer.begin()) {
    Serial.println("Failed to initialize broadcast peer");
    fail_reboot();
  }

  ESP_NOW.onNewPeer(register_new_peer, nullptr);
  Serial.println("ESP-NOW initialized");

  memset(&new_msg, 0, sizeof(new_msg));
  new_msg.msg_type = MSG_DISCOVERY;
  new_msg.priority = self_priority;
  new_msg.ready    = false;

  Serial.println("=== Setup Complete ===\n");
}

void loop() {
  static unsigned long lastCommTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastCommTime >= ESPNOW_COMM_INTERVAL_MS) {
    lastCommTime = currentTime;

    if (!master_decided) {
      broadcast_peer.send_message((const uint8_t *)&new_msg, sizeof(new_msg));

      if (current_peer_count == ESPNOW_PEER_COUNT && check_all_peers_ready()) {
        master_decided = true;
        uint32_t highest_priority = check_highest_priority();
        device_is_master = (highest_priority == self_priority);

        if (device_is_master) {
          Serial.println(">>> This device is MASTER (unexpected for HMI)");
        } else {
          Serial.println(">>> This device is SLAVE (HMI)");
          for (auto &peer : peers) {
            if (peer->priority == highest_priority) {
              peer->peer_is_master = true;
              master_peer = peer;
              break;
            }
          }
        }
      }
    } else {
      // After discovery, HMI does not push setpoint anymore.
      // It only sends MSG_HMI_SETPOINT when polled in onReceive.
    }
  }

  updateDisplay();
  delay(50);
}
