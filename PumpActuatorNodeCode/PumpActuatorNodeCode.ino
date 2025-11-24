/*
  ESP-NOW Pump Actuator Node
  - Priority 3
  - Receives MSG_CONTROL_COMMAND from HEAD:
      data  = error_cm (for logging)
      data2 = pumpPower_percent (0-100)
  - Drives pump via L298N with PWM
*/

#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>
#include <vector>
#include <new>

/* ESP-NOW config */
#define ESPNOW_WIFI_IFACE       WIFI_IF_STA
#define ESPNOW_WIFI_CHANNEL     4
#define ESPNOW_PEER_COUNT       1

#define ESPNOW_EXAMPLE_PMK "pmk1234567890123"
#define ESPNOW_EXAMPLE_LMK "lmk1234567890123"

/* Role priorities */
#define PRIORITY_PUMP 3
#define PRIORITY_HEAD 5

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

/* Pump config: L298N HW-095 */
const int pumpPwmPin    = 18;  // ENA
const int pumpIn1Pin    = 19;  // IN1
const int pumpIn2Pin    = 21;  // IN2

const int pwmFreq       = 20000;
const int pwmResolution = 8;

/* ESP-NOW globals */
uint32_t self_priority      = PRIORITY_PUMP;
uint8_t  current_peer_count = 0;
bool     device_is_master   = false;
bool     master_decided     = false;
uint32_t sent_msg_count     = 0;
esp_now_data_t new_msg;

/* Hardcoded HEAD MAC address */
static uint8_t HEAD_MAC[] = {0x24, 0xDC, 0xC3, 0x45, 0x88, 0x84};


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
    esp_now_data_t *msg = (esp_now_data_t *)data;

    if (!peer_ready && msg->ready) {
      Serial.printf("Peer " MACSTR " ready\n", MAC2STR(addr()));
      peer_ready = true;
    }

    if (broadcast) return;
    if (!peer_is_master) return;

    if (msg->msg_type == MSG_CONTROL_COMMAND) {
      int32_t error_cm  = msg->data;
      int32_t pumpPower = msg->data2;

      if (pumpPower < 0)   pumpPower = 0;
      if (pumpPower > 100) pumpPower = 100;

      Serial.printf("CONTROL_COMMAND from HEAD " MACSTR ": Err=%ld cm, Pump=%ld %%\n",
                    MAC2STR(addr()), (long)error_cm, (long)pumpPower);

      uint32_t maxDuty = (1u << pwmResolution) - 1;
      uint32_t duty    = map(pumpPower, 0, 100, 0, (int)maxDuty);

      // Forward direction
      digitalWrite(pumpIn1Pin, HIGH);
      digitalWrite(pumpIn2Pin, LOW);
      ledcWrite(pumpPwmPin, duty);
    }
  }

  void onSent(bool success) {
    log_i("Message sent %s", success ? "successfully" : "failed");
  }
};

std::vector<ESP_NOW_Network_Peer *> peers;
ESP_NOW_Network_Peer broadcast_peer(ESP_NOW.BROADCAST_ADDR, 0, nullptr);
ESP_NOW_Network_Peer *master_peer = nullptr;

void fail_reboot() {
  Serial.println("Rebooting in 5 seconds...");
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

/* New peer callback */
void register_new_peer(const esp_now_recv_info_t *info,
                       const uint8_t *data,
                       int len,
                       void *arg) {
  esp_now_data_t *msg = (esp_now_data_t *)data;
  int priority        = (int)msg->priority;

  if ((uint32_t)priority == self_priority) {
    Serial.println("ERROR duplicate priority");
    fail_reboot();
  }

  if (current_peer_count >= ESPNOW_PEER_COUNT) {
    return;
  }

  // Validate this is the HEAD node
  if (memcmp(info->src_addr, HEAD_MAC, 6) != 0) {
    Serial.println("ERROR: Unknown MAC - not the expected HEAD");
    return;
  }
  
  if (priority != PRIORITY_HEAD) {
    Serial.println("ERROR: HEAD has wrong priority");
    return;
  }

  Serial.printf("New peer: " MACSTR " priority %d\n", MAC2STR(info->src_addr), priority);

  ESP_NOW_Network_Peer *new_peer =
    new (std::nothrow) ESP_NOW_Network_Peer(info->src_addr, priority);

  if (new_peer == nullptr || !new_peer->begin()) {
    Serial.println("Failed to register peer");
    delete new_peer;
    return;
  }

  peers.push_back(new_peer);
  current_peer_count++;

  new_peer->peer_is_master = true;
  master_peer = new_peer;
  Serial.println("Peer identified as HEAD (master)");

  if (current_peer_count == ESPNOW_PEER_COUNT) {
    Serial.println("All peers found");
    new_msg.ready = true;
  }
}


/* Setup pump hardware */
void setupPump() {
  pinMode(pumpIn1Pin, OUTPUT);
  pinMode(pumpIn2Pin, OUTPUT);

  digitalWrite(pumpIn1Pin, LOW);
  digitalWrite(pumpIn2Pin, LOW);

  ledcAttach(pumpPwmPin, pwmFreq, pwmResolution);
  ledcWrite(pumpPwmPin, 0);
}

/* Setup */
void setup() {
  uint8_t self_mac[6];
  Serial.begin(115200);
  delay(500);

  Serial.println("\n=== ESP-NOW Pump Actuator Node ===");

  setupPump();

  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }

  WiFi.macAddress(self_mac);
  self_priority = PRIORITY_PUMP;

  Serial.println("WiFi initialized");
  Serial.println("MAC: " + WiFi.macAddress());
  Serial.printf("Channel: %d\n", ESPNOW_WIFI_CHANNEL);
  Serial.printf("Priority: %lu\n", self_priority);

  if (!ESP_NOW.begin((const uint8_t *)ESPNOW_EXAMPLE_PMK)) {
    Serial.println("ESP-NOW init failed");
    fail_reboot();
  }

  if (!broadcast_peer.begin()) {
    Serial.println("Broadcast peer init failed");
    fail_reboot();
  }

  ESP_NOW.onNewPeer(register_new_peer, nullptr);

  memset(&new_msg, 0, sizeof(new_msg));
  new_msg.msg_type = MSG_DISCOVERY;
  new_msg.priority = self_priority;
  new_msg.ready    = false;

  Serial.println("Pump node setup complete, discovering HEAD...");
}

/* Loop */
void loop() {
  if (!master_decided) {
    broadcast_peer.send_message((const uint8_t *)&new_msg, sizeof(new_msg));

    if (current_peer_count == ESPNOW_PEER_COUNT && check_all_peers_ready()) {
      master_decided = true;
      uint32_t highest_priority = check_highest_priority();
      device_is_master = (highest_priority == self_priority);

      if (device_is_master) {
        Serial.println(">>> Pump became master (unexpected)");
      } else {
        Serial.println(">>> Pump is slave, HEAD is master");
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

  delay(200);
}
