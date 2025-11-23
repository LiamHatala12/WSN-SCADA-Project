/*
  ESP-NOW Sensor Node
  - Priority 1
  - Responds to MSG_POLL_SENSOR with MSG_SENSOR_DATA
*/

#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>
#include <vector>
#include <new>

/* ESP-NOW config */
#define ESPNOW_WIFI_IFACE       WIFI_IF_STA
#define ESPNOW_WIFI_CHANNEL     4
#define ESPNOW_SEND_INTERVAL_MS 300
#define ESPNOW_PEER_COUNT       1

#define ESPNOW_EXAMPLE_PMK "pmk1234567890123"
#define ESPNOW_EXAMPLE_LMK "lmk1234567890123"

/* Role priorities */
#define PRIORITY_SENSOR 1
#define PRIORITY_HEAD   5

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

/* Ultrasonic sensor */
const int pingPin = 23;

long readUltrasonicCm() {
  long duration;
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH, 30000);
  if (duration == 0) return 0;
  return duration / 29 / 2;
}

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

    if (msg->msg_type == MSG_POLL_SENSOR) {
      Serial.println("[SENSOR] Received poll request from HEAD");
      
      long dist = readUltrasonicCm();
      Serial.printf("[SENSOR] Ultrasonic reading: %ld cm (distance to water surface)\n", dist);
      
      esp_now_data_t reply;
      memset(&reply, 0, sizeof(reply));
      reply.msg_type = MSG_SENSOR_DATA;
      reply.priority = PRIORITY_SENSOR;
      reply.count    = msg->count;
      reply.data     = (int32_t)dist;
      reply.ready    = true;

      if (!send_message((const uint8_t *)&reply, sizeof(reply))) {
        Serial.println("[SENSOR] Failed to send SENSOR_DATA");
      } else {
        Serial.printf("[SENSOR] Sent SENSOR_DATA: %ld cm to HEAD\n", dist);
      }
    }
  }

  void onSent(bool success) {
    log_i("Message sent %s", success ? "successfully" : "failed");
  }
};

std::vector<ESP_NOW_Network_Peer *> peers;
ESP_NOW_Network_Peer broadcast_peer(ESP_NOW.BROADCAST_ADDR, 0, nullptr);
ESP_NOW_Network_Peer *master_peer = nullptr;

uint32_t self_priority      = PRIORITY_SENSOR;
uint8_t  current_peer_count = 0;
bool     device_is_master   = false;
bool     master_decided     = false;
uint32_t sent_msg_count     = 0;
esp_now_data_t new_msg;

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
  int priority = (int)msg->priority;

  if ((uint32_t)priority == self_priority) {
    Serial.println("ERROR: duplicate priority");
    fail_reboot();
  }

  if (current_peer_count >= ESPNOW_PEER_COUNT) return;

  Serial.printf("New peer: " MACSTR " (priority %d)\n",
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
  uint8_t self_mac[6];
  Serial.begin(115200);
  delay(500);

  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }

  WiFi.macAddress(self_mac);
  self_priority = PRIORITY_SENSOR;

  Serial.println("\n=== ESP-NOW Sensor Node ===");
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

  Serial.println("Sensor setup complete, discovering HEAD...");
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
        Serial.println(">>> Sensor became master (unexpected)");
      } else {
        Serial.println(">>> Sensor is slave, HEAD is master");
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

  delay(ESPNOW_SEND_INTERVAL_MS);
}
