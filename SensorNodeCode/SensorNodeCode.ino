/*
    ESP-NOW Network with Ultrasonic Sensor
    Simplified version - Slave sends sensor data to Master
*/

#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>
#include <vector>
#include <new>

/* Definitions */
#define ESPNOW_WIFI_IFACE WIFI_IF_STA
#define ESPNOW_WIFI_CHANNEL 4
#define ESPNOW_SEND_INTERVAL_MS 300  // Send every 1 second
#define ESPNOW_PEER_COUNT 1           // Number of peers (1 master + 1 slave = set to 1)

// Security keys (must match on all devices)
#define ESPNOW_EXAMPLE_PMK "pmk1234567890123"
#define ESPNOW_EXAMPLE_LMK "lmk1234567890123"

/* Data Structure */
typedef struct {
  uint32_t count;
  uint32_t priority;
  uint32_t data;
  bool ready;
} __attribute__((packed)) esp_now_data_t;

/* Global Variables */
uint32_t self_priority = 0;
uint8_t current_peer_count = 0;
bool device_is_master = false;
bool master_decided = false;
uint32_t sent_msg_count = 0;
esp_now_data_t new_msg;

/* Ultrasonic Sensor */
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
  duration = pulseIn(pingPin, HIGH);
  return duration / 29 / 2;
}

/* ESP-NOW Peer Class */
class ESP_NOW_Network_Peer : public ESP_NOW_Peer {
public:
  uint32_t priority;
  bool peer_is_master = false;
  bool peer_ready = false;

  ESP_NOW_Network_Peer(const uint8_t *mac_addr, uint32_t priority = 0, const uint8_t *lmk = (const uint8_t *)ESPNOW_EXAMPLE_LMK)
    : ESP_NOW_Peer(mac_addr, ESPNOW_WIFI_CHANNEL, ESPNOW_WIFI_IFACE, lmk), priority(priority) {}

  bool begin() {
    if (!add()) {
      log_e("Failed to register peer");
      return false;
    }
    return true;
  }

  bool send_message(const uint8_t *data, size_t len) {
    if (data == nullptr || len == 0) {
      return false;
    }
    return send(data, len);
  }

  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    esp_now_data_t *msg = (esp_now_data_t *)data;

    if (peer_ready == false && msg->ready == true) {
      Serial.printf("Peer " MACSTR " ready\n", MAC2STR(addr()));
      peer_ready = true;
    }

    if (!broadcast && device_is_master) {
      Serial.printf("Received from " MACSTR "\n", MAC2STR(addr()));
      Serial.printf("  Count: %lu, Distance: %lu cm\n", msg->count, msg->data);
    }
  }

  void onSent(bool success) {
    log_i("Message sent %s", success ? "successfully" : "failed");
  }
};

/* Peer Management */
std::vector<ESP_NOW_Network_Peer *> peers;
ESP_NOW_Network_Peer broadcast_peer(ESP_NOW.BROADCAST_ADDR, 0, nullptr);
ESP_NOW_Network_Peer *master_peer = nullptr;

/* Helper Functions */
void fail_reboot() {
  Serial.println("Rebooting in 5 seconds...");
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
    if (!peer->peer_ready) {
      return false;
    }
  }
  return true;
}

/* Callback for New Peer Discovery */
void register_new_peer(const esp_now_recv_info_t *info, const uint8_t *data, int len, void *arg) {
  esp_now_data_t *msg = (esp_now_data_t *)data;
  int priority = msg->priority;

  if (priority == self_priority) {
    Serial.println("ERROR! Duplicate priority detected.");
    fail_reboot();
  }

  if (current_peer_count < ESPNOW_PEER_COUNT) {
    Serial.printf("New peer: " MACSTR " (priority %d)\n", MAC2STR(info->src_addr), priority);
    ESP_NOW_Network_Peer *new_peer = new (std::nothrow) ESP_NOW_Network_Peer(info->src_addr, priority);
    if (new_peer == nullptr || !new_peer->begin()) {
      Serial.println("Failed to register peer");
      delete new_peer;
      return;
    }
    peers.push_back(new_peer);
    current_peer_count++;
    
    if (current_peer_count == ESPNOW_PEER_COUNT) {
      Serial.println("All peers found");
      new_msg.ready = true;
    }
  }
}

/* Setup */
void setup() {
  uint8_t self_mac[6];
  Serial.begin(115200);

  // Initialize Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }

  Serial.println("\n=== ESP-NOW Sensor Network ===");
  Serial.println("MAC: " + WiFi.macAddress());
  Serial.printf("Channel: %d\n", ESPNOW_WIFI_CHANNEL);

  // Set priority (1 = slave, higher = master)
  WiFi.macAddress(self_mac);
  self_priority = 1;  // Change to higher number for master
  Serial.printf("Priority: %lu\n", self_priority);

  // Initialize ESP-NOW
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
  new_msg.priority = self_priority;
  Serial.println("Setup complete. Searching for peers...\n");
}

/* Main Loop */
void loop() {
  if (!master_decided) {
    // Broadcast priority to find peers
    broadcast_peer.send_message((const uint8_t *)&new_msg, sizeof(new_msg));

    if (current_peer_count == ESPNOW_PEER_COUNT && check_all_peers_ready()) {
      Serial.println("All peers ready");
      master_decided = true;
      uint32_t highest_priority = check_highest_priority();
      
      if (highest_priority == self_priority) {
        device_is_master = true;
        Serial.println(">>> I AM THE MASTER <<<\n");
      } else {
        for (int i = 0; i < ESPNOW_PEER_COUNT; i++) {
          if (peers[i]->priority == highest_priority) {
            peers[i]->peer_is_master = true;
            master_peer = peers[i];
            Serial.printf(">>> Master is " MACSTR " <<<\n\n", MAC2STR(peers[i]->addr()));
            break;
          }
        }
      }
    }
  } else {
    // Slave: Send sensor data to master
    if (!device_is_master && master_peer != nullptr) {
      new_msg.count = sent_msg_count + 1;
      new_msg.data = readUltrasonicCm();
      
      if (master_peer->send_message((const uint8_t *)&new_msg, sizeof(new_msg))) {
        Serial.printf("Sent to master: Count=%lu, Distance=%lu cm\n", new_msg.count, new_msg.data);
        sent_msg_count++;
      } else {
        Serial.println("Send failed");
      }
    }
    // Master: Just receive (handled by onReceive callback)
  }
  delay(ESPNOW_SEND_INTERVAL_MS);
}
