/*
  ESP-NOW Actuator Node with HS-422 Servo, polled control
  - Slave node with priority 2
  - Receives MSG_CONTROL_COMMAND from master
    data  = error_cm
    data2 = pump_power_percent
    data3 = servo_angle_deg
  - Moves servo to commanded angle
*/

#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>
#include <vector>
#include <new>

#include <ESP32Servo.h>

/* ESP-NOW Definitions */
#define ESPNOW_WIFI_IFACE       WIFI_IF_STA
#define ESPNOW_WIFI_CHANNEL     4
#define ESPNOW_SEND_INTERVAL_MS 300
#define ESPNOW_PEER_COUNT       1

#define ESPNOW_EXAMPLE_PMK "pmk1234567890123"
#define ESPNOW_EXAMPLE_LMK "lmk1234567890123"

#define PRIORITY_ACTUATOR 2
#define PRIORITY_HEAD     5

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

/* Shared data struct */
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

/* Servo configuration */
#define SERVO_PIN    23
#define SERVO_MIN_PW 900
#define SERVO_MAX_PW 2100

Servo hs422;
int current_angle = 90;  // track last commanded angle

/* Global ESP-NOW state */
uint32_t self_priority      = PRIORITY_ACTUATOR;
uint8_t  current_peer_count = 0;
bool     device_is_master   = false;
bool     master_decided     = false;
uint32_t sent_msg_count     = 0;
esp_now_data_t new_msg;

/* ESP-NOW Peer Class */
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
    if (data == nullptr || len == 0) {
      return false;
    }
    return send(data, len);
  }

  void move_servo_to_angle(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    current_angle = angle;
    hs422.write(current_angle);
    Serial.printf("Servo moved to %d deg\n", current_angle);
  }

  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    esp_now_data_t *msg = (esp_now_data_t *)data;

    if (!peer_ready && msg->ready) {
      Serial.printf("Peer " MACSTR " ready\n", MAC2STR(addr()));
      peer_ready = true;
    }

    if (broadcast) {
      return;
    }

    // This node is actuator, it expects messages from master
    if (!device_is_master && peer_is_master) {
      if (msg->msg_type == MSG_CONTROL_COMMAND) {
        int32_t error_cm   = msg->data;
        int32_t pump_power = msg->data2;
        int32_t servo_deg  = msg->data3;

        Serial.printf("Received CONTROL_COMMAND from master " MACSTR "\n", MAC2STR(addr()));
        Serial.printf("  Error: %ld cm, Pump: %ld %%, Servo: %ld deg\n",
                      (long)error_cm, (long)pump_power, (long)servo_deg);

        move_servo_to_angle((int)servo_deg);
      } else {
        Serial.printf("Received unexpected msg_type %d from master\n", msg->msg_type);
      }
    }
  }

  void onSent(bool success) {
    log_i("Message sent %s", success ? "successfully" : "failed");
  }
};

/* Peers */
std::vector<ESP_NOW_Network_Peer *> peers;
ESP_NOW_Network_Peer  broadcast_peer(ESP_NOW.BROADCAST_ADDR, 0, nullptr);
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

/* New peer callback */
void register_new_peer(const esp_now_recv_info_t *info,
                       const uint8_t *data,
                       int len,
                       void *arg) {
  esp_now_data_t *msg = (esp_now_data_t *)data;
  int priority        = msg->priority;

  if ((uint32_t)priority == self_priority) {
    Serial.println("ERROR! Duplicate priority detected.");
    fail_reboot();
  }

  if (current_peer_count < ESPNOW_PEER_COUNT) {
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
      Serial.println("Peer identified as master");
    }

    if (current_peer_count == ESPNOW_PEER_COUNT) {
      Serial.println("All peers found");
      new_msg.ready = true;
    }
  }
}

/* Setup */
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=== ESP-NOW Actuator Node (servo) ===");

  // Initialize servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  hs422.setPeriodHertz(50);
  hs422.attach(SERVO_PIN, SERVO_MIN_PW, SERVO_MAX_PW);

  hs422.write(current_angle);
  delay(500);
  Serial.println("Servo initialized and moved to 90 deg");

  // ESP-NOW setup
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }

  Serial.println("Wi-Fi initialized");
  Serial.println("MAC: " + WiFi.macAddress());
  Serial.printf("Channel: %d\n", ESPNOW_WIFI_CHANNEL);

  self_priority = PRIORITY_ACTUATOR;
  Serial.printf("Priority: %lu (Actuator)\n", self_priority);

  if (!ESP_NOW.begin((const uint8_t *)ESPNOW_EXAMPLE_PMK)) {
    Serial.println("Failed to initialize ESP-NOW");
    fail_reboot();
  }

  if (!broadcast_peer.begin()) {
    Serial.println("Failed to initialize broadcast peer");
    fail_reboot();
  }

  ESP_NOW.onNewPeer(register_new_peer, nullptr);

  memset(&new_msg, 0, sizeof(new_msg));
  new_msg.msg_type = MSG_DISCOVERY;
  new_msg.priority = self_priority;
  new_msg.ready    = false;

  Serial.println("Setup complete. Searching for master...\n");
}

/* Loop */
void loop() {
  if (!master_decided) {
    // Broadcast priority until master is found
    broadcast_peer.send_message((const uint8_t *)&new_msg, sizeof(new_msg));

    if (current_peer_count == ESPNOW_PEER_COUNT && check_all_peers_ready()) {
      master_decided = true;
      uint32_t highest_priority = check_highest_priority();

      if (highest_priority == self_priority) {
        device_is_master = true;
        Serial.println(">>> I AM THE MASTER (unexpected for actuator) <<<");
      } else {
        for (int i = 0; i < (int)peers.size(); i++) {
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
    // Actuator node: just receives in onReceive and moves servo
  }

  delay(ESPNOW_SEND_INTERVAL_MS);
}
