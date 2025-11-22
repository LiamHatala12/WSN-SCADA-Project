/*
  ESP-NOW Actuator Node with HS-422 Servo
  - Slave node (priority 2)
  - Receives error (cm) from master via ESP-NOW
  - Maps error to servo angle 0–180 degrees
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
#define ESPNOW_PEER_COUNT       1  // One master

// Security keys (must match on all devices)
#define ESPNOW_EXAMPLE_PMK "pmk1234567890123"
#define ESPNOW_EXAMPLE_LMK "lmk1234567890123"

/* Data Structure - align with master */
typedef struct {
  uint32_t count;
  uint32_t priority;
  int32_t  data;
  int32_t  data2;
  int32_t  data3;
  bool     ready;
} __attribute__((packed)) esp_now_data_t;

/* Global Variables */
uint32_t self_priority = 0;
uint8_t current_peer_count = 0;
bool device_is_master = false;
bool master_decided = false;
uint32_t sent_msg_count = 0;   // not really used here
esp_now_data_t new_msg;

/* Servo: HS-422 Hitec */
#define SERVO_PIN    23
#define SERVO_MIN_PW 900
#define SERVO_MAX_PW 2100

Servo hs422;
int current_angle = 90;  // track last commanded angle

// Error mapping parameters
static const int32_t MAX_ERROR_CM      = 100; // clamp error magnitude
static const int32_t ERROR_DEADBAND_CM = 1;   // small deadband

/* Map control error (cm) to servo angle 0–180
   error > 0  -> measured < setpoint -> open valve (toward 180)
   error < 0  -> measured > setpoint -> close valve (toward 0)
*/
int mapErrorToAngle(int32_t error_cm) {
  if (abs(error_cm) <= ERROR_DEADBAND_CM) {
    // within deadband, hold current angle
    return current_angle;
  }

  if (error_cm > MAX_ERROR_CM) {
    error_cm = MAX_ERROR_CM;
  } else if (error_cm < -MAX_ERROR_CM) {
    error_cm = -MAX_ERROR_CM;
  }

  long angle = map((long)error_cm,
                   (long)-MAX_ERROR_CM,
                   (long) MAX_ERROR_CM,
                   0L,
                   180L);

  if (angle < 0)   angle = 0;
  if (angle > 180) angle = 180;

  return (int)angle;
}

/* ESP-NOW Peer Class */
class ESP_NOW_Network_Peer : public ESP_NOW_Peer {
public:
  uint32_t priority;
  bool peer_is_master = false;
  bool peer_ready     = false;

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

  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    esp_now_data_t *msg = (esp_now_data_t *)data;

    if (!peer_ready && msg->ready) {
      Serial.printf("Peer " MACSTR " ready\n", MAC2STR(addr()));
      peer_ready = true;
    }

    if (!broadcast && peer_is_master) {
      int32_t error_cm = msg->data;

      Serial.printf("Received from master " MACSTR "\n", MAC2STR(addr()));
      Serial.printf("  Count: %lu, Error: %ld cm\n", msg->count, (long)error_cm);

      int target_angle = mapErrorToAngle(error_cm);
      if (target_angle != current_angle) {
        current_angle = target_angle;
        hs422.write(current_angle);
        Serial.printf("  -> Servo angle set to %d degrees\n", current_angle);
      } else {
        Serial.printf("  -> Error within deadband, holding at %d degrees\n", current_angle);
      }
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
void register_new_peer(const esp_now_recv_info_t *info,
                       const uint8_t *data,
                       int len,
                       void *arg) {
  esp_now_data_t *msg = (esp_now_data_t *)data;
  int priority = msg->priority;

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

    // Higher priority peer is the master in this setup
    if ((uint32_t)priority > self_priority) {
      new_peer->peer_is_master = true;
      master_peer = new_peer;
      Serial.println("Peer identified as master");
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
  delay(1000);

  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║       ESP-NOW HS-422 Actuator Node        ║");
  Serial.println("╚════════════════════════════════════════════╝");

  // Servo init (same style as your test code, but no long demo sequence)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  hs422.setPeriodHertz(50);
  hs422.attach(SERVO_PIN, SERVO_MIN_PW, SERVO_MAX_PW);
  hs422.write(current_angle);

  Serial.printf("Servo PIN   : %d\n", SERVO_PIN);
  Serial.printf("Pulse Width : %d - %d µs\n", SERVO_MIN_PW, SERVO_MAX_PW);
  Serial.println("Role        : Actuator (servo)");
  Serial.println("==============================================");

  // Wi-Fi / ESP-NOW init
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }

  Serial.println("\n=== ESP-NOW Actuator Network ===");
  Serial.println("MAC: " + WiFi.macAddress());
  Serial.printf("Channel: %d\n", ESPNOW_WIFI_CHANNEL);

  WiFi.macAddress(self_mac);
  self_priority = 2;  // Actuator priority (sensor=1, master>2)
  Serial.printf("Priority: %lu (Actuator)\n", self_priority);

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
  new_msg.ready    = false;

  Serial.println("Setup complete. Searching for master...\n");
}

/* Main Loop */
void loop() {
  if (!master_decided) {
    // Broadcast priority to find master
    broadcast_peer.send_message((const uint8_t *)&new_msg, sizeof(new_msg));

    if (current_peer_count == ESPNOW_PEER_COUNT && check_all_peers_ready()) {
      Serial.println("All peers ready");
      master_decided = true;
      uint32_t highest_priority = check_highest_priority();

      if (highest_priority == self_priority) {
        device_is_master = true;
        Serial.println(">>> I AM THE MASTER (unexpected for actuator) <<<\n");
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
    // Actuator node: just receives in onReceive and moves servo
  }

  delay(ESPNOW_SEND_INTERVAL_MS);
}