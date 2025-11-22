/*
  ESP-NOW Master Node with FreeRTOS
  - Receives distance in cm from ultrasonic sensor node (priority 1)
  - Receives user setpoint from HMI node (priority 0)
  - Computes error = setpoint_cm - measured_cm
  - Sends error to actuator node (priority 2)
  - Sends status (water level, pump power, servo position) to HMI node
  - Uses FreeRTOS queue and task to handle sensor data and control logic
*/

#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>
#include <vector>
#include <new>
#include <algorithm>

// FreeRTOS includes (already part of ESP32 Arduino core)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/* Definitions */
#define ESPNOW_WIFI_IFACE       WIFI_IF_STA
#define ESPNOW_WIFI_CHANNEL     4
// sensor (1) + actuator (2) + HMI (0)
#define ESPNOW_PEER_COUNT       3
#define ESPNOW_SEND_INTERVAL_MS 5000
#define REPORT_INTERVAL         5

#define ESPNOW_EXAMPLE_PMK "pmk1234567890123"
#define ESPNOW_EXAMPLE_LMK "lmk1234567890123"

// Default user setpoint in cm
static const int32_t DEFAULT_SETPOINT_CM = 60;

/* Data struct - must match on head and HMI
   Semantics depend on peer role and direction:
   - Sensor -> Master: data  = distance_cm
   - HMI -> Master   : data  = setpoint_cm
   - Master -> Act   : data  = error_cm
   - Master -> HMI   : data  = water_level_cm
                       data2 = pump_power_percent
                       data3 = servo_position_deg
*/
typedef struct {
  uint32_t count;
  uint32_t priority;
  int32_t  data;
  int32_t  data2;
  int32_t  data3;
  bool     ready;
} __attribute__((packed)) esp_now_data_t;

/* Global Variables */
// Latest sensor value (cm) for debugging
volatile int32_t sensorData          = 0;
volatile int32_t lastError           = 0;
volatile int32_t current_setpoint_cm = DEFAULT_SETPOINT_CM;

uint32_t self_priority      = 0;
uint8_t current_peer_count  = 0;
bool device_is_master       = false;
bool master_decided         = false;
uint32_t sent_msg_count     = 0;
uint32_t recv_msg_count     = 0;
esp_now_data_t new_msg;

// Keep a small window of last readings
std::vector<int32_t> last_data(5, 0);

/* FreeRTOS objects */
QueueHandle_t sensorQueue       = nullptr;     // carries distance readings from callback to control task
TaskHandle_t controlTaskHandle  = nullptr;

/* Helper functions to derive fake pump and servo info from error */
static const int32_t MAX_ERROR_FOR_PUMP = 100;  // clamp for pump and servo calculations

int32_t compute_pump_power_percent(int32_t error_cm) {
  int32_t e = abs(error_cm);
  if (e > MAX_ERROR_FOR_PUMP) {
    e = MAX_ERROR_FOR_PUMP;
  }
  long p = map((long)e, 0L, (long)MAX_ERROR_FOR_PUMP, 0L, 100L);
  if (p < 0) p = 0;
  if (p > 100) p = 100;
  return (int32_t)p;
}

int32_t compute_servo_position_deg(int32_t error_cm) {
  int32_t e = error_cm;
  if (e > MAX_ERROR_FOR_PUMP)  e = MAX_ERROR_FOR_PUMP;
  if (e < -MAX_ERROR_FOR_PUMP) e = -MAX_ERROR_FOR_PUMP;
  long angle = map((long)e,
                   (long)-MAX_ERROR_FOR_PUMP,
                   (long) MAX_ERROR_FOR_PUMP,
                   0L,
                   180L);
  if (angle < 0)   angle = 0;
  if (angle > 180) angle = 180;
  return (int32_t)angle;
}

/* Classes */
class ESP_NOW_Network_Peer : public ESP_NOW_Peer {
public:
  enum PeerRole {
    PEER_ROLE_UNKNOWN = 0,
    PEER_ROLE_SENSOR,
    PEER_ROLE_ACTUATOR,
    PEER_ROLE_HMI
  };

  uint32_t priority;
  bool peer_is_master = false;
  bool peer_ready     = false;
  PeerRole role       = PEER_ROLE_UNKNOWN;

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

    if (!broadcast) {
      recv_msg_count++;

      // Sensor -> Master: distance in cm
      if (device_is_master && role == PEER_ROLE_SENSOR) {
        int32_t distance_cm = msg->data;

        sensorData = distance_cm;

        last_data.push_back(distance_cm);
        last_data.erase(last_data.begin());

        Serial.printf("Received sensor data from " MACSTR "\n", MAC2STR(addr()));
        Serial.printf("  Count: %lu\n", msg->count);
        Serial.printf("  Distance: %ld cm\n", (long)distance_cm);

        if (sensorQueue != nullptr) {
          xQueueSend(sensorQueue, &distance_cm, 0);
        }
      }
      // HMI -> Master: user defined setpoint in cm
      else if (device_is_master && role == PEER_ROLE_HMI) {
        int32_t new_setpoint = msg->data;
        current_setpoint_cm = new_setpoint;

        Serial.printf("Received setpoint update from HMI " MACSTR "\n", MAC2STR(addr()));
        Serial.printf("  New setpoint: %ld cm\n", (long)new_setpoint);
      }
      // Slave receiving from master (this code is master, kept for completeness)
      else if (!device_is_master && peer_is_master) {
        Serial.println("Received a message from the master");
        Serial.printf("  Data: %ld\n", (long)msg->data);
      } else {
        Serial.printf("Peer " MACSTR " says: data=%ld\n", MAC2STR(addr()), (long)msg->data);
      }
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
ESP_NOW_Network_Peer broadcast_peer(ESP_NOW.BROADCAST_ADDR, 0, nullptr);
ESP_NOW_Network_Peer *master_peer   = nullptr;   // not used on master, kept for compatibility
ESP_NOW_Network_Peer *sensor_peer   = nullptr;
ESP_NOW_Network_Peer *actuator_peer = nullptr;
ESP_NOW_Network_Peer *hmi_peer      = nullptr;

/* Helper functions */
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

int32_t calc_average() {
  int32_t sum = 0;
  for (auto &d : last_data) {
    sum += d;
  }
  return sum / (int32_t)last_data.size();
}

bool check_all_peers_ready() {
  for (auto &peer : peers) {
    if (!peer->peer_ready) {
      return false;
    }
  }
  return true;
}

/* FreeRTOS control task
   - Waits for new distance values from sensorQueue
   - Uses current_setpoint_cm (updated by HMI)
   - Computes error = setpoint - distance
   - Sends error to actuator peer
   - Sends status (water level, pump power, servo position) to HMI peer
*/
void controlTask(void *pvParameters) {
  int32_t distance_cm = 0;
  uint32_t local_count = 0;

  esp_now_data_t out_msg;
  esp_now_data_t status_msg;

  memset(&out_msg, 0, sizeof(out_msg));
  memset(&status_msg, 0, sizeof(status_msg));

  out_msg.priority    = 0;
  status_msg.priority = 0;

  for (;;) {
    if (xQueueReceive(sensorQueue, &distance_cm, portMAX_DELAY) == pdTRUE) {
      int32_t setpoint = current_setpoint_cm;
      int32_t error    = setpoint - distance_cm;
      lastError        = error;

      int32_t pump_power = compute_pump_power_percent(error);
      int32_t servo_deg  = compute_servo_position_deg(error);

      local_count++;

      Serial.printf("[CONTROL] Setpoint: %ld cm, Measured: %ld cm, Error: %ld cm\n",
                    (long)setpoint, (long)distance_cm, (long)error);
      Serial.printf("[CONTROL] Pump power: %ld %%, Servo: %ld deg\n",
                    (long)pump_power, (long)servo_deg);

      // Send error to actuator
      if (device_is_master && actuator_peer != nullptr) {
        out_msg.count = local_count;
        out_msg.data  = error;
        out_msg.data2 = 0;
        out_msg.data3 = 0;
        out_msg.ready = true;

        if (!actuator_peer->send_message((const uint8_t *)&out_msg, sizeof(out_msg))) {
          Serial.println("[CONTROL] Failed to send error to actuator");
        } else {
          Serial.println("[CONTROL] Error sent to actuator");
        }
      } else {
        Serial.println("[CONTROL] Actuator peer not ready yet");
      }

      // Send status to HMI: water level, pump power, servo position
      if (device_is_master && hmi_peer != nullptr) {
        status_msg.count = local_count;
        status_msg.data  = distance_cm;  // water level
        status_msg.data2 = pump_power;   // fake pump power percent
        status_msg.data3 = servo_deg;    // servo position in degrees
        status_msg.ready = true;

        if (!hmi_peer->send_message((const uint8_t *)&status_msg, sizeof(status_msg))) {
          Serial.println("[CONTROL] Failed to send status to HMI");
        } else {
          Serial.println("[CONTROL] Status sent to HMI");
        }
      }
    }
  }
}

/* Callbacks */
void register_new_peer(const esp_now_recv_info_t *info, const uint8_t *data, int len, void *arg) {
  esp_now_data_t *msg = (esp_now_data_t *)data;
  int priority = (int)msg->priority;

  if ((uint32_t)priority == self_priority) {
    Serial.println("ERROR! Device has the same priority as this device. Unsupported behavior.");
    fail_reboot();
  }

  if (current_peer_count < ESPNOW_PEER_COUNT) {
    Serial.printf("New peer found: " MACSTR " with priority %d\n", MAC2STR(info->src_addr), priority);
    ESP_NOW_Network_Peer *new_peer = new (std::nothrow) ESP_NOW_Network_Peer(info->src_addr, priority);
    if (new_peer == nullptr || !new_peer->begin()) {
      Serial.println("Failed to create or register the new peer");
      delete new_peer;
      return;
    }

    // Assign roles based on priority (all less than master)
    if ((uint32_t)priority < self_priority) {
      if (priority == 1) {
        new_peer->role = ESP_NOW_Network_Peer::PEER_ROLE_SENSOR;
        sensor_peer = new_peer;
        Serial.println("Peer assigned role: SENSOR");
      } else if (priority == 2) {
        new_peer->role = ESP_NOW_Network_Peer::PEER_ROLE_ACTUATOR;
        actuator_peer = new_peer;
        Serial.println("Peer assigned role: ACTUATOR");
      } else if (priority == 0) {
        new_peer->role = ESP_NOW_Network_Peer::PEER_ROLE_HMI;
        hmi_peer = new_peer;
        Serial.println("Peer assigned role: HMI");
      } else {
        new_peer->role = ESP_NOW_Network_Peer::PEER_ROLE_UNKNOWN;
      }
    }

    peers.push_back(new_peer);
    current_peer_count++;

    if (current_peer_count == ESPNOW_PEER_COUNT) {
      Serial.println("All peers have been found");
      new_msg.ready = true;
    }
  }
}

/* Setup */
void setup() {
  Serial.begin(115200);
  delay(1000);

  uint8_t self_mac[6];

  // ESP-NOW setup
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }

  Serial.println("\n=== ESP-NOW HEAD NODE (Master) ===");
  Serial.println("Wi-Fi parameters:");
  Serial.println("  Mode: STA");
  Serial.println("  MAC Address: " + WiFi.macAddress());
  Serial.printf("  Channel: %d\n", ESPNOW_WIFI_CHANNEL);

  WiFi.macAddress(self_mac);
  self_priority = 5;  // Master priority
  current_setpoint_cm = DEFAULT_SETPOINT_CM;

  Serial.printf("This device's priority: %lu\n", self_priority);
  Serial.printf("Initial control setpoint: %ld cm\n", (long)current_setpoint_cm);

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

  // Initialize message used for discovery
  memset(&new_msg, 0, sizeof(new_msg));
  new_msg.priority = self_priority;
  new_msg.ready    = false;

  // Create FreeRTOS queue and task
  sensorQueue = xQueueCreate(5, sizeof(int32_t));
  if (sensorQueue == nullptr) {
    Serial.println("Failed to create sensorQueue");
    fail_reboot();
  }

  BaseType_t created = xTaskCreatePinnedToCore(
    controlTask,
    "ControlTask",
    4096,
    nullptr,
    1,
    &controlTaskHandle,
    1  // run on core 1
  );

  if (created != pdPASS) {
    Serial.println("Failed to create control task");
    fail_reboot();
  }

  Serial.println("Setup complete. Broadcasting own priority to find peers...");
  delay(100);
}

/* Main Loop */
void loop() {
  if (!master_decided) {
    if (!broadcast_peer.send_message((const uint8_t *)&new_msg, sizeof(new_msg))) {
      Serial.println("Failed to broadcast message");
    }

    if (current_peer_count == ESPNOW_PEER_COUNT) {
      if (check_all_peers_ready()) {
        Serial.println("All peers are ready");
        master_decided = true;
        uint32_t highest_priority = check_highest_priority();
        if (highest_priority == self_priority) {
          device_is_master = true;
          Serial.println("This device is the master");
        } else {
          for (int i = 0; i < ESPNOW_PEER_COUNT; i++) {
            if (peers[i]->priority == highest_priority) {
              peers[i]->peer_is_master = true;
              master_peer = peers[i];
              Serial.printf("Peer " MACSTR " is the master with priority %lu\n",
                            MAC2STR(peers[i]->addr()), highest_priority);
              break;
            }
          }
        }
        Serial.println("The master has been decided");
      } else {
        Serial.println("Waiting for all peers to be ready...");
      }
    }
  }
  delay(50);
}
