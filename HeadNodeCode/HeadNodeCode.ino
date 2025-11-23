/*
  ESP-NOW Master Node with FreeRTOS + HMI + Polling

  Roles and priorities:
    HMI      priority 0
    Sensor   priority 1
    Actuator priority 2
    Head     priority 5

  Protocol overview after discovery:
    1) Head polls HMI for setpoint:
       - Head -> HMI: MSG_POLL_HMI
       - HMI -> Head: MSG_HMI_SETPOINT (data = setpoint_cm)

    2) Head polls Sensor for distance:
       - Head -> Sensor: MSG_POLL_SENSOR
       - Sensor -> Head: MSG_SENSOR_DATA (data = distance_cm)

    3) Head computes error = setpoint_cm - distance_cm,
       computes pump power and valve angle, then:
       - Head -> Actuator: MSG_CONTROL_COMMAND
         (data = error_cm, data2 = pump_power, data3 = servo_angle_deg)
       - Head -> HMI: MSG_CONTROL_STATUS
         (data = distance_cm, data2 = pump_power, data3 = servo_angle_deg)

  Sensor and HMI do not send data unless polled, which avoids collisions.

  This version uses TWO FreeRTOS tasks:
    - pollingTask: sends MSG_POLL_HMI and MSG_POLL_SENSOR periodically
    - controlTask: waits on sensorQueue for new distance and computes control
*/

#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>
#include <vector>
#include <new>
#include <algorithm>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/* Definitions */
#define ESPNOW_WIFI_IFACE       WIFI_IF_STA
#define ESPNOW_WIFI_CHANNEL     4
#define ESPNOW_PEER_COUNT       3      // HMI, Sensor, Actuator
#define ESPNOW_DISC_INTERVAL_MS 200
#define CONTROL_LOOP_PERIOD_MS  300    // control loop rate (ms)
#define POLL_LOOP_PERIOD_MS     300    // polling rate (ms) - can be same as control

#define ESPNOW_EXAMPLE_PMK "pmk1234567890123"
#define ESPNOW_EXAMPLE_LMK "lmk1234567890123"

/* Role priorities */
#define PRIORITY_HMI       0
#define PRIORITY_SENSOR    1
#define PRIORITY_ACTUATOR  2
#define PRIORITY_HEAD      5

/* Message types */
enum MsgType : uint8_t {
  MSG_DISCOVERY       = 0,
  MSG_POLL_SENSOR     = 1,
  MSG_SENSOR_DATA     = 2,
  MSG_POLL_HMI        = 3,
  MSG_HMI_SETPOINT    = 4,
  MSG_CONTROL_COMMAND = 5,
  MSG_CONTROL_STATUS  = 6
};

/* Shared data struct on all nodes */
typedef struct {
  uint8_t  msg_type;   // MsgType
  uint8_t  rsvd[3];    // padding to keep 32 bit alignment
  uint32_t count;
  uint32_t priority;
  int32_t  data;
  int32_t  data2;
  int32_t  data3;
  bool     ready;
} __attribute__((packed)) esp_now_data_t;

/* Global control variables */
volatile int32_t sensorData          = 0;   // cm
volatile int32_t lastError           = 0;   // cm
volatile int32_t current_setpoint_cm = 60;  // default setpoint

uint32_t self_priority      = PRIORITY_HEAD;
uint8_t  current_peer_count = 0;
bool     device_is_master   = false;
bool     master_decided     = false;
uint32_t sent_msg_count     = 0;
uint32_t recv_msg_count     = 0;
esp_now_data_t new_msg;    // used during discovery

// Keep a small window of last readings (for debug / averaging)
std::vector<int32_t> last_data(5, 0);

/* FreeRTOS objects */
QueueHandle_t sensorQueue        = nullptr; // carries distance readings to control task
TaskHandle_t  controlTaskHandle  = nullptr;
TaskHandle_t  pollingTaskHandle  = nullptr;

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
  if (angle < 0L)   angle = 0L;
  if (angle > 180L) angle = 180L;
  return (int32_t)angle;
}

/* Classes */
class ESP_NOW_Network_Peer : public ESP_NOW_Peer {
public:
  enum PeerRole {
    PEER_ROLE_UNKNOWN = 0,
    PEER_ROLE_HMI,
    PEER_ROLE_SENSOR,
    PEER_ROLE_ACTUATOR
  };

  uint32_t priority;
  bool     peer_is_master = false;
  bool     peer_ready     = false;
  PeerRole role           = PEER_ROLE_UNKNOWN;

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

    if (device_is_master) {
      // Data messages back to the master
      switch (msg->msg_type) {
        case MSG_SENSOR_DATA:
          if (role == PEER_ROLE_SENSOR) {
            int32_t distance_cm = msg->data;
            sensorData = distance_cm;

            last_data.push_back(distance_cm);
            last_data.erase(last_data.begin());

            Serial.printf("Received SENSOR_DATA from " MACSTR "\n", MAC2STR(addr()));
            Serial.printf("  Count: %lu\n", msg->count);
            Serial.printf("  Distance: %ld cm\n", (long)distance_cm);

            if (sensorQueue != nullptr) {
              xQueueSend(sensorQueue, &distance_cm, 0);
            }
          }
          break;

        case MSG_HMI_SETPOINT:
          if (role == PEER_ROLE_HMI) {
            int32_t new_setpoint = msg->data;
            current_setpoint_cm = new_setpoint;

            Serial.printf("Received HMI_SETPOINT from HMI " MACSTR "\n", MAC2STR(addr()));
            Serial.printf("  New setpoint: %ld cm\n", (long)new_setpoint);
          }
          break;

        default:
          // Other message types to master are not expected here
          break;
      }
    } else if (!device_is_master && peer_is_master) {
      // This code path is not used when this sketch runs as head
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
ESP_NOW_Network_Peer *master_peer   = nullptr;   // not used on master
ESP_NOW_Network_Peer *hmi_peer      = nullptr;
ESP_NOW_Network_Peer *sensor_peer   = nullptr;
ESP_NOW_Network_Peer *actuator_peer = nullptr;

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

bool check_all_peers_ready() {
  for (auto &peer : peers) {
    if (!peer->peer_ready) {
      return false;
    }
  }
  return true;
}

/* FreeRTOS POLLING task */
void pollingTask(void *pvParameters) {
  const TickType_t loopDelay = pdMS_TO_TICKS(POLL_LOOP_PERIOD_MS);

  for (;;) {
    if (!master_decided || !device_is_master) {
      vTaskDelay(loopDelay);
      continue;
    }

    // 1) Poll HMI for setpoint (non-blocking, used asynchronously)
    if (hmi_peer != nullptr) {
      esp_now_data_t poll_hmi;
      memset(&poll_hmi, 0, sizeof(poll_hmi));
      poll_hmi.msg_type = MSG_POLL_HMI;
      poll_hmi.priority = self_priority;
      poll_hmi.count    = ++sent_msg_count;
      poll_hmi.ready    = false;

      if (!hmi_peer->send_message((const uint8_t *)&poll_hmi, sizeof(poll_hmi))) {
        Serial.println("[POLL] Failed to poll HMI");
      } else {
        Serial.println("[POLL] Polled HMI for setpoint");
      }
    }

    // 2) Poll Sensor for distance (reply will be pushed into sensorQueue by onReceive)
    if (sensor_peer != nullptr) {
      esp_now_data_t poll_sensor;
      memset(&poll_sensor, 0, sizeof(poll_sensor));
      poll_sensor.msg_type = MSG_POLL_SENSOR;
      poll_sensor.priority = self_priority;
      poll_sensor.count    = ++sent_msg_count;
      poll_sensor.ready    = false;

      if (!sensor_peer->send_message((const uint8_t *)&poll_sensor, sizeof(poll_sensor))) {
        Serial.println("[POLL] Failed to poll sensor");
      } else {
        Serial.println("[POLL] Polled sensor for distance");
      }
    }

    vTaskDelay(loopDelay);
  }
}

/* FreeRTOS CONTROL task
   - Waits for new distance values from sensorQueue
   - Uses latest current_setpoint_cm (updated by MSG_HMI_SETPOINT)
   - Computes error, pump power, servo angle
   - Sends MSG_CONTROL_COMMAND to actuator and MSG_CONTROL_STATUS to HMI
*/
void controlTask(void *pvParameters) {
  for (;;) {
    if (!master_decided || !device_is_master) {
      vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS));
      continue;
    }

    int32_t distance_cm = 0;

    // Block until we get a new measurement
    if (xQueueReceive(sensorQueue, &distance_cm, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    sensorData = distance_cm;

    int32_t setpoint_cm = current_setpoint_cm;
    int32_t error_cm    = setpoint_cm - distance_cm;
    lastError           = error_cm;

    int32_t pump_power = compute_pump_power_percent(error_cm);
    int32_t servo_deg  = compute_servo_position_deg(error_cm);

    Serial.printf("[CONTROL] Setpoint: %ld cm, Measured: %ld cm, Error: %ld cm\n",
                  (long)setpoint_cm, (long)distance_cm, (long)error_cm);
    Serial.printf("[CONTROL] Pump power: %ld %%, Servo: %ld deg\n",
                  (long)pump_power, (long)servo_deg);

    // 1) Send control command to actuator
    if (actuator_peer != nullptr) {
      esp_now_data_t cmd;
      memset(&cmd, 0, sizeof(cmd));
      cmd.msg_type = MSG_CONTROL_COMMAND;
      cmd.priority = self_priority;
      cmd.count    = ++sent_msg_count;
      cmd.data     = error_cm;
      cmd.data2    = pump_power;
      cmd.data3    = servo_deg;
      cmd.ready    = true;

      if (!actuator_peer->send_message((const uint8_t *)&cmd, sizeof(cmd))) {
        Serial.println("[CONTROL] Failed to send control command to actuator");
      } else {
        Serial.println("[CONTROL] Control command sent to actuator");
      }
    }

    // 2) Send status to HMI
    if (hmi_peer != nullptr) {
      esp_now_data_t status;
      memset(&status, 0, sizeof(status));
      status.msg_type = MSG_CONTROL_STATUS;
      status.priority = self_priority;
      status.count    = ++sent_msg_count;
      status.data     = distance_cm;
      status.data2    = pump_power;
      status.data3    = servo_deg;
      status.ready    = true;

      if (!hmi_peer->send_message((const uint8_t *)&status, sizeof(status))) {
        Serial.println("[CONTROL] Failed to send status to HMI");
      } else {
        Serial.println("[CONTROL] Status sent to HMI");
      }
    }

    // Control loop naturally runs every time we get a new sensor reading
  }
}

/* Callbacks */
void register_new_peer(const esp_now_recv_info_t *info,
                       const uint8_t *data,
                       int len,
                       void *arg) {
  esp_now_data_t *msg = (esp_now_data_t *)data;
  int priority        = (int)msg->priority;

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
      Serial.println("Failed to create or register the new peer");
      delete new_peer;
      return;
    }

    // Assign roles based on priority
    if ((uint32_t)priority < self_priority) {
      if (priority == PRIORITY_SENSOR) {
        new_peer->role = ESP_NOW_Network_Peer::PEER_ROLE_SENSOR;
        sensor_peer    = new_peer;
        Serial.println("Peer assigned role: SENSOR");
      } else if (priority == PRIORITY_ACTUATOR) {
        new_peer->role = ESP_NOW_Network_Peer::PEER_ROLE_ACTUATOR;
        actuator_peer  = new_peer;
        Serial.println("Peer assigned role: ACTUATOR");
      } else if (priority == PRIORITY_HMI) {
        new_peer->role = ESP_NOW_Network_Peer::PEER_ROLE_HMI;
        hmi_peer       = new_peer;
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

  Serial.println("\n=== ESP-NOW HEAD NODE (Master with polling + control tasks) ===");
  Serial.println("Wi-Fi parameters:");
  Serial.println("  Mode: STA");
  Serial.println("  MAC Address: " + WiFi.macAddress());
  Serial.printf("  Channel: %d\n", ESPNOW_WIFI_CHANNEL);

  WiFi.macAddress(self_mac);
  self_priority = PRIORITY_HEAD;
  Serial.printf("This device priority: %lu\n", self_priority);
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
  new_msg.msg_type = MSG_DISCOVERY;
  new_msg.priority = self_priority;
  new_msg.ready    = false;

  // Create FreeRTOS queue
  sensorQueue = xQueueCreate(5, sizeof(int32_t));
  if (sensorQueue == nullptr) {
    Serial.println("Failed to create sensorQueue");
    fail_reboot();
  }

  // Create CONTROL task
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

  // Create POLLING task
  created = xTaskCreatePinnedToCore(
    pollingTask,
    "PollingTask",
    4096,
    nullptr,
    1,
    &pollingTaskHandle,
    1  // run on core 1 (same core is fine here)
  );

  if (created != pdPASS) {
    Serial.println("Failed to create polling task");
    fail_reboot();
  }

  Serial.println("Setup complete. Broadcasting own priority to find peers...");
  delay(100);
}

/* Main Loop: only handles discovery and master decision */
void loop() {
  if (!master_decided) {
    if (!broadcast_peer.send_message((const uint8_t *)&new_msg, sizeof(new_msg))) {
      Serial.println("Failed to broadcast discovery message");
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
          for (int i = 0; i < (int)peers.size(); i++) {
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

  delay(ESPNOW_DISC_INTERVAL_MS);
}
