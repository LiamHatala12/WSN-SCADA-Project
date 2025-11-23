/*
  ESP-NOW Head Node
  - Master with PID control for pump
  - Servo is user disturbance: HMI sets servo angle, Head forwards to Servo node
*/

#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>
#include <vector>
#include <new>
#include <algorithm>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/* ESP-NOW config */
#define ESPNOW_WIFI_IFACE       WIFI_IF_STA
#define ESPNOW_WIFI_CHANNEL     4
#define ESPNOW_PEER_COUNT       4      // HMI, Sensor, Servo, Pump
#define ESPNOW_DISC_INTERVAL_MS 200
#define CONTROL_LOOP_PERIOD_MS  300

#define ESPNOW_EXAMPLE_PMK "pmk1234567890123"
#define ESPNOW_EXAMPLE_LMK "lmk1234567890123"

/* Role priorities */
#define PRIORITY_HMI    0
#define PRIORITY_SENSOR 1
#define PRIORITY_SERVO  2
#define PRIORITY_PUMP   3
#define PRIORITY_HEAD   5

/* Message types and struct (shared across all nodes) */
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

/* Control state */
volatile int32_t sensorData          = 0;   // cm
volatile int32_t lastError           = 0;   // cm
volatile int32_t current_setpoint_cm = 12;  // default
volatile int32_t servo_setpoint_deg  = 45;  // disturbance set by HMI

// PID gains scaled by 10
volatile int32_t Kp_x10 = 20;   // 2.0
volatile int32_t Ki_x10 = 5;    // 0.5
volatile int32_t Kd_x10 = 10;   // 1.0

uint32_t self_priority      = PRIORITY_HEAD;
uint8_t  current_peer_count = 0;
bool     device_is_master   = false;
bool     master_decided     = false;
uint32_t sent_msg_count     = 0;
uint32_t recv_msg_count     = 0;
esp_now_data_t new_msg;

std::vector<int32_t> last_data(5, 0);

/* FreeRTOS objects */
QueueHandle_t sensorQueue        = nullptr;
TaskHandle_t  controlTaskHandle  = nullptr;
TaskHandle_t  pollingTaskHandle  = nullptr;

/* Peer class */
class ESP_NOW_Network_Peer : public ESP_NOW_Peer {
public:
  enum PeerRole {
    PEER_ROLE_UNKNOWN = 0,
    PEER_ROLE_HMI,
    PEER_ROLE_SENSOR,
    PEER_ROLE_SERVO,
    PEER_ROLE_PUMP
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

  bool begin() {
    if (!add()) {
      log_e("Failed to add ESP-NOW peer");
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
      Serial.printf("Peer " MACSTR " reported ready\n", MAC2STR(addr()));
      peer_ready = true;
    }

    if (broadcast) return;

    recv_msg_count++;

    if (!device_is_master) return;

    switch (msg->msg_type) {
      case MSG_SENSOR_DATA:
        if (role == PEER_ROLE_SENSOR) {
          {
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
        }
        break;

      case MSG_HMI_SETPOINT:
        if (role == PEER_ROLE_HMI) {
          int32_t sp   = msg->data;
          int32_t kp10 = msg->data2;
          int32_t ki10 = msg->data3;
          int32_t kd10 = msg->data4;

          current_setpoint_cm = sp;
          Kp_x10 = kp10;
          Ki_x10 = ki10;
          Kd_x10 = kd10;

          Serial.printf("HMI_SETPOINT from HMI " MACSTR "\n", MAC2STR(addr()));
          Serial.printf("  SP = %ld cm, Kp_x10 = %ld, Ki_x10 = %ld, Kd_x10 = %ld\n",
                        (long)sp, (long)kp10, (long)ki10, (long)kd10);
        }
        break;

      case MSG_SERVO_SETPOINT:
        if (role == PEER_ROLE_HMI) {
          int32_t servo_deg = msg->data;
          servo_setpoint_deg = servo_deg;
          Serial.printf("SERVO_SETPOINT from HMI " MACSTR ": %ld deg\n",
                        MAC2STR(addr()), (long)servo_deg);
        }
        break;

      default:
        break;
    }
  }

  void onSent(bool success) {
    log_i("Message sent %s to " MACSTR,
          success ? "successfully" : "failed",
          MAC2STR(addr()));
  }
};

/* Peers */
std::vector<ESP_NOW_Network_Peer *> peers;
ESP_NOW_Network_Peer  broadcast_peer(ESP_NOW.BROADCAST_ADDR, 0, nullptr);
ESP_NOW_Network_Peer *master_peer   = nullptr;  // unused on head
ESP_NOW_Network_Peer *hmi_peer      = nullptr;
ESP_NOW_Network_Peer *sensor_peer   = nullptr;
ESP_NOW_Network_Peer *servo_peer    = nullptr;
ESP_NOW_Network_Peer *pump_peer     = nullptr;

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
    if (!peer->peer_ready) return false;
  }
  return true;
}

/* POLLING TASK: ask HMI and Sensor for data */
void pollingTask(void *pvParameters) {
  const TickType_t loopDelay = pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS);

  for (;;) {
    if (!master_decided || !device_is_master) {
      vTaskDelay(loopDelay);
      continue;
    }

    // Poll HMI (setpoint, gains, servo disturbance)
    if (hmi_peer != nullptr) {
      esp_now_data_t poll_hmi;
      memset(&poll_hmi, 0, sizeof(poll_hmi));
      poll_hmi.msg_type = MSG_POLL_HMI;
      poll_hmi.priority = self_priority;
      poll_hmi.count    = ++sent_msg_count;

      if (!hmi_peer->send_message((const uint8_t *)&poll_hmi, sizeof(poll_hmi))) {
        Serial.println("[POLL] Failed to poll HMI");
      } else {
        Serial.println("[POLL] Polled HMI");
      }
    }

    // Poll Sensor for distance
    if (sensor_peer != nullptr) {
      esp_now_data_t poll_sensor;
      memset(&poll_sensor, 0, sizeof(poll_sensor));
      poll_sensor.msg_type = MSG_POLL_SENSOR;
      poll_sensor.priority = self_priority;
      poll_sensor.count    = ++sent_msg_count;

      if (!sensor_peer->send_message((const uint8_t *)&poll_sensor, sizeof(poll_sensor))) {
        Serial.println("[POLL] Failed to poll Sensor");
      } else {
        Serial.println("[POLL] Polled Sensor");
      }
    }

    vTaskDelay(loopDelay);
  }
}

/* CONTROL TASK: wait for sensor reading, run PID on pump, forward servo setpoint */
void controlTask(void *pvParameters) {
  const float dt = CONTROL_LOOP_PERIOD_MS / 1000.0f;

  static float integral     = 0.0f;
  static float prev_error_f = 0.0f;

  int32_t distance_cm = 0;

  for (;;) {
    if (!master_decided || !device_is_master) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    if (xQueueReceive(sensorQueue, &distance_cm, portMAX_DELAY) == pdTRUE) {
      sensorData = distance_cm;

      int32_t sp_cm  = current_setpoint_cm;
      int32_t kp10   = Kp_x10;
      int32_t ki10   = Ki_x10;
      int32_t kd10   = Kd_x10;
      int32_t servo_deg_set = servo_setpoint_deg;

      int32_t error_cm = sp_cm - distance_cm;
      lastError        = error_cm;

      float e  = (float)error_cm;
      float Kp = kp10 / 10.0f;
      float Ki = ki10 / 10.0f;
      float Kd = kd10 / 10.0f;

      integral += e * dt;
      float derivative = (e - prev_error_f) / dt;
      float u          = Kp * e + Ki * integral + Kd * derivative;
      prev_error_f     = e;

      // Clamp u to [-100, 100]
      if (u > 100.0f)  u = 100.0f;
      if (u < -100.0f) u = -100.0f;

      int32_t pumpPower = (int32_t)fabs(u);    // 0 to 100

      Serial.printf("[PID] SP: %ld cm, Meas: %ld cm, Err: %ld cm\n",
                    (long)sp_cm, (long)distance_cm, (long)error_cm);
      Serial.printf("[PID] Kp: %.2f, Ki: %.2f, Kd: %.2f, u: %.2f\n",
                    Kp, Ki, Kd, u);
      Serial.printf("[PID] Pump: %ld %%, Servo disturbance: %ld deg\n",
                    (long)pumpPower, (long)servo_deg_set);

      // Send command to Pump actuator
      if (pump_peer != nullptr) {
        esp_now_data_t cmd;
        memset(&cmd, 0, sizeof(cmd));
        cmd.msg_type = MSG_CONTROL_COMMAND;
        cmd.priority = self_priority;
        cmd.count    = ++sent_msg_count;
        cmd.data     = error_cm;     // for logging
        cmd.data2    = pumpPower;    // main control output
        cmd.data3    = 0;
        cmd.data4    = 0;
        cmd.ready    = true;

        if (!pump_peer->send_message((const uint8_t *)&cmd, sizeof(cmd))) {
          Serial.println("[CONTROL] Failed to send command to Pump");
        } else {
          Serial.println("[CONTROL] Command sent to Pump");
        }
      }

      // Send command to Servo actuator (disturbance)
      if (servo_peer != nullptr) {
        esp_now_data_t scmd;
        memset(&scmd, 0, sizeof(scmd));
        scmd.msg_type = MSG_SERVO_COMMAND;
        scmd.priority = self_priority;
        scmd.count    = ++sent_msg_count;
        scmd.data     = servo_deg_set;
        scmd.ready    = true;

        if (!servo_peer->send_message((const uint8_t *)&scmd, sizeof(scmd))) {
          Serial.println("[CONTROL] Failed to send servo command");
        } else {
          Serial.println("[CONTROL] Servo command sent");
        }
      }

      // Status to HMI (for display)
      if (hmi_peer != nullptr) {
        esp_now_data_t status;
        memset(&status, 0, sizeof(status));
        status.msg_type = MSG_CONTROL_STATUS;
        status.priority = self_priority;
        status.count    = ++sent_msg_count;
        status.data     = distance_cm;
        status.data2    = pumpPower;
        status.data3    = servo_deg_set;  // optional
        status.data4    = 0;
        status.ready    = true;

        if (!hmi_peer->send_message((const uint8_t *)&status, sizeof(status))) {
          Serial.println("[CONTROL] Failed to send status to HMI");
        } else {
          Serial.println("[CONTROL] Status sent to HMI");
        }
      }
    }
  }
}

/* New peer callback */
void register_new_peer(const esp_now_recv_info_t *info,
                       const uint8_t *data,
                       int len,
                       void *arg) {
  esp_now_data_t *msg = (esp_now_data_t *)data;
  int priority        = (int)msg->priority;

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
    Serial.println("Failed to create or register peer");
    delete new_peer;
    return;
  }

  if ((uint32_t)priority < self_priority) {
    if (priority == PRIORITY_SENSOR) {
      new_peer->role = ESP_NOW_Network_Peer::PEER_ROLE_SENSOR;
      sensor_peer    = new_peer;
      Serial.println("Peer assigned role: SENSOR");
    } else if (priority == PRIORITY_HMI) {
      new_peer->role = ESP_NOW_Network_Peer::PEER_ROLE_HMI;
      hmi_peer       = new_peer;
      Serial.println("Peer assigned role: HMI");
    } else if (priority == PRIORITY_SERVO) {
      new_peer->role = ESP_NOW_Network_Peer::PEER_ROLE_SERVO;
      servo_peer     = new_peer;
      Serial.println("Peer assigned role: SERVO");
    } else if (priority == PRIORITY_PUMP) {
      new_peer->role = ESP_NOW_Network_Peer::PEER_ROLE_PUMP;
      pump_peer      = new_peer;
      Serial.println("Peer assigned role: PUMP");
    } else {
      new_peer->role = ESP_NOW_Network_Peer::PEER_ROLE_UNKNOWN;
    }
  }

  peers.push_back(new_peer);
  current_peer_count++;

  if (current_peer_count == ESPNOW_PEER_COUNT) {
    Serial.println("All peers found");
    new_msg.ready = true;
  }
}

/* Setup */
void setup() {
  Serial.begin(115200);
  delay(1000);

  uint8_t self_mac[6];

  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }

  Serial.println("\n=== ESP-NOW HEAD NODE (PID + servo disturbance) ===");
  Serial.println("WiFi in STA mode");
  Serial.println("MAC: " + WiFi.macAddress());
  Serial.printf("Channel: %d\n", ESPNOW_WIFI_CHANNEL);

  WiFi.macAddress(self_mac);
  self_priority = PRIORITY_HEAD;

  Serial.printf("Priority: %lu\n", self_priority);
  Serial.printf("Initial SP: %ld cm, Kp_x10: %ld, Ki_x10: %ld, Kd_x10: %ld, Servo: %ld deg\n",
                (long)current_setpoint_cm, (long)Kp_x10, (long)Ki_x10, (long)Kd_x10, (long)servo_setpoint_deg);

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

  sensorQueue = xQueueCreate(5, sizeof(int32_t));
  if (sensorQueue == nullptr) {
    Serial.println("Failed to create sensorQueue");
    fail_reboot();
  }

  BaseType_t ok1 = xTaskCreatePinnedToCore(
    controlTask,
    "ControlTask",
    4096,
    nullptr,
    1,
    &controlTaskHandle,
    1
  );

  BaseType_t ok2 = xTaskCreatePinnedToCore(
    pollingTask,
    "PollingTask",
    4096,
    nullptr,
    1,
    &pollingTaskHandle,
    1
  );

  if (ok1 != pdPASS || ok2 != pdPASS) {
    Serial.println("Failed to create tasks");
    fail_reboot();
  }

  Serial.println("Setup complete. Discovering peers...");
}

/* Loop: discovery and master decision only */
void loop() {
  if (!master_decided) {
    if (!broadcast_peer.send_message((const uint8_t *)&new_msg, sizeof(new_msg))) {
      Serial.println("Failed to broadcast discovery");
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
          for (auto &peer : peers) {
            if (peer->priority == highest_priority) {
              peer->peer_is_master = true;
              master_peer = peer;
              Serial.printf("Peer " MACSTR " is the master with priority %lu\n",
                            MAC2STR(peer->addr()), highest_priority);
              break;
            }
          }
        }
        Serial.println("Master decided");
      } else {
        Serial.println("Waiting for all peers to be ready...");
      }
    }
  }

  delay(ESPNOW_DISC_INTERVAL_MS);
}
