/*
  ESP-NOW Master Node with FreeRTOS + HMI + Polling + PID


  Roles and priorities:
    HMI          priority 0
    Sensor       priority 1
    Servo node   priority 2
    Pump node    priority 3
    Head         priority 5


  Protocol overview after discovery:
    1) Head polls HMI for setpoint and PID / servo params:
       - Head -> HMI: MSG_POLL_HMI
       - HMI -> Head: MSG_HMI_SETPOINT (SP + Kp,Ki,Kd)
       - HMI -> Head: MSG_SERVO_SETPOINT (servo disturbance angle)


    2) Head polls Sensor for distance:
       - Head -> Sensor: MSG_POLL_SENSOR
       - Sensor -> Head: MSG_SENSOR_DATA (data = distance_cm)


       Head converts:
         water_level_cm = MAX_WATER_LEVEL_CM - distance_cm


    3) Head runs PID on water_level_cm:
         error = setpoint_cm - water_level_cm


       Then:
       - Head -> Pump node:  MSG_CONTROL_COMMAND
         (data = control effort, data2 = pump_power)
       - Head -> Servo node: MSG_SERVO_COMMAND
         (data = servo_disturbance_deg)
       - Head -> HMI:        MSG_CONTROL_STATUS
         (data = raw distance_cm, data2 = pump_power, data3 = servo_deg)


  Sensor, HMI, Servo, Pump do not send data unless polled or commanded, which avoids collisions.
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
#define ESPNOW_PEER_COUNT       4      // HMI, Sensor, Servo, Pump
#define ESPNOW_DISC_INTERVAL_MS 200
#define CONTROL_LOOP_PERIOD_MS  300    // control loop rate (ms)
#define POLL_LOOP_PERIOD_MS     300    // polling rate (ms)
#define MAX_WATER_LEVEL_CM      24     // maximum tank water level in cm


#define ESPNOW_EXAMPLE_PMK "pmk1234567890123"
#define ESPNOW_EXAMPLE_LMK "lmk1234567890123"


/* Role priorities */
#define PRIORITY_HMI       0
#define PRIORITY_SENSOR    1
#define PRIORITY_SERVO     2
#define PRIORITY_PUMP      3
#define PRIORITY_HEAD      5


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



/* PID state */
volatile int32_t current_setpoint_cm = 12;   // default setpoint
volatile int32_t current_Kp_x10      = 20;   // Kp = 2.0
volatile int32_t current_Ki_x10      = 5;    // Ki = 0.5
volatile int32_t current_Kd_x10      = 10;   // Kd = 1.0
volatile int32_t current_servo_setpoint_deg = 0; // disturbance from HMI


static float lastErrorF        = 0.0f;
static float integralTermF     = 0.0f;
static uint32_t lastPidTime_ms = 0;


/* Global control variables */
volatile int32_t sensorData    = 0;   // we store water level here
volatile int32_t lastError     = 0;   // cm


uint32_t self_priority      = PRIORITY_HEAD;
uint8_t  current_peer_count = 0;
bool     device_is_master   = false;
bool     master_decided     = false;
uint32_t sent_msg_count     = 0;
uint32_t recv_msg_count     = 0;
esp_now_data_t new_msg;    // used during discovery


std::vector<int32_t> last_data(5, 0);


/* FreeRTOS objects */
QueueHandle_t sensorQueue       = nullptr; // carries raw distance to control task
TaskHandle_t  controlTaskHandle = nullptr;
TaskHandle_t  pollTaskHandle    = nullptr;


/* Helper for pump from PID output */
static const float  MAX_OUTPUT      = 100.0f;
static const float  MIN_OUTPUT      = -100.0f;
static const float  DT_SEC_DEFAULT  = 0.3f;
static const float  MAX_WINDUP      = 200.0f;


/* Hardcoded MAC addresses for each node */
static uint8_t HMI_MAC[]    = {0x1c, 0x69, 0x20, 0x92, 0x60, 0x30};
static uint8_t SENSOR_MAC[] = {0xb0, 0xa7, 0x32, 0x2b, 0x6c, 0xd8};
static uint8_t SERVO_MAC[]  = {0xb0, 0xa7, 0x32, 0x2b, 0x0f, 0x20};
static uint8_t PUMP_MAC[]   = {0x24, 0xdc, 0xc3, 0x45, 0xf3, 0x50};


int32_t compute_pump_power_percent_from_output(float u) {
  float mag = fabsf(u);
  if (mag > 100.0f) mag = 100.0f;
  return (int32_t)mag;
}


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


    // ALWAYS check for ready flag, even during discovery
    if (!peer_ready && msg->ready) {
      Serial.printf("Peer " MACSTR " reported ready\n", MAC2STR(addr()));
      peer_ready = true;
    }


    // Ignore broadcasts (discovery messages)
    if (broadcast) return;


    recv_msg_count++;


    // Only process operational messages once we are master
    if (!device_is_master) return;


    switch (msg->msg_type) {
      case MSG_SENSOR_DATA:
        if (role == PEER_ROLE_SENSOR) {
          int32_t distance_cm = msg->data;


          Serial.printf("Received SENSOR_DATA from " MACSTR "\n", MAC2STR(addr()));
          Serial.printf("  Count: %lu\n", msg->count);
          Serial.printf("  Distance (raw): %ld cm\n", (long)distance_cm);


          if (sensorQueue != nullptr) {
            xQueueSend(sensorQueue, &distance_cm, 0);
          }
        }
        break;


      case MSG_HMI_SETPOINT:
        if (role == PEER_ROLE_HMI) {
          int32_t sp      = msg->data;
          int32_t kp_x10  = msg->data2;
          int32_t ki_x10  = msg->data3;
          int32_t kd_x10  = msg->data4;


          current_setpoint_cm = sp;
          current_Kp_x10      = kp_x10;
          current_Ki_x10      = ki_x10;
          current_Kd_x10      = kd_x10;


          Serial.printf("HMI_SETPOINT from HMI " MACSTR "\n", MAC2STR(addr()));
          Serial.printf("  SP = %ld cm, Kp_x10 = %ld, Ki_x10 = %ld, Kd_x10 = %ld\n",
                        (long)sp, (long)kp_x10, (long)ki_x10, (long)kd_x10);
        }
        break;


      case MSG_SERVO_SETPOINT:
        if (role == PEER_ROLE_HMI) {
          int32_t servoSet = msg->data;
          current_servo_setpoint_deg = servoSet;
          Serial.printf("SERVO_SETPOINT from HMI " MACSTR ": %ld deg\n",
                        MAC2STR(addr()), (long)servoSet);
        }
        break;


      default:
        break;
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
ESP_NOW_Network_Peer *master_peer    = nullptr;
ESP_NOW_Network_Peer *hmi_peer       = nullptr;
ESP_NOW_Network_Peer *sensor_peer    = nullptr;
ESP_NOW_Network_Peer *servo_peer     = nullptr;
ESP_NOW_Network_Peer *pump_peer      = nullptr;


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


/* Polling task */
void pollTask(void *pvParameters) {
  const TickType_t loopDelay = pdMS_TO_TICKS(POLL_LOOP_PERIOD_MS);


  for (;;) {
    if (!master_decided || !device_is_master) {
      vTaskDelay(loopDelay);
      continue;
    }


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
        Serial.println("[POLL] Polled HMI");
      }
    }


    if (sensor_peer != nullptr) {
      esp_now_data_t poll_sensor;
      memset(&poll_sensor, 0, sizeof(poll_sensor));
      poll_sensor.msg_type = MSG_POLL_SENSOR;
      poll_sensor.priority = self_priority;
      poll_sensor.count    = ++sent_msg_count;
      poll_sensor.ready    = false;


      if (!sensor_peer->send_message((const uint8_t *)&poll_sensor, sizeof(poll_sensor))) {
        Serial.println("[POLL] Failed to poll Sensor");
      } else {
        Serial.println("[POLL] Polled Sensor");
      }
    }


    vTaskDelay(loopDelay);
  }
}


/* Control task with PID on water level */
void controlTask(void *pvParameters) {
  for (;;) {
    if (!master_decided || !device_is_master) {
      vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS));
      continue;
    }


    int32_t distance_cm = 0;


    if (xQueueReceive(sensorQueue, &distance_cm, portMAX_DELAY) != pdTRUE) {
      continue;
    }


    // Convert raw distance to water level
    int32_t water_level_cm = MAX_WATER_LEVEL_CM - distance_cm;
    if (water_level_cm < 0) water_level_cm = 0;
    if (water_level_cm > MAX_WATER_LEVEL_CM) water_level_cm = MAX_WATER_LEVEL_CM;


    sensorData = water_level_cm;


    float setpointF = (float)current_setpoint_cm;
    float pvF       = (float)water_level_cm;


    float Kp = (float)current_Kp_x10 / 10.0f;
    float Ki = (float)current_Ki_x10 / 10.0f;
    float Kd = (float)current_Kd_x10 / 10.0f;


    uint32_t now_ms = millis();
    float dt = (lastPidTime_ms == 0) ? DT_SEC_DEFAULT : ((now_ms - lastPidTime_ms) / 1000.0f);
    if (dt <= 0.0f) dt = DT_SEC_DEFAULT;
    lastPidTime_ms = now_ms;


    float error = setpointF - pvF;
    integralTermF += error * dt;
    if (integralTermF > MAX_WINDUP)  integralTermF = MAX_WINDUP;
    if (integralTermF < -MAX_WINDUP) integralTermF = -MAX_WINDUP;


    float derivative = (error - lastErrorF) / dt;


    float output = Kp * error + Ki * integralTermF + Kd * derivative;
    if (output > MAX_OUTPUT) output = MAX_OUTPUT;
    if (output < MIN_OUTPUT) output = MIN_OUTPUT;


    int32_t pump_power = compute_pump_power_percent_from_output(output);
    int32_t servo_deg  = current_servo_setpoint_deg;


    lastErrorF = error;
    lastError  = (int32_t)error;


    Serial.printf("[PID] SP: %.2f cm, Water level: %.2f cm, Error: %.2f cm\n",
                  setpointF, pvF, error);
    Serial.printf("[PID] Kp: %.2f, Ki: %.2f, Kd: %.2f, u: %.2f\n",
                  Kp, Ki, Kd, output);
    Serial.printf("[PID] Pump: %ld %%, Servo disturbance: %ld deg\n",
                  (long)pump_power, (long)servo_deg);


    // Pump command
    if (pump_peer != nullptr) {
      esp_now_data_t cmd;
      memset(&cmd, 0, sizeof(cmd));
      cmd.msg_type = MSG_CONTROL_COMMAND;
      cmd.priority = self_priority;
      cmd.count    = ++sent_msg_count;
      cmd.data     = (int32_t)output;
      cmd.data2    = pump_power;
      cmd.ready    = true;


      if (!pump_peer->send_message((const uint8_t *)&cmd, sizeof(cmd))) {
        Serial.println("[CONTROL] Failed to send control command to Pump");
      } else {
        Serial.println("[CONTROL] Command sent to Pump");
      }
    }


    // Servo command
    if (servo_peer != nullptr) {
      esp_now_data_t scmd;
      memset(&scmd, 0, sizeof(scmd));
      scmd.msg_type = MSG_SERVO_COMMAND;
      scmd.priority = self_priority;
      scmd.count    = ++sent_msg_count;
      scmd.data     = current_servo_setpoint_deg;
      scmd.ready    = true;


      if (!servo_peer->send_message((const uint8_t *)&scmd, sizeof(scmd))) {
        Serial.println("[CONTROL] Failed to send servo command");
      } else {
        Serial.println("[CONTROL] Servo command sent");
      }
    }


    // Status to HMI
    if (hmi_peer != nullptr) {
      esp_now_data_t status;
      memset(&status, 0, sizeof(status));
      status.msg_type = MSG_CONTROL_STATUS;
      status.priority = self_priority;
      status.count    = ++sent_msg_count;
      status.data     = water_level_cm;
      status.data2    = pump_power;
      status.data3    = current_servo_setpoint_deg;
      status.ready    = true;


      if (!hmi_peer->send_message((const uint8_t *)&status, sizeof(status))) {
        Serial.println("[CONTROL] Failed to send status to HMI");
      } else {
        Serial.println("[CONTROL] Status sent to HMI");
      }
    }
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

    // Validate MAC address matches expected priority
    bool valid_peer = false;
    ESP_NOW_Network_Peer::PeerRole assigned_role = ESP_NOW_Network_Peer::PEER_ROLE_UNKNOWN;
    
    if (memcmp(info->src_addr, HMI_MAC, 6) == 0 && priority == PRIORITY_HMI) {
      assigned_role = ESP_NOW_Network_Peer::PEER_ROLE_HMI;
      valid_peer = true;
    } else if (memcmp(info->src_addr, SENSOR_MAC, 6) == 0 && priority == PRIORITY_SENSOR) {
      assigned_role = ESP_NOW_Network_Peer::PEER_ROLE_SENSOR;
      valid_peer = true;
    } else if (memcmp(info->src_addr, SERVO_MAC, 6) == 0 && priority == PRIORITY_SERVO) {
      assigned_role = ESP_NOW_Network_Peer::PEER_ROLE_SERVO;
      valid_peer = true;
    } else if (memcmp(info->src_addr, PUMP_MAC, 6) == 0 && priority == PRIORITY_PUMP) {
      assigned_role = ESP_NOW_Network_Peer::PEER_ROLE_PUMP;
      valid_peer = true;
    }

    if (!valid_peer) {
      Serial.println("ERROR! Unknown or mismatched MAC/priority combination. Ignoring peer.");
      return;
    }

    ESP_NOW_Network_Peer *new_peer =
      new (std::nothrow) ESP_NOW_Network_Peer(info->src_addr, priority);

    if (new_peer == nullptr || !new_peer->begin()) {
      Serial.println("Failed to create or register the new peer");
      delete new_peer;
      return;
    }

    new_peer->role = assigned_role;
    
    // Assign to specific peer pointers
    if (assigned_role == ESP_NOW_Network_Peer::PEER_ROLE_HMI) {
      hmi_peer = new_peer;
      Serial.println("Peer assigned role: HMI");
    } else if (assigned_role == ESP_NOW_Network_Peer::PEER_ROLE_SENSOR) {
      sensor_peer = new_peer;
      Serial.println("Peer assigned role: SENSOR");
    } else if (assigned_role == ESP_NOW_Network_Peer::PEER_ROLE_SERVO) {
      servo_peer = new_peer;
      Serial.println("Peer assigned role: SERVO");
    } else if (assigned_role == ESP_NOW_Network_Peer::PEER_ROLE_PUMP) {
      pump_peer = new_peer;
      Serial.println("Peer assigned role: PUMP");
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


  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }


  Serial.println("\n=== ESP-NOW HEAD NODE (Master with polling + PID) ===");
  Serial.println("Wi-Fi parameters:");
  Serial.println("  Mode: STA");
  Serial.println("  MAC Address: " + WiFi.macAddress());
  Serial.printf("  Channel: %d\n", ESPNOW_WIFI_CHANNEL);


  WiFi.macAddress(self_mac);
  self_priority = PRIORITY_HEAD;
  Serial.printf("This device priority: %lu\n", self_priority);
  Serial.printf("Initial setpoint: %ld cm\n", (long)current_setpoint_cm);


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


  memset(&new_msg, 0, sizeof(new_msg));
  new_msg.msg_type = MSG_DISCOVERY;
  new_msg.priority = self_priority;
  new_msg.ready    = false;


  sensorQueue = xQueueCreate(5, sizeof(int32_t));
  if (sensorQueue == nullptr) {
    Serial.println("Failed to create sensorQueue");
    fail_reboot();
  }


  BaseType_t created1 = xTaskCreatePinnedToCore(
    controlTask,
    "ControlTask",
    4096,
    nullptr,
    1,
    &controlTaskHandle,
    1
  );


  if (created1 != pdPASS) {
    Serial.println("Failed to create control task");
    fail_reboot();
  }


  BaseType_t created2 = xTaskCreatePinnedToCore(
    pollTask,
    "PollTask",
    4096,
    nullptr,
    1,
    &pollTaskHandle,
    1
  );


  if (created2 != pdPASS) {
    Serial.println("Failed to create poll task");
    fail_reboot();
  }


  Serial.println("Setup complete. Broadcasting own priority to find peers...");
  delay(100);
}


/* Main Loop: discovery and master decision */
void loop() {
  if (!master_decided) {
    // Continue broadcasting with updated ready state
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
