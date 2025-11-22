
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>
#include <vector>
#include <new>

/* Definitions */
#define ESPNOW_WIFI_IFACE WIFI_IF_STA
#define ESPNOW_WIFI_CHANNEL 4
#define ESPNOW_SEND_INTERVAL_MS 5000
#define ESPNOW_PEER_COUNT 1
#define REPORT_INTERVAL 5

#define ESPNOW_EXAMPLE_PMK "pmk1234567890123"
#define ESPNOW_EXAMPLE_LMK "lmk1234567890123"

// TFT Screen pins
#define TFT_CS   5
#define TFT_DC   2
#define TFT_RST  4
#define BL_PIN   32

// Rotary Encoder pins
#define CLK_PIN  3
#define DT_PIN   1

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

/* Structs */
typedef struct {
  uint32_t count;
  uint32_t priority;
  uint32_t data;
  bool ready;
  char str[7];
} __attribute__((packed)) esp_now_data_t;

/* Global Variables */
// Variables for rotary encoder
volatile int counter = 0;
volatile int lastEncoded = 0;
int lastDisplayedCounter = -999;

// Variables for display - sensor data
int lastDisplayedSensorData = -1;
volatile uint32_t sensorData = 0;

uint32_t self_priority = 0;
uint8_t current_peer_count = 0;
bool device_is_master = false;
bool master_decided = false;
uint32_t sent_msg_count = 0;
uint32_t recv_msg_count = 0;
esp_now_data_t new_msg;
std::vector<uint32_t> last_data(5);

/* Classes */
class ESP_NOW_Network_Peer : public ESP_NOW_Peer {
public:
  uint32_t priority;
  bool peer_is_master = false;
  bool peer_ready = false;

  ESP_NOW_Network_Peer(const uint8_t *mac_addr, uint32_t priority = 0, const uint8_t *lmk = (const uint8_t *)ESPNOW_EXAMPLE_LMK)
    : ESP_NOW_Peer(mac_addr, ESPNOW_WIFI_CHANNEL, ESPNOW_WIFI_IFACE, lmk), priority(priority) {}

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

    if (peer_ready == false && msg->ready == true) {
      Serial.printf("Peer " MACSTR " reported ready\n", MAC2STR(addr()));
      peer_ready = true;
    }

    if (!broadcast) {
      recv_msg_count++;
      if (device_is_master) {
        // Store sensor data for screen display
        sensorData = msg->data;
        
        // Full logging to Serial Monitor
        Serial.printf("Received a message from peer " MACSTR "\n", MAC2STR(addr()));
        Serial.printf("  Count: %lu\n", msg->count);
        Serial.printf("  Sensor data: %lu\n", msg->data);
        
        last_data.push_back(msg->data);
        last_data.erase(last_data.begin());
      } else if (peer_is_master) {
        Serial.println("Received a message from the master");
        Serial.printf("  Average data: %lu\n", msg->data);
      } else {
        Serial.printf("Peer " MACSTR " says: %s\n", MAC2STR(addr()), msg->str);
      }
    }
  }

  void onSent(bool success) {
    bool broadcast = memcmp(addr(), ESP_NOW.BROADCAST_ADDR, ESP_NOW_ETH_ALEN) == 0;
    if (broadcast) {
      log_i("Broadcast message reported as sent %s", success ? "successfully" : "unsuccessfully");
    } else {
      log_i("Unicast message reported as sent %s to peer " MACSTR, success ? "successfully" : "unsuccessfully", MAC2STR(addr()));
    }
  }
};

/* Peers */
std::vector<ESP_NOW_Network_Peer *> peers;
ESP_NOW_Network_Peer broadcast_peer(ESP_NOW.BROADCAST_ADDR, 0, nullptr);
ESP_NOW_Network_Peer *master_peer = nullptr;

/* Helper functions */
void updateEncoder() {
  int MSB = digitalRead(CLK_PIN);
  int LSB = digitalRead(DT_PIN);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) counter++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) counter--;

  lastEncoded = encoded;
}

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

uint32_t calc_average() {
  uint32_t avg = 0;
  for (auto &d : last_data) {
    avg += d;
  }
  avg /= last_data.size();
  return avg;
}

bool check_all_peers_ready() {
  for (auto &peer : peers) {
    if (!peer->peer_ready) {
      return false;
    }
  }
  return true;
}

/* Callbacks */
void register_new_peer(const esp_now_recv_info_t *info, const uint8_t *data, int len, void *arg) {
  esp_now_data_t *msg = (esp_now_data_t *)data;
  int priority = msg->priority;

  if (priority == self_priority) {
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
    peers.push_back(new_peer);
    current_peer_count++;
    if (current_peer_count == ESPNOW_PEER_COUNT) {
      Serial.println("All peers have been found");
      new_msg.ready = true;
    }
  }
}

void setup() {
  pinMode(BL_PIN, OUTPUT);
  digitalWrite(BL_PIN, HIGH);

  Serial.begin(115200);

  // Initialize display
  tft.init(240, 320);
  tft.setRotation(0);
  tft.fillScreen(ST77XX_BLACK);

  // Print Labels  
  tft.setTextSize(3);
  tft.setTextColor(ST77XX_WHITE);
  
  tft.setCursor(10, 20);
  tft.println("Set Point:");

  tft.drawLine(0, 100, 240, 100, ST77XX_WHITE);

  tft.setCursor(10, 110);
  tft.println("Water Level:");

  // Set rotary encoder pins
  pinMode(CLK_PIN, INPUT_PULLUP);
  pinMode(DT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CLK_PIN), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DT_PIN), updateEncoder, CHANGE);

  // ESP-NOW setup
  uint8_t self_mac[6];

  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }

  Serial.println("ESP-NOW Network Example");
  Serial.println("Wi-Fi parameters:");
  Serial.println("  Mode: STA");
  Serial.println("  MAC Address: " + WiFi.macAddress());
  Serial.printf("  Channel: %d\n", ESPNOW_WIFI_CHANNEL);

  WiFi.macAddress(self_mac);
  self_priority = 5;
  Serial.printf("This device's priority: %lu\n", self_priority);

  // Display Connection info
  tft.setCursor(10, 0);
  tft.setTextSize(1); 
  tft.setTextColor(ST77XX_WHITE);
  
  tft.print("MAC:");
  tft.print(WiFi.macAddress());

  tft.print(" Ch:");
  tft.print(ESPNOW_WIFI_CHANNEL);
  
  tft.print(" P:");
  tft.print(self_priority);
  
  tft.drawLine(0, 10, 240, 10, ST77XX_WHITE);

  if (!ESP_NOW.begin((const uint8_t *)ESPNOW_EXAMPLE_PMK)) {
    Serial.println("Failed to initialize ESP-NOW");
    fail_reboot();
  }

  Serial.printf("ESP-NOW version: %d, max data length: %d\n", ESP_NOW.getVersion(), ESP_NOW.getMaxDataLen());

  if (!broadcast_peer.begin()) {
    Serial.println("Failed to initialize broadcast peer");
    fail_reboot();
  }

  ESP_NOW.onNewPeer(register_new_peer, nullptr);

  Serial.println("Setup complete. Broadcasting own priority to find the master...");
  memset(&new_msg, 0, sizeof(new_msg));
  strncpy(new_msg.str, "Hello!", sizeof(new_msg.str));
  new_msg.priority = self_priority;

  delay(100);
}

void loop() {
  // ESP-NOW Master/Slave Decision Logic
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
              Serial.printf("Peer " MACSTR " is the master with priority %lu\n", MAC2STR(peers[i]->addr()), highest_priority);
              break;
            }
          }
        }
        Serial.println("The master has been decided");
      } else {
        Serial.println("Waiting for all peers to be ready...");
      }
    }
  } else {
    if (!device_is_master) {
      new_msg.count = sent_msg_count + 1;
      new_msg.data = random(10000);
      if (!master_peer->send_message((const uint8_t *)&new_msg, sizeof(new_msg))) {
        Serial.println("Failed to send message to the master");
      } else {
        Serial.printf("Sent message to the master. Count: %lu, Data: %lu\n", new_msg.count, new_msg.data);
        sent_msg_count++;
      }

      if (sent_msg_count % REPORT_INTERVAL == 0) {
        for (auto &peer : peers) {
          if (!peer->peer_is_master) {
            if (!peer->send_message((const uint8_t *)&new_msg, sizeof(new_msg))) {
              Serial.printf("Failed to send message to peer " MACSTR "\n", MAC2STR(peer->addr()));
            } else {
              Serial.printf("Sent message \"%s\" to peer " MACSTR "\n", new_msg.str, MAC2STR(peer->addr()));
            }
          }
        }
      }
    } else {
      if (recv_msg_count % REPORT_INTERVAL == 0 && recv_msg_count > 0) {
        uint32_t avg = calc_average();
        new_msg.data = avg;
        for (auto &peer : peers) {
          new_msg.count = sent_msg_count + 1;
          if (!peer->send_message((const uint8_t *)&new_msg, sizeof(new_msg))) {
            Serial.printf("Failed to send message to peer " MACSTR "\n", MAC2STR(peer->addr()));
          } else {
            Serial.printf(
              "Sent message to peer " MACSTR ". Recv: %lu, Sent: %lu, Avg: %lu\n", MAC2STR(peer->addr()), recv_msg_count, new_msg.count, new_msg.data
            );
            sent_msg_count++;
          }
        }
      }
    }
  }

  // Display rotary encoder value at TOP in large green numbers
  if (counter != lastDisplayedCounter) {
    lastDisplayedCounter = counter;
    
    tft.fillRect(10, 50, 90, 35, ST77XX_BLACK);
    tft.setCursor(10, 50);
    tft.setTextSize(5);
    tft.setTextColor(ST77XX_GREEN);
    tft.print(counter);
  
    // Display "cm" unit
    tft.setCursor(100, 65);
    tft.setTextSize(3);
    tft.setTextColor(ST77XX_WHITE);
    tft.print("cm^3");
  }

  // Display "Water Level" label and sensor value BELOW
  if (sensorData != lastDisplayedSensorData) {
    lastDisplayedSensorData = sensorData;
    
    // Clear water level section
    tft.fillRect(10, 145, 90, 35, ST77XX_BLACK);
    
    // Display sensor reading in large green numbers
    tft.setCursor(10, 145);
    tft.setTextSize(5);
    tft.setTextColor(ST77XX_GREEN);
    tft.print(sensorData*10.75*10.75*3.14159);
    
    // Display "cm" unit
    tft.setCursor(100, 160);
    tft.setTextSize(3);
    tft.setTextColor(ST77XX_WHITE);
    tft.print("cm");
  }

  delay(50);
}
