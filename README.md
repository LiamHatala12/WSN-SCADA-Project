# 🌊 WSN-SCADA Water Level Control System

<div align="center">

![ESP32](https://img.shields.io/badge/ESP32-Microcontroller-blue?style=for-the-badge&logo=espressif) ![ESP-NOW](https://img.shields.io/badge/ESP--NOW-Protocol-green?style=for-the-badge) ![FreeRTOS](https://img.shields.io/badge/FreeRTOS-RTOS-orange?style=for-the-badge) ![PID Control](https://img.shields.io/badge/PID-Control-red?style=for-the-badge)

**A Wireless Sensor Network (WSN) based SCADA system for real-time water level monitoring and control using ESP32 microcontrollers with ESP-NOW communication protocol.**

</div>

---

## 📋 Table of Contents

- [Overview](#-overview)
- [System Architecture](#-system-architecture)
- [Hardware Components](#-hardware-components)
- [Node Descriptions](#-node-descriptions)
- [Communication Protocol](#-communication-protocol)
- [PID Control System](#-pid-control-system)
- [Getting Started](#-getting-started)
- [Pin Configurations](#-pin-configurations)
- [Project Structure](#-project-structure)

---

## 🎯 Overview

This project implements a **Supervisory Control and Data Acquisition (SCADA)** system using a **Wireless Sensor Network (WSN)** architecture. The system monitors and controls water levels in a tank using:

- **ESP-NOW Protocol**: Low-latency, connectionless wireless communication
- **PID Controller**: Precise water level regulation
- **FreeRTOS**: Real-time multitasking on the Head Node
- **Master-Slave Polling**: Collision-free data exchange

### Key Features

✅ Real-time water level monitoring with ultrasonic sensor  
✅ Adjustable PID parameters (Kp, Ki, Kd) via HMI  
✅ TFT-based Human Machine Interface with rotary encoder  
✅ PWM-controlled pump actuator with voltage-aware operation  
✅ Servo-based disturbance injection for testing  
✅ Encrypted ESP-NOW communication with PMK/LMK  

---

## 🏗 System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                          HEAD NODE (Master)                         │
│                           Priority: 5                               │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │  • ESP-NOW Master Controller                                │    │
│  │  • PID Control Algorithm                                    │    │
│  │  • FreeRTOS Task Management (Polling + Control Tasks)       │    │
│  │  • Sensor Data Aggregation                                  │    │
│  └─────────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────────┘
                                   │
          ┌────────────────────────┼────────────────────────┐
          │                        │                        │
          ▼                        ▼                        ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────────┐
│   SENSOR NODE   │    │    HMI NODE     │    │   ACTUATOR NODES    │
│  Priority: 1    │    │   Priority: 0   │    │  Pump (3) / Servo(2)│
├─────────────────┤    ├─────────────────┤    ├─────────────────────┤
│ • Ultrasonic    │    │ • ST7789 TFT    │    │ • L298N Motor Driver│
│   HC-SR04       │    │ • Rotary Encoder│    │ • HS-422 Servo      │
│ • Distance →    │    │ • Setpoint Input│    │ • PWM Control       │
│   Water Level   │    │ • PID Tuning    │    │ • 10V-12V Operation │
└─────────────────┘    └─────────────────┘    └─────────────────────┘
```

---

## 🔧 Hardware Components

| Component | Quantity | Purpose |
|-----------|----------|---------|
| ESP32 DevKit | 5 | Microcontrollers for each node |
| HC-SR04 Ultrasonic Sensor | 1 | Water level measurement |
| ST7789 TFT Display (240x320) | 1 | Human Machine Interface |
| Rotary Encoder with Button | 1 | User input for HMI |
| L298N Motor Driver (HW-095) | 1 | Pump PWM control |
| 12V DC Water Pump | 1 | Water flow actuator |
| HS-422 Servo Motor | 1 | Disturbance injection |
| 12V Power Supply | 1 | System power |

---

## 📡 Node Descriptions

### 1. Head Node (Master Controller)
**File:** `HeadNodeCode/HeadNodeCode.ino`

The central coordinator running FreeRTOS with two main tasks:

| Task | Period | Function |
|------|--------|----------|
| **Poll Task** | 300ms | Polls HMI and Sensor nodes sequentially |
| **Control Task** | 300ms | Executes PID algorithm, sends commands to actuators |

**Features:**
- Master election based on priority (highest priority = 5)
- Thread-safe PID parameter updates via mutex protection
- Queue-based sensor data passing between tasks
- Anti-windup protection for integral term

### 2. Sensor Node
**File:** `SensorNodeCode/SensorNodeCode.ino`

**Priority:** 1

Measures water level using ultrasonic time-of-flight:
- Responds only when polled by Head Node
- Returns distance in millimeters
- Head converts to water level: `water_level = MAX_HEIGHT - distance`

### 3. HMI Node (Human Machine Interface)
**File:** `HMINodeCode/HMINodeCode.ino`

**Priority:** 0

Interactive control panel featuring:
- **240x320 TFT Display** showing:
  - Current setpoint (mm and %)
  - Actual water level (mm, %, and volume in mL)
  - PID parameters (Kp, Ki, Kd)
  - Actuator status (pump %, servo angle)
- **Rotary Encoder** for parameter adjustment
- **5 Edit Modes:** Setpoint, P, I, D, Servo Position

### 4. Pump Actuator Node
**File:** `PumpActuatorNodeCode/PumpActuatorNodeCode.ino`

**Priority:** 3

Controls water inflow via PWM:
- Receives pump power percentage from Head Node
- Maps 83-100% duty cycle to 10V-12V operating range
- Below 83% duty = pump OFF (below minimum operating voltage)
- 20kHz PWM frequency, 8-bit resolution

### 5. Servo Actuator Node
**File:** `ServoActuatorNodeCode/ServoActuatorNodeCode.ino`

**Priority:** 2

Simulates system disturbances:
- Controls valve/disturbance position
- 0-180° range with 900-2100μs pulse width
- Position set via HMI for testing PID response

---

## 📨 Communication Protocol

### Message Types

| Type | ID | Direction | Description |
|------|------|-----------|-------------|
| `MSG_DISCOVERY` | 0 | Broadcast | Node discovery during startup |
| `MSG_POLL_SENSOR` | 1 | Head → Sensor | Request sensor reading |
| `MSG_SENSOR_DATA` | 2 | Sensor → Head | Distance measurement |
| `MSG_POLL_HMI` | 3 | Head → HMI | Request setpoint & PID params |
| `MSG_HMI_SETPOINT` | 4 | HMI → Head | Setpoint + Kp, Ki, Kd |
| `MSG_CONTROL_COMMAND` | 5 | Head → Pump | Pump power percentage |
| `MSG_CONTROL_STATUS` | 6 | Head → HMI | Current water level & status |
| `MSG_SERVO_SETPOINT` | 7 | HMI → Head | Servo disturbance angle |
| `MSG_SERVO_COMMAND` | 8 | Head → Servo | Servo position command |

### Message Structure

```cpp
typedef struct {
  uint8_t  msg_type;    // Message type enum
  uint8_t  rsvd[3];     // Reserved/padding
  uint32_t count;       // Message counter
  uint32_t priority;    // Sender priority
  int32_t  data;        // Primary data field
  int32_t  data2;       // Secondary data field
  int32_t  data3;       // Tertiary data field
  int32_t  data4;       // Quaternary data field
  bool     ready;       // Node ready flag
} __attribute__((packed)) esp_now_data_t;
```

### Polling Sequence

```
1. Head → HMI:    MSG_POLL_HMI
2. HMI → Head:    MSG_HMI_SETPOINT + MSG_SERVO_SETPOINT
       [75ms gap]
3. Head → Sensor: MSG_POLL_SENSOR
4. Sensor → Head: MSG_SENSOR_DATA
       [PID computation]
5. Head → Pump:   MSG_CONTROL_COMMAND
6. Head → Servo:  MSG_SERVO_COMMAND
7. Head → HMI:    MSG_CONTROL_STATUS
```

---

## 🎛 PID Control System

### Algorithm

The Head Node implements a discrete PID controller:

$$u(t) = K_p \cdot e(t) + K_i \cdot \int_0^t e(\tau) d\tau + K_d \cdot \frac{de(t)}{dt}$$

Where:
- $e(t) = \text{setpoint} - \text{water level}$
- $u(t)$ = control output (0-100%)

### Parameters

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| Kp | 2.0 | 0-100 | Proportional gain |
| Ki | 0.5 | 0-100 | Integral gain |
| Kd | 1.0 | 0-100 | Derivative gain |
| Setpoint | 120mm | 0-240mm | Target water level |

### Safety Features

- **Anti-windup:** Integral term clamped to ±200
- **Output limiting:** PID output constrained to -100 to +100
- **Negative error handling:** Pump OFF when level exceeds setpoint

---

## 🚀 Getting Started

### Prerequisites

- Arduino IDE with ESP32 board support
- Required Libraries:
  - `ESP32_NOW` (included in ESP32 core)
  - `Adafruit_GFX`
  - `Adafruit_ST7789`
  - `ESP32Servo`

### Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/yourusername/WSN-SCADA-Project.git
   ```

2. **Configure MAC addresses:**
   
   Update the MAC addresses in each node's code to match your ESP32 devices:
   ```cpp
   // In HeadNodeCode.ino
   static uint8_t HMI_MAC[]    = {0x1c, 0x69, 0x20, 0x92, 0x60, 0x30};
   static uint8_t SENSOR_MAC[] = {0xb0, 0xa7, 0x32, 0x2b, 0x6c, 0xd8};
   static uint8_t SERVO_MAC[]  = {0xb0, 0xa7, 0x32, 0x2b, 0x0f, 0x20};
   static uint8_t PUMP_MAC[]   = {0x24, 0xdc, 0xc3, 0x45, 0xf3, 0x50};
   ```

3. **Upload code to each ESP32:**
   - HeadNodeCode → Head Node ESP32
   - SensorNodeCode → Sensor Node ESP32
   - HMINodeCode → HMI Node ESP32
   - PumpActuatorNodeCode → Pump Node ESP32
   - ServoActuatorNodeCode → Servo Node ESP32

4. **Power on all nodes** - Discovery happens automatically via broadcast

---

## 📌 Pin Configurations

### Head Node
| Function | Pin |
|----------|-----|
| WiFi (ESP-NOW) | Internal |

### Sensor Node
| Function | Pin |
|----------|-----|
| Ultrasonic Trigger/Echo | GPIO 23 |

### HMI Node
| Function | Pin |
|----------|-----|
| TFT CS | GPIO 5 |
| TFT DC | GPIO 2 |
| TFT RST | GPIO 4 |
| TFT Backlight | GPIO 32 |
| Encoder CLK | GPIO 21 |
| Encoder DT | GPIO 22 |
| Encoder SW | GPIO 19 |

### Pump Actuator Node
| Function | Pin |
|----------|-----|
| L298N ENA (PWM) | GPIO 18 |
| L298N IN1 | GPIO 19 |
| L298N IN2 | GPIO 21 |

### Servo Actuator Node
| Function | Pin |
|----------|-----|
| Servo Signal | GPIO 23 |

---

## 📂 Project Structure

```
WSN-SCADA-Project/
├── README.md                          # This file
├── HeadNodeCode/
│   └── HeadNodeCode.ino              # Master controller with FreeRTOS
├── SensorNodeCode/
│   └── SensorNodeCode.ino            # Ultrasonic sensor node
├── HMINodeCode/
│   └── HMINodeCode.ino               # TFT display interface
├── PumpActuatorNodeCode/
│   └── PumpActuatorNodeCode.ino      # Pump motor control
├── ServoActuatorNodeCode/
│   └── ServoActuatorNodeCode.ino     # Servo disturbance control
├── WSN Project Proposal Presentation.pdf
└── WSN Project Final Presentation.pdf
```

---

## 🔐 Security

- **PMK (Primary Master Key):** `pmk1234567890123`
- **LMK (Local Master Key):** `lmk1234567890123`

> ⚠️ **Note:** Change these keys for production deployments!

---

## 📊 System Specifications

| Parameter | Value |
|-----------|-------|
| Communication Protocol | ESP-NOW |
| WiFi Channel | 4 |
| Control Loop Period | 300ms |
| Polling Period | 300ms |
| Max Water Level | 260mm |
| Tank Radius | 10.75cm |
| PWM Frequency (Pump) | 20kHz |
| PWM Resolution | 8-bit |

---

## 👥 Contributors

This project was developed as part of a Wireless Sensor Networks course project.

- **Riley Johnson**
- **Liam Hatala**
- **Rehan Siddiqi**
- **Hassan Ahmad**

---
