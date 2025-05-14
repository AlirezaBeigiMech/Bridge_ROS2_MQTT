# 🚀 ROS2 to MQTT Bridge Example

[![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/rolling/index.html)
[![MQTT](https://img.shields.io/badge/MQTT-HiveMQ-brightgreen)](https://www.hivemq.com/mqtt/)
[![C++](https://img.shields.io/badge/C%2B%2B-17-blue)](https://en.cppreference.com/w/cpp/17)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

This repository demonstrates a simple **ROS2 to MQTT bridge** in **C++** using the **paho.mqtt.c** client library and ROS2 Jazzy.

The bridge subscribes to a ROS2 topic and publishes incoming messages to an MQTT broker. It's a minimal working example showing how to integrate ROS2 nodes with cloud MQTT services like **HiveMQ Cloud**.

---

## 📦 Features

- ✅ Subscribes to a ROS2 topic (`/chatter`)
- ✅ Publishes incoming messages to an MQTT topic
- ✅ Handles secure MQTT connections (SSL/TLS)
- ✅ Includes basic message receiving callback from MQTT broker
- ✅ Lightweight, minimal dependencies (no rclc, pure C++)

---
## 🌐 Connecting to HiveMQ Cloud


To connect your ROS2 MQTT Bridge to HiveMQ Cloud, follow these simple steps:
### 1️⃣ Create a HiveMQ Cloud Account

Go to: https://www.hivemq.com/mqtt-cloud-broker/
Sign up for a Free HiveMQ Cloud account.

### 2️⃣ Set Up Your MQTT Broker

After logging in, create a new cluster.
Note down these details:
```bash
Server URL (e.g., your-cluster-uuid.s2.eu.hivemq.cloud)
Port: 8883 (SSL/TLS)
Username & Password (auto-generated or set manually).
```


## 🛠️ How to Build

```bash
# Clone the repository
git clone https://github.com/AlirezaBeigiMech/ros2_mqtt_bridge.git
cd Bridge_ROS2_MQTT
```

# Build using colcon

```bash
colcon build --packages-select ros2_mqtt_bridge --cmake-clean-cache
```

# Source the ROS2 workspace
```bash
source install/setup.bash
```

## 🏃 How to Run

In a separate terminal, start a ROS2 publisher:
```bash
ros2 run ros2_mqtt_bridge talker "hello from CLI"
```

Run the ROS2-MQTT bridge:
```bash
ros2 run ros2_mqtt_bridge listener
```

## 📝 Configuration

Modify broker credentials and topic names in mqtt_ros_bridge_node.cpp:

```bash
#define ADDRESS  "ssl://your-hivemq-url:8883"
#define CLIENTID "your_client_id"
#define USERNAME "your_username"
#define PASSWORD "your_password"
#define TOPIC    "ros2/mqtt/topic"
```

## 📚 Dependencies

- ROS2 Jazzy
- Eclipse Paho MQTT C library (submodule in src/paho.mqtt.c)
- Standard C++17
