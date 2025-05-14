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

## 🛠️ How to Build

```bash
# Clone the repository
git clone https://github.com/yourusername/ros2_mqtt_bridge.git
cd ros2_mqtt_bridge
```

# Build using colcon

```bash
colcon build --packages-select mqtt_ros_bridge
```

# Source the ROS2 workspace
```bash
source install/setup.bash
```

## 🏃 How to Run

In a separate terminal, start a ROS2 publisher:
```bash
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello MQTT World!'"
```

Run the ROS2-MQTT bridge:
```bash
ros2 run mqtt_ros_bridge mqtt_ros_bridge_node
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
