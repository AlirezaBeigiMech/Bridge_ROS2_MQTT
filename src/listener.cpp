#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <unistd.h>
#include <string>
#include <iostream>

#include "MQTTClient.h"
#include "paho.mqtt.c/src/samples/pubsub_opts.h"


#define ADDRESS  "ssl://your-hivemq-url:8883"
#define CLIENTID "your_client_id"
#define USERNAME "your_username"
#define PASSWORD "your_password"
#define TOPIC    "ros2/mqtt/topic"

class MqttBridge {
    public:
        MqttBridge() : client(nullptr) {}
    
        ~MqttBridge() {
            if (client) {
                MQTTClient_unsubscribe(client, topic.c_str());
                MQTTClient_disconnect(client, timeout);
                MQTTClient_destroy(&client);
            }
        }
    
        bool setup() {
            MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
            MQTTClient_SSLOptions ssl_opts = MQTTClient_SSLOptions_initializer;
    
            MQTTClient_create(&client, address.c_str(), clientId.c_str(), MQTTCLIENT_PERSISTENCE_NONE, nullptr);
            MQTTClient_setCallbacks(client, nullptr, nullptr, &MqttBridge::messageArrived, nullptr);
    
            ssl_opts.enableServerCertAuth = 0;
            conn_opts.ssl = &ssl_opts;
            conn_opts.keepAliveInterval = 10;
            conn_opts.cleansession = 1;
            conn_opts.username = username.c_str();
            conn_opts.password = password.c_str();
    
            int rc = MQTTClient_connect(client, &conn_opts);
            if (rc != MQTTCLIENT_SUCCESS) {
                std::cerr << "Failed to connect to MQTT broker, rc=" << rc << std::endl;
                return false;
            }
    
            std::cout << "Connected to MQTT broker" << std::endl;
            MQTTClient_subscribe(client, topic.c_str(), qos);
            return true;
        }
    
        void publish(const std::string & payload) {
            MQTTClient_deliveryToken token;
            int rc = MQTTClient_publish(client, topic.c_str(), payload.size(), payload.c_str(), qos, 0, &token);
            if (rc != MQTTCLIENT_SUCCESS) {
                std::cerr << "Failed to publish MQTT message, rc=" << rc << std::endl;
            } else {
                std::cout << "Published MQTT message: " << payload << std::endl;
            }
        }
    
        static int messageArrived(void *context, char *topicName, int topicLen, MQTTClient_message *message) {
            (void)context;   // Mark as intentionally unused
            (void)topicLen;  // Mark as intentionally unused
        
            std::cout << "[MQTT] Message on topic '" << topicName << "': " 
                      << static_cast<char *>(message->payload) << std::endl;
        
            MQTTClient_freeMessage(&message);
            MQTTClient_free(topicName);
            return 1;
        }
    
    private:
        MQTTClient client;
    
        const std::string address = ADDRESS;
        const std::string clientId = CLIENTID;
        const std::string username = USERNAME;
        const std::string password = PASSWORD;
        const std::string topic = TOPIC;
        const int qos = 1;
        const long timeout = 10000L;
};



class Listener : public rclcpp::Node
{
public:
    Listener() : Node("listener"), mqtt_bridge()
    {
        if (!mqtt_bridge.setup()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to setup MQTT Bridge");
        }

        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "chatter", 10, std::bind(&Listener::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

        // Publish msg->data to MQTT
        mqtt_bridge.publish(msg->data);

        // Yield to process incoming MQTT messages
        MQTTClient_yield();
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    // Add MQTT Bridge as a mutable member (because callback is const)
    mutable MqttBridge mqtt_bridge;
};


int main(int argc, char * argv[])
{
    MqttBridge mqtt_bridge;
    if (!mqtt_bridge.setup()) {
        std::cerr << "Failed to setup MQTT" << std::endl;
        return 1;
    }



    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Listener>());
    rclcpp::shutdown();
    return 0;
}
