#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>
#include <string>

class Talker : public rclcpp::Node
{
public:
    Talker(const std::string & message) : Node("talker"), message_(message)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

        // Use timer to defer publishing to the event loop (safe way)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Talker::publish_once_and_exit, this)
        );
    }

private:
    void publish_once_and_exit()
    {
        auto msg = std_msgs::msg::String();
        msg.data = message_;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
        publisher_->publish(msg);

        // Stop spinning after publishing
        rclcpp::shutdown();
    }

    std::string message_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    if (argc < 2)
    {
        RCLCPP_ERROR(rclcpp::get_logger("talker"), "Usage: ros2 run my_ros2_talker_listener talker \"your_message\"");
        return 1;
    }

    std::string message_to_publish = argv[1];
    auto talker_node = std::make_shared<Talker>(message_to_publish);
    rclcpp::spin(talker_node);

    return 0;
}
