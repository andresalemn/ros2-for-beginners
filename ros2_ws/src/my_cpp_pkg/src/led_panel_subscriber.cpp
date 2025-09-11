#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/led_state_array.hpp"

class LedPanelSubscriber : public rclcpp::Node
{
public:
    LedPanelSubscriber() : Node("led_panel_subscriber")
    {
        subscription_ = this->create_subscription<my_robot_interfaces::msg::LedStateArray>(
            "led_panel_state", 10, std::bind(&LedPanelSubscriber::topic_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "LED Panel Subscriber C++ has been started.");
    }

private:
    void topic_callback(const my_robot_interfaces::msg::LedStateArray::SharedPtr msg) const
    {
        std::string output = "led_panel: [";
        for (size_t i = 0; i < msg->led_panel.size(); ++i)
        {
            output += std::to_string(msg->led_panel[i]);
            if (i != msg->led_panel.size() - 1)
            {
                output += ", ";
            }
        }
        output += "]";
        RCLCPP_INFO(this->get_logger(), output.c_str());
    }

    rclcpp::Subscription<my_robot_interfaces::msg::LedStateArray>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedPanelSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
