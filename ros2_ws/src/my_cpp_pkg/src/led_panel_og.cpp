#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"
#include "my_robot_interfaces/msg/led_state_array.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class LedPanelNode : public rclcpp::Node
{
public:
    LedPanelNode() : Node("led_panel"), requested_number_(0),requested_state_(0)
    {
        server_ = this->create_service<my_robot_interfaces::srv::SetLed>("set_led",
                            std::bind(&LedPanelNode::callbackSetLed, this, _1, _2));
       publisher_ = this->create_publisher<my_robot_interfaces::msg::LedStateArray>(
            "led_panel_state", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&LedPanelNode::publishLedStatus, this));
        RCLCPP_INFO(this->get_logger(), "LED Panel Server C++ has been started.");
    }

private:
    void callbackSetLed(const my_robot_interfaces::srv::SetLed::Request::SharedPtr request,
                        const my_robot_interfaces::srv::SetLed::Response::SharedPtr response)
    {
        requested_number_ = request->led_number;
        requested_state_ = request->state;
        response->success = true;
    }
    void publishLedStatus()
    {
        auto msg = my_robot_interfaces::msg::LedStateArray();
        if (requested_state_ == 0){
            msg.led_panel[requested_number_] = 0;
        } else if (requested_state_ == 1){
            msg.led_panel[requested_number_] = 1;
        }
        publisher_->publish(msg);
    }

    int requested_number_;
    int requested_state_;
    rclcpp::Publisher<my_robot_interfaces::msg::LedStateArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<my_robot_interfaces::srv::SetLed>::SharedPtr server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedPanelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
