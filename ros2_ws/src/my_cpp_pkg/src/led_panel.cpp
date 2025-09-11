#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"
#include "my_robot_interfaces/msg/led_state_array.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class LedPanelNode : public rclcpp::Node
{
public:
    LedPanelNode() : Node("led_panel")
    {
        this->declare_parameter("led_states", std::vector<int64_t>{0, 0, 0});
        led_states_ = this->get_parameter("led_states").as_integer_array();

        server_ = this->create_service<my_robot_interfaces::srv::SetLed>("set_led",
                            std::bind(&LedPanelNode::callbackSetLed, this, _1, _2));
        publisher_ = this->create_publisher<my_robot_interfaces::msg::LedStateArray>(
            "led_panel_state", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(2),
                                         std::bind(&LedPanelNode::publishLedStatus, this));
        RCLCPP_INFO(this->get_logger(), "LED Panel Server C++ has been started.");
    }

private:
    void publishLedStatus()
    {
        auto msg = my_robot_interfaces::msg::LedStateArray();
        msg.led_panel = led_states_;
        publisher_->publish(msg);
    }

    void callbackSetLed(const my_robot_interfaces::srv::SetLed::Request::SharedPtr request,
                        const my_robot_interfaces::srv::SetLed::Response::SharedPtr response)
    {
        int64_t requested_number_;
        int64_t requested_state_;
        requested_number_ = request->led_number;
        requested_state_ = request->state;

        if (requested_number_ > (int64_t)led_states_.size() || requested_number_ <= 0){
            response->success = false;
            return;
        } 
        if (requested_state_ != 0 && requested_state_ != 1){
            response->success = false;
            return;
        }

        led_states_.at(requested_number_ - 1) = requested_state_;
        response->success = true;
        publishLedStatus();
    }


    std::vector<int64_t> led_states_;
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
