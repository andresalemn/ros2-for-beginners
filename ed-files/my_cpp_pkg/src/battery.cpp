#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class BatteryNode : public rclcpp::Node
{
public:
    BatteryNode() : Node("battery"), battery_state_("full")
    {
        last_time_battery_state_changed_ = this->get_clock()->now().seconds();
        battery_timer_ = this->create_wall_timer(
            0.1s,
            std::bind(&BatteryNode::checkBatteryState, this));
        set_led_client_ = this->create_client<my_robot_interfaces::srv::SetLed>("set_led");
        while (!set_led_client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Service Server to be up...");
        }
        RCLCPP_INFO(this->get_logger(), "Battery node has been started.");
    }

private:
    void checkBatteryState()
    {
        double time_now = this->get_clock()->now().seconds();
        if (battery_state_ == "full")
        {
            if (time_now - last_time_battery_state_changed_ > 4.0)
            {
                RCLCPP_INFO(this->get_logger(), "Battery is empty! Charging battery...");
                battery_state_ = "empty";
                last_time_battery_state_changed_ = time_now;
                callSetLed(2, 1);
            }
        }
        else
        {
            if (time_now - last_time_battery_state_changed_ > 6.0)
            {
                RCLCPP_INFO(this->get_logger(), "Battery is now full again.");
                battery_state_ = "full";
                last_time_battery_state_changed_ = time_now;
                callSetLed(2, 0);
            }
        }
    }

    void callSetLed(int led_number, int state)
    {
        auto request = std::make_shared<my_robot_interfaces::srv::SetLed::Request>();
        request->led_number = led_number;
        request->state = state;

        auto future = set_led_client_->async_send_request(
            request, std::bind(&BatteryNode::callbackCallSetLed, this, _1));
    }

    void callbackCallSetLed(rclcpp::Client<my_robot_interfaces::srv::SetLed>::SharedFuture future)
    {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "LED state was changed");
        }
        else {
            RCLCPP_INFO(this->get_logger(), "LED not changed");
        }
    }

    std::string battery_state_;
    double last_time_battery_state_changed_;

    rclcpp::TimerBase::SharedPtr battery_timer_;
    rclcpp::Client<my_robot_interfaces::srv::SetLed>::SharedPtr set_led_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}