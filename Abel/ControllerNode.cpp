#include "SparkMax.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include <cmath>
#include <string>
#include <algorithm>

const float VELOCITY_MAX = 2500.0;  // Max RPM

enum CAN_IDs
{
    LEFT_MOTOR = 1,
    RIGHT_MOTOR = 2
};

namespace Gp
{
    enum Axes
    {
        _LEFT_HORIZONTAL_STICK = 0,
        _LEFT_VERTICAL_STICK = 1
    };

    enum Buttons
    {
        _LEFT_TRIGGER = 6,
        _RIGHT_TRIGGER = 7
    };
}

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode(const std::string &can_interface)
        : Node("controller_node"),
          leftMotor(can_interface, LEFT_MOTOR),
          rightMotor(can_interface, RIGHT_MOTOR)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing Motors...");

        leftMotor.SetIdleMode(IdleMode::kBrake);
        rightMotor.SetIdleMode(IdleMode::kBrake);

        leftMotor.SetMotorType(MotorType::kBrushless);
        rightMotor.SetMotorType(MotorType::kBrushless);

        leftMotor.SetInverted(false);
        rightMotor.SetInverted(true);

        leftMotor.BurnFlash();
        rightMotor.BurnFlash();

        RCLCPP_INFO(this->get_logger(), "Motors Initialized");

        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&ControllerNode::joy_callback, this, std::placeholders::_1));

        heartbeatPub = this->create_publisher<std_msgs::msg::String>("/heartbeat", 10);
        timer = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&ControllerNode::publish_heartbeat, this));
    }

private:
    SparkMax leftMotor;
    SparkMax rightMotor;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeatPub;
    rclcpp::TimerBase::SharedPtr timer;

    float computeStepOutput(float value)
    {
        float absVal = std::fabs(value);
        if (absVal < 0.25f) return 0.0f;
        else if (absVal < 0.5f) return (value > 0 ? 0.25f : -0.25f);
        else if (absVal < 0.75f) return (value > 0 ? 0.5f : -0.5f);
        else if (absVal < 1.0f) return (value > 0 ? 0.75f : -0.75f);
        return (value > 0 ? 1.0f : -1.0f);
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
        if (joy_msg->axes.size() < 2) return;

        // send CAN heartbeat
        try {
            leftMotor.Heartbeat();
            rightMotor.Heartbeat();
        } catch (const std::exception &ex) {
            RCLCPP_ERROR(this->get_logger(), "Heartbeat error: %s", ex.what());
        }

        // safety triggers
        bool triggersPressed = joy_msg->buttons.size() > 7 &&
                               (joy_msg->buttons[Gp::Buttons::_LEFT_TRIGGER] > 0 ||
                                joy_msg->buttons[Gp::Buttons::_RIGHT_TRIGGER] > 0);
        if (!triggersPressed)
        {
            leftMotor.SetDutyCycle(0.0f);
            rightMotor.SetDutyCycle(0.0f);
            return;
        }

        float forward = -joy_msg->axes[Gp::Axes::_LEFT_VERTICAL_STICK];
        float turn = joy_msg->axes[Gp::Axes::_LEFT_HORIZONTAL_STICK];

        float left = std::clamp(forward + turn, -1.0f, 1.0f);
        float right = std::clamp(forward - turn, -1.0f, 1.0f);

        leftMotor.SetVelocity(computeStepOutput(left) * VELOCITY_MAX);
        rightMotor.SetVelocity(computeStepOutput(right) * VELOCITY_MAX);
    }

    void publish_heartbeat()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "Heartbeat";
        heartbeatPub->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Heartbeat published");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string can_interface = "can0";
    auto node = std::make_shared<ControllerNode>(can_interface);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
