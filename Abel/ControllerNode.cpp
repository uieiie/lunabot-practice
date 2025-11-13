#include "SparkMax.hpp"
#include <rclcpp/rclcpp.hpp>
#include <general_msgs/msg/joy.hpp>
#include <controller_pkg/srv/excavation_request.hpp>
#include <cmath>
#include <string>

const float VELOCITY_MAX = 2500.0f;

enum CAN_IDs
{
    LEFT_MOTOR = 1,
    RIGHT_MOTOR = 2
};

namespace Gp
{
    enum Buttons
    {
        _A = 0,
        _B = 1,
        _X = 2,
        _Y = 3,
        _LEFT_BUMPER = 4,
        _RIGHT_BUMPER = 5,
        _LEFT_TRIGGER = 6,
        _RIGHT_TRIGGER = 7,
        _WINDOW_KEY = 8,
        _D_PAD_UP = 12,
        _D_PAD_DOWN = 13,
        _D_PAD_LEFT = 14,
        _D_PAD_RIGHT = 15,
        _X_BOX_KEY = 16
    };

    enum Axes
    {
        _LEFT_HORIZONTAL_STICK = 0,
        _LEFT_VERTICAL_STICK = 1,
        _RIGHT_HORIZONTAL_STICK = 2,
        _RIGHT_VERTICAL_STICK = 3
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
        RCLCPP_INFO(this->get_logger(), "Starting controller node...");
        RCLCPP_INFO(this->get_logger(), "Setting up motors...");

        leftMotor.SetIdleMode(IdleMode::kBrake);
        rightMotor.SetIdleMode(IdleMode::kBrake);
        leftMotor.SetMotorType(MotorType::kBrushless);
        rightMotor.SetMotorType(MotorType::kBrushless);

        excavation_client_ = this->create_client<controller_pkg::srv::ExcavationRequest>(
            "excavation_service");

        joy_subscriber_ = this->create_subscription<general_msgs::msg::Joy>(
            "/joy",
            10,
            std::bind(&ControllerNode::joy_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ControllerNode::heartbeat_timer, this));
    }

private:
    // step function for joystick
    float joystickToDutyCycle(float value)
    {
        if (value > 1.0f)
            value = 1.0f;
        if (value < -1.0f)
            value = -1.0f;

        float a = std::fabs(value);

        if (a < 0.25f)
            return 0.0f;
        else if (a < 0.5f)
            return (value > 0 ? 0.25f : -0.25f);
        else if (a < 0.75f)
            return (value > 0 ? 0.5f : -0.5f);
        else if (a < 1.0f)
            return (value > 0 ? 0.75f : -0.75f);

        return (value > 0 ? 1.0f : -1.0f);
    }

    void send_excavation_request()
    {
        if (!excavation_client_->wait_for_service(std::chrono::milliseconds(500)))
        {
            RCLCPP_WARN(this->get_logger(), "Excavation service not available.");
            return;
        }

        auto req = std::make_shared<controller_pkg::srv::ExcavationRequest::Request>();
        req->start_excavation = true;

        excavation_client_->async_send_request(req);
        RCLCPP_INFO(this->get_logger(), "Sent excavation request.");
    }

    void joy_callback(const general_msgs::msg::Joy::SharedPtr msg)
    {
        float leftJS = -msg->axes[Gp::Axes::_LEFT_VERTICAL_STICK];
        float rightJS = -msg->axes[Gp::Axes::_RIGHT_VERTICAL_STICK];

        float leftOut = joystickToDutyCycle(leftJS);
        float rightOut = joystickToDutyCycle(rightJS);

        leftMotor.SetDutyCycle(leftOut);
        rightMotor.SetDutyCycle(rightOut);

        if (msg->buttons[Gp::Buttons::_A])
        {
            send_excavation_request();
        }
    }

    void heartbeat_timer()
    {
        try
        {
            leftMotor.Heartbeat();
            rightMotor.Heartbeat();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Heartbeat error: %s", e.what());
        }
    }

    rclcpp::Subscription<general_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Client<controller_pkg::srv::ExcavationRequest>::SharedPtr excavation_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    SparkMax leftMotor;
    SparkMax rightMotor;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::string can_interface = "can0";
    auto param_node = rclcpp::Node::make_shared("controller_param_node");

    param_node->declare_parameter<std::string>("can_interface", "can0");
    param_node->get_parameter("can_interface", can_interface);

    auto node = std::make_shared<ControllerNode>(can_interface);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
