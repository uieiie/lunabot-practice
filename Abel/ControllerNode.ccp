#include "SparkMax.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "interfaces_pkg/msg/motor_health.hpp"
#include "interfaces_pkg/srv/depositing_request.hpp"
#include "interfaces_pkg/srv/excavation_request.hpp"
#include <cmath>
#include <string>
#include <cstdlib>
#include <algorithm>

const float VELOCITY_MAX = 2500.0; // Max RPM

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
    RCLCPP_INFO(this->get_logger(), "Initializing Drive Motors...");

    leftMotor.SetIdleMode(IdleMode::kBrake);
    rightMotor.SetIdleMode(IdleMode::kBrake);
    leftMotor.SetMotorType(MotorType::kBrushless);
    rightMotor.SetMotorType(MotorType::kBrushless);
    leftMotor.SetSensorType(SensorType::kHallSensor);
    rightMotor.SetSensorType(SensorType::kHallSensor);

    leftMotor.SetInverted(false);
    rightMotor.SetInverted(true);

    leftMotor.SetP(0, 0.0002f);
    leftMotor.SetI(0, 0.0f);
    leftMotor.SetD(0, 0.0f);
    leftMotor.SetF(0, 0.00021f);

    rightMotor.SetP(0, 0.0002f);
    rightMotor.SetI(0, 0.0f);
    rightMotor.SetD(0, 0.0f);
    rightMotor.SetF(0, 0.00021f);

    leftMotor.BurnFlash();
    rightMotor.BurnFlash();

    RCLCPP_INFO(this->get_logger(), "Motors Initialized.");

    // Subscribe to joystick topic
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        std::bind(&ControllerNode::joy_callback, this, std::placeholders::_1));
  }

private:
  SparkMax leftMotor;
  SparkMax rightMotor;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

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
    if (joy_msg->axes.size() < 2)
    {
      RCLCPP_WARN(this->get_logger(), "Joystick message missing axes.");
      return;
    }

    // Safety triggers (both must be pressed)
    bool triggersPressed = (joy_msg->buttons.size() > 7 &&
                            (joy_msg->buttons[Gp::Buttons::_LEFT_TRIGGER] > 0 ||
                             joy_msg->buttons[Gp::Buttons::_RIGHT_TRIGGER] > 0));

    if (!triggersPressed)
    {
      leftMotor.SetDutyCycle(0.0f);
      rightMotor.SetDutyCycle(0.0f);
      return;
    }

    // Drive control (arcade drive)
    float forward = -joy_msg->axes[Gp::Axes::_LEFT_VERTICAL_STICK];
    float turn = joy_msg->axes[Gp::Axes::_LEFT_HORIZONTAL_STICK];

    float left = forward + turn;
    float right = forward - turn;

    left = std::clamp(left, -1.0f, 1.0f);
    right = std::clamp(right, -1.0f, 1.0f);

    leftMotor.SetVelocity(computeStepOutput(left) * VELOCITY_MAX);
    rightMotor.SetVelocity(computeStepOutput(right) * VELOCITY_MAX);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::string can_interface = "can0";
  auto temp_node = rclcpp::Node::make_shared("controller_param_node");
  temp_node->declare_parameter<std::string>("can_interface", "can0");
  temp_node->get_parameter("can_interface", can_interface);

  auto node = std::make_shared<ControllerNode>(can_interface);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

