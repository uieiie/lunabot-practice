#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"

#include </home/developer/suave-ros/suave-ros/src/controller/src/suave_ros_sparkmax.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode() : Node("controller_node")
    {
        motorLeftID = this->declare_parameter<int>("motor_left_id", 1);
        motorRightID = this->declare_parameter<int>("motor_right_id", 2);

        stickID = this->declare_parameter<int>("stick_id", 1);
        tickRateMS = this->declare_parameter<int>("tick_rate_ms", 500);

        leftMotor = suave::CEREVOSparkMax(motorLeftID);
        rightMotor = suave::CEREVOSparkMax(motorRightID);

        leftMotor.SetInverted(false);
        rightMotor.SetInverted(false);

        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy",
            10,
            std::bind(&ControllerNode::joy_callback, this, _1)
        );

        heartbeatPub = this->create_publisher<std_msgs::msg::String>("/heartbeat", 10);

        timer = this->create_wall_timer(
            std::chrono::milliseconds(tickRateMS),
            std::bind(&ControllerNode::publish_heartbeat, this)
        );

        RCLCPP_INFO(this->get_logger(), "Controller node with heartbeats initialized");
    }

private:
    void publish_heartbeat()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "Heartbeat";
        heartbeatPub->publish(msg);
    }
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->axes.size() < 4 || msg->buttons.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Invalid joystick message received");
            return;
        }

        try {
            leftMotor.Heartbeat();
            rightMotor.Heartbeat();
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Heartbeat error: %s", ex.what());
        }

        if (msg->buttons[5] == 0 && msg->buttons[7] == 0)
        {
            leftMotor.Cease();
            rightMotor.Cease();
            RCLCPP_INFO(this->get_logger(), "Motor safety deactivated");
            return;
        }

        double forward = msg->axes[1];
        double rotation = msg->axes[3];

        double leftPower = forward + rotation;
        double rightPower = forward - rotation;

        leftPower = std::clamp(leftPower, -1.0, 1.0);
        rightPower = std::clamp(rightPower, -1.0, 1.0);

        RCLCPP_INFO(this->get_logger(), "L: %.2f  R: %.2f", leftPower, rightPower);

        try
        {
            leftMotor.Move(leftPower);
            rightMotor.Move(rightPower);
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send CAN commands: %s", ex.what());
        }
    }

    int motorLeftID;
    int motorRightID;
    int stickID;
    int tickRateMS;

    suave::CEREVOSparkMax leftMotor;
    suave::CEREVOSparkMax rightMotor;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeatPub;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}

