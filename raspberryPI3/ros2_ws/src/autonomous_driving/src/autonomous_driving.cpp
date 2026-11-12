
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <algorithm>
#include <cstdint>

#include "interfaces/msg/motors_order.hpp"
#include "interfaces/msg/autonomous_command.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class autonomous_drive : public rclcpp::Node
{
public:

    autonomous_drive()
        : Node("autonomous_driving_node")
    {
        // Initialize sequence state
        sequenceStarted = false;
        sequenceFinished = false;

        // Initialize motor commands
        leftRearPwmCmd = 50;
        rightRearPwmCmd = 50;
        steeringAngle = 0;

        // Publisher: motor commands
        publisher_can_ =
            this->create_publisher<interfaces::msg::MotorsOrder>(
                "motors_order", 10);

        // Subscriber: autonomous start/stop command
        subscription_start_ =
            this->create_subscription<interfaces::msg::AutonomousCommand>(
                "autonomous_command", 10,
                std::bind(&autonomous_drive::startCallback, this, _1));

        // Timer: update commands every 50 ms
        timer_ = this->create_wall_timer(
            50ms,
            std::bind(&autonomous_drive::updateCmd, this));

        RCLCPP_INFO(
            this->get_logger(),
            "autonomous_driving_node READY - waiting for command");
    }

private:

    // ==========================================
    // START / STOP CALLBACK
    // ==========================================

    void startCallback(
        const interfaces::msg::AutonomousCommand::SharedPtr msg)
    {
        if (msg->start && !sequenceStarted)
        {
            sequenceStarted = true;
            sequenceFinished = false;

            // Record sequence starting time
            time_start = this->now();

            RCLCPP_INFO(this->get_logger(), "Sequence STARTED");
        }
        else if (!msg->start)
        {
            sequenceStarted = false;
            sequenceFinished = false;

            arreter();

            RCLCPP_INFO(this->get_logger(), "Sequence STOPPED");
        }
    }


    // ==========================================
    // PERIODIC COMMAND UPDATE
    // ==========================================

    void updateCmd()
    {
        auto motorsOrder = interfaces::msg::MotorsOrder();

        if (sequenceStarted && !sequenceFinished)
        {
            double elapsed = (this->now() - time_start).seconds();

            if (elapsed < 3.0)
            {
                // 0-3 seconds: move forward
                avance(0.5);
            }
            else if (elapsed < 6.0)
            {
                // 3-6 seconds: move backwards
                reculer(0.5);
            }
            else if (elapsed < 8.0)
            {
                // 6-8 seconds: turn right while moving
                droite(0.3);
            }
            else
            {
                // After 8 seconds: stop
                arreter();

                sequenceFinished = true;

                RCLCPP_INFO(
                    this->get_logger(),
                    "Sequence FINISHED");
            }
        }
        else
        {
            // Stay stopped while waiting for a new command
            arreter();
        }

        // Fill motor message
        motorsOrder.left_rear_pwm = leftRearPwmCmd;
        motorsOrder.right_rear_pwm = rightRearPwmCmd;
        motorsOrder.steering_angle = steeringAngle;

        // Publish motor command
        publisher_can_->publish(motorsOrder);
    }


    // ==========================================
    // MOVEMENT FUNCTIONS
    // ==========================================

    void avance(float throttle)
    {
        manualPropulsionCmd(
            throttle, false,
            leftRearPwmCmd, rightRearPwmCmd);

        steeringAngle = 0;
    }


    void reculer(float throttle)
    {
        manualPropulsionCmd(
            throttle, true,
            leftRearPwmCmd, rightRearPwmCmd);

        steeringAngle = 0;
    }


    void droite(float throttle)
    {
        // Move forward while steering right
        manualPropulsionCmd(
            throttle, false,
            leftRearPwmCmd, rightRearPwmCmd);

        steeringAngle = 50;
    }


    void arreter()
    {
        // Neutral motor PWM
        leftRearPwmCmd = 50;
        rightRearPwmCmd = 50;

        // Center steering
        steeringAngle = 0;
    }


    // ==========================================
    // PROPULSION HELPER
    // ==========================================

    void manualPropulsionCmd(
        float requestedThrottle,
        bool reverse,
        int8_t & leftPwm,
        int8_t & rightPwm)
    {
        // Limit throttle to [0,1]
        requestedThrottle =
            std::clamp(requestedThrottle, 0.0f, 1.0f);

        if (reverse)
        {
            leftPwm = static_cast<int8_t>(
                50 - 50 * requestedThrottle);
        }
        else
        {
            leftPwm = static_cast<int8_t>(
                50 + 50 * requestedThrottle);
        }

        rightPwm = leftPwm;
    }


    // ==========================================
    // VARIABLES
    // ==========================================

    bool sequenceStarted = false;
    bool sequenceFinished = false;

    rclcpp::Time time_start;

    int8_t leftRearPwmCmd = 50;
    int8_t rightRearPwmCmd = 50;
    int8_t steeringAngle = 0;


    // ==========================================
    // ROS 2 COMMUNICATION
    // ==========================================

    rclcpp::Publisher<interfaces::msg::MotorsOrder>::SharedPtr
        publisher_can_;

    rclcpp::Subscription<interfaces::msg::AutonomousCommand>::SharedPtr
        subscription_start_;

    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<autonomous_drive>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
