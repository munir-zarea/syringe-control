#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/int32.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <chrono>

class ServoController : public rclcpp::Node
{
public:
    ServoController() : Node("servo_controller"), servo_position_(90)
    {
        // Initialize joystick subscriber
        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&ServoController::joy_callback, this, std::placeholders::_1));

        // Open serial communication with Arduino
        serial_port_.open("/dev/ttyACM0", std::ios::out);
        if (!serial_port_.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: /dev/ttyACM0");
            rclcpp::shutdown();
        }

        last_update_time_ = this->now(); // Initialize the timer

        RCLCPP_INFO(this->get_logger(), "Servo Controller Node initialized.");
    }

    ~ServoController()
    {
        if (serial_port_.is_open())
        {
            serial_port_.close();
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    std::ofstream serial_port_; // Serial connection to Arduino
    int servo_position_;        // Current servo position (90 degrees is neutral)
    rclcpp::Time last_update_time_; // Time of the last servo command

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Assuming the right joystick's vertical axis is axis 4
        const float dead_zone = 0.05; // Ignore small movements
        float axis_value = msg->axes[4];

        // Introduce a delay between commands (e.g., 100 ms)
        auto current_time = this->now();
        auto time_diff = current_time - last_update_time_;

        if (std::abs(axis_value) > dead_zone && time_diff > rclcpp::Duration::from_seconds(0.1))
        {
            // Adjust the servo position based on joystick movement
            servo_position_ += static_cast<int>(axis_value * 2); // Scale movement slower
            servo_position_ = std::clamp(servo_position_, 0, 180); // Limit range

            RCLCPP_INFO(this->get_logger(), "Sending Servo Position: %d", servo_position_);

            // Send the servo position to Arduino via serial
            if (serial_port_.is_open())
            {
                serial_port_ << servo_position_ << '\n';
                serial_port_.flush(); // Ensure data is sent immediately
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Serial port not open!");
            }

            // Update the last command time
            last_update_time_ = current_time;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServoController>());
    rclcpp::shutdown();
    return 0;
}
