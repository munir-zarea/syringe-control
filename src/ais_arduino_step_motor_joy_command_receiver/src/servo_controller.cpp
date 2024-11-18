#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <serial/serial.h>  // Serial communication with Arduino
#include <chrono>
#include <cmath>            // For std::round()

class ServoController : public rclcpp::Node
{
public:
    ServoController()
        : Node("servo_controller"), baudrate_(9600), port_("/dev/ttyACM0"), current_position_(90), increment_step_(1), tolerance_(0.05)
    {
        // Open serial port
        try {
            openSerialPort();
        } catch (const serial::IOException &e) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open serial port on startup.");
        }

        // Subscribe to joystick topic
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&ServoController::joyCallback, this, std::placeholders::_1));

        // Timer to send updated commands
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // Check joystick input every 50ms
            std::bind(&ServoController::updateServoPosition, this));
    }

private:
    void openSerialPort()
    {
        try {
            serial_port_.setPort(port_);
            serial_port_.setBaudrate(baudrate_);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_port_.setTimeout(timeout);
            serial_port_.open();
        } catch (const serial::IOException &e) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open serial port on %s.", port_.c_str());
        }

        if (serial_port_.isOpen()) {
            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully on %s.", port_.c_str());
        }
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Joystick's right vertical axis (commonly `axes[4]`)
        vertical_axis_ = msg->axes[4]; // Adjust if your joystick uses a different axis index
    }

    void updateServoPosition()
    {
        // Ignore small movements (deadzone) to avoid noise
        if (std::abs(vertical_axis_) < tolerance_) {
            return; // Do nothing if joystick is near neutral
        }

        // Increment or decrement the position based on joystick input
        if (vertical_axis_ > 0) {
            current_position_ += increment_step_; // Move up
        } else if (vertical_axis_ < 0) {
            current_position_ -= increment_step_; // Move down
        }

        // Enforce limits (0 to 270 degrees)
        if (current_position_ < 0) current_position_ = 0;
        if (current_position_ > 270) current_position_ = 270;

        // Send updated position to the Arduino
        std::string command = std::to_string(current_position_) + "\n";

        try {
            if (serial_port_.isOpen()) {
                serial_port_.write(command);
                RCLCPP_INFO(this->get_logger(), "Moving servo to position: %d", current_position_);
            } else {
                RCLCPP_WARN(this->get_logger(), "Serial port not open. Attempting to reconnect...");
                openSerialPort();
            }
        } catch (const serial::IOException &e) {
            RCLCPP_ERROR(this->get_logger(), "IOException: %s", e.what());
            serial_port_.close();
        } catch (const serial::SerialException &e) {
            RCLCPP_ERROR(this->get_logger(), "SerialException: %s", e.what());
            serial_port_.close();
        }
    }

    // Serial parameters
    serial::Serial serial_port_;
    std::string port_;
    int baudrate_;
    int current_position_;  // Tracks the current servo position
    const int increment_step_; // Step size for each update
    const float tolerance_; // Deadzone tolerance for joystick input
    float vertical_axis_ = 0.0; // Tracks the current joystick axis value

    // ROS2 components
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServoController>());
    rclcpp::shutdown();
    return 0;
}

