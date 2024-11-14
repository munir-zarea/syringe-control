#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <serial/serial.h>  // Serial communication with Arduino
#include <stdexcept>

class ServoController : public rclcpp::Node
{
public:
    ServoController() : Node("servo_controller"), baudrate_(9600), last_position_(90), servo_active_(false)
    {
        // Attempt to open the serial port
        try {
            openSerialPort();
        } catch (const serial::IOException &e) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open serial port on startup.");
        }

        // Subscribe to joystick topic
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&ServoController::joyCallback, this, std::placeholders::_1));
    }

private:
    void openSerialPort()
    {
        // Try opening /dev/ttyACM0 first
        port_ = "/dev/ttyACM0";
        try {
            serial_port_.setPort(port_);
            serial_port_.setBaudrate(baudrate_);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_port_.setTimeout(timeout);
            serial_port_.open();
        } catch (const serial::IOException &e) {
            // If /dev/ttyACM0 fails, try /dev/ttyACM1
            port_ = "/dev/ttyACM1";
            try {
                serial_port_.setPort(port_);
                serial_port_.open();
            } catch (const serial::IOException &e) {
                RCLCPP_ERROR(this->get_logger(), "Unable to open serial port on either /dev/ttyACM0 or /dev/ttyACM1.");
            }
        }

        if (serial_port_.isOpen()) {
            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully on %s.", port_.c_str());
        }
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Check if the button is pressed (button index 0)
        bool button_pressed = msg->buttons[0] == 1;

        int servo_position = button_pressed ? 180 : 90;  // 180 if pressed, 90 if released

        if (servo_position != last_position_) {
            last_position_ = servo_position;
            std::string command = std::to_string(servo_position) + "\n";

            RCLCPP_INFO(this->get_logger(), "Button %s! Setting servo position to: %d",
                        button_pressed ? "pressed" : "released", servo_position);

            try {
                if (serial_port_.isOpen()) {
                    RCLCPP_INFO(this->get_logger(), "Sending command to Arduino: %s", command.c_str());
                    serial_port_.write(command);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Serial port not open. Attempting to reconnect...");
                    openSerialPort();
                }
            } catch (const serial::SerialException &e) {
                RCLCPP_ERROR(this->get_logger(), "SerialException: %s", e.what());
                serial_port_.close();  // Close the port if there’s an error
            } catch (const serial::IOException &e) {
                RCLCPP_ERROR(this->get_logger(), "IOException: %s", e.what());
                serial_port_.close();
            }
        }
    }

    // Serial parameters
    serial::Serial serial_port_;
    std::string port_;
    int baudrate_;

    // Last known servo position to avoid redundant commands
    int last_position_;
    bool servo_active_;  // Indicates if the servo is active

    // ROS2 Subscriber
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServoController>());
    rclcpp::shutdown();
    return 0;
}

