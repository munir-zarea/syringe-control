#include <rclcpp/rclcpp.hpp>
#include <linux/joystick.h>  // For joystick input
#include <fcntl.h>           // For open()
#include <unistd.h>          // For close(), read()
#include <stdexcept>
#include <string>
#include <cstring>

class ServoController : public rclcpp::Node
{
public:
    ServoController() : Node("servo_controller"), last_position_(90)
    {
        // Declare and retrieve the joystick device parameter
        this->declare_parameter<std::string>("device_id", "/dev/input/js0");  // Default to /dev/input/js0
        this->get_parameter("device_id", device_id_);

        // Attempt to open the joystick device
        fd_ = open(device_id_.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open joystick device: %s", device_id_.c_str());
            throw std::runtime_error("Failed to open joystick device");
        }

        // Start a timer to read joystick events
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&ServoController::readJoystick, this));
    }

    ~ServoController()
    {
        if (fd_ >= 0) {
            close(fd_);  // Close the joystick file descriptor
        }
    }

private:
    void readJoystick()
    {
        struct js_event e;
        while (read(fd_, &e, sizeof(e)) > 0) {
            // Process joystick events
            if (e.type == JS_EVENT_BUTTON && e.number == 0) {
                bool button_pressed = e.value == 1;
                int servo_position = button_pressed ? 180 : 90;

                if (servo_position != last_position_) {
                    last_position_ = servo_position;
                    RCLCPP_INFO(this->get_logger(), "Button %s! Setting servo position to: %d",
                                button_pressed ? "pressed" : "released", servo_position);
                    // Here, you would send the position to your servo
                    // (e.g., through a topic or directly to hardware)
                }
            }
        }

        if (errno != EAGAIN) {
            RCLCPP_ERROR(this->get_logger(), "Error reading joystick events: %s", strerror(errno));
        }
    }

    std::string device_id_;  // Joystick device ID
    int fd_;                 // File descriptor for the joystick
    int last_position_;      // Last known servo position

    rclcpp::TimerBase::SharedPtr timer_;  // Timer for reading joystick events
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<ServoController>());
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Exception caught: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
