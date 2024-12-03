#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

class ServoController : public rclcpp::Node
{
public:
    ServoController() : Node("servo_controller"), last_position_(90)
    {
        // Subscribe to the joystick topic
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&ServoController::joyCallback, this, std::placeholders::_1));
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Check if the button is pressed (e.g., button index 0)
        bool button_pressed = msg->buttons[0] == 1;

        // Determine the servo position based on the button press
        int servo_position = button_pressed ? 180 : 90;

        // Only send the command if the position has changed
        if (servo_position != last_position_)
        {
            last_position_ = servo_position;

            // Log the servo position (simulate sending it to a servo controller)
            RCLCPP_INFO(this->get_logger(), "Button %s! Setting servo position to: %d",
                        button_pressed ? "pressed" : "released", servo_position);
        }
    }

    // Last known servo position to avoid redundant commands
    int last_position_;

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
