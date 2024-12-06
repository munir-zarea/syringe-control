cmake_minimum_required(VERSION 3.5)
project(ais_arduino_step_motor_joy_command_receiver)

if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(joy REQUIRED)
find_package(serial REQUIRED)  # Ensure serial is found as a package
find_package(std_msgs REQUIRED)

# Add the executable for your node
add_executable(servo_controller src/servo_controller.cpp)

# Add include directories explicitly
target_include_directories(servo_controller PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${serial_INCLUDE_DIRS}  # Include directories from the serial package
)

# Link libraries to the executable
ament_target_dependencies(servo_controller rclcpp sensor_msgs joy serial std_msgs)

# Install the executable
install(TARGETS
  servo_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Include launch files (remove if not used)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Finalize the package
ament_package()
