cmake_minimum_required(VERSION 3.5)
project(arduino_car_hardware)

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)


# Add source files
add_library(arduino_car_hardware
    src/arduino_car_hardware.cpp
    src/arduino_comms.cpp
)

add_library(bicycle_controller
    src/bicycle_controller.cpp
)

# Link dependencies
ament_target_dependencies(arduino_car_hardware
    rclcpp
    hardware_interface
    pluginlib
)

ament_target_dependencies(bicycle_controller
    rclcpp
    controller_interface
    geometry_msgs
)

# Export plugins for ros2_control
pluginlib_export_plugin_description_file(hardware_interface share/arduino_car_hardware_plugins.xml)
pluginlib_export_plugin_description_file(controller_interface share/bicycle_controller_plugins.xml)

# Install headers
install(
    DIRECTORY include/
    DESTINATION include
)

# Install libraries
install(
    TARGETS arduino_car_hardware bicycle_controller
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install plugin XML files
install(
    FILES
        share/arduino_car_hardware_plugins.xml
        share/bicycle_controller_plugins.xml
    DESTINATION share/${PROJECT_NAME}
)


# Install config and launch files
install(
    DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
