#include <rclcpp/rclcpp.hpp>
#include <controller_manager/controller_manager.hpp>
#include "arduino_car_hardware/arduino_car_hardware.h"

int main(int argc, char **argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("arduino_car_node");

    // Declare parameters for serial communication
    std::string device;
    int baud_rate;
    node->declare_parameter<std::string>("device", "/dev/ttyACM0");
    node->declare_parameter<int>("baud_rate", 115200);

    node->get_parameter("device", device);
    node->get_parameter("baud_rate", baud_rate);

    // Create an instance of ArduinoCarHardware
    auto hardware = std::make_shared<ArduinoCarHardware>();

    // Initialize the hardware interface
    if (hardware->on_init({
            {"serial_device", device},
            {"baud_rate", std::to_string(baud_rate)},
            {"timeout_ms", "100"}
        }) != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize ArduinoCarHardware.");
        rclcpp::shutdown();
        return 1;
    }

    // Create and initialize the controller manager
    controller_manager::ControllerManager cm(hardware, node);

    // Main control loop at 10 Hz
    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok()) {
        // Perform hardware read-write cycle
        if (hardware->read(rclcpp::Clock().now(), rclcpp::Duration::from_seconds(0.1)) != hardware_interface::return_type::OK) {
            RCLCPP_WARN(node->get_logger(), "Hardware read failed.");
        }

        cm.update(rclcpp::Clock().now(), rclcpp::Duration::from_seconds(0.1));

        if (hardware->write(rclcpp::Clock().now(), rclcpp::Duration::from_seconds(0.1)) != hardware_interface::return_type::OK) {
            RCLCPP_WARN(node->get_logger(), "Hardware write failed.");
        }

        loop_rate.sleep();
    }

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
