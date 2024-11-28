#include "arduino_car_hardware.h"
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_status_values.hpp>


CallbackReturn ArduinoCarHardware::on_init(const hardware_interface::HardwareInfo &info) {
    RCLCPP_INFO(rclcpp::get_logger("ArduinoCarHardware"), "Initializing hardware interface...");

    // Retrieve hardware parameters
    const std::string port = info.hardware_parameters["serial_device"];
    const int baud_rate = std::stoi(info.hardware_parameters["baud_rate"]);
    const int timeout_ms = std::stoi(info.hardware_parameters["timeout_ms"]);

    try {
        // Initialize Arduino communication
        arduino_comms_.setup(port, baud_rate, timeout_ms);

        if (!arduino_comms_.connected()) {
            RCLCPP_ERROR(rclcpp::get_logger("ArduinoCarHardware"),
                         "Failed to connect to Arduino on port: %s", port.c_str());
            return CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("ArduinoCarHardware"),
                    "Successfully connected to Arduino on port: %s", port.c_str());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("ArduinoCarHardware"),
                     "Exception during Arduino setup: %s", e.what());
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArduinoCarHardware::export_state_interfaces() {
    RCLCPP_INFO(rclcpp::get_logger("ArduinoCarHardware"), "Exporting state interfaces...");
    return {
        hardware_interface::StateInterface("wheel", "velocity", &velocity_state_),
        hardware_interface::StateInterface("steering", "position", &steering_state_)
    };
}

std::vector<hardware_interface::CommandInterface> ArduinoCarHardware::export_command_interfaces() {
    RCLCPP_INFO(rclcpp::get_logger("ArduinoCarHardware"), "Exporting command interfaces...");
    return {
        hardware_interface::CommandInterface("wheel", "velocity", &velocity_command_),
        hardware_interface::CommandInterface("steering", "position", &steering_command_)
    };
}

CallbackReturn ArduinoCarHardware::on_activate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(rclcpp::get_logger("ArduinoCarHardware"), "Activating hardware interface...");
    return CallbackReturn::SUCCESS;
}

CallbackReturn ArduinoCarHardware::on_deactivate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(rclcpp::get_logger("ArduinoCarHardware"), "Deactivating hardware interface...");
    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArduinoCarHardware::read(const rclcpp::Time &, const rclcpp::Duration &) {
    try {
        // Read sensor data from Arduino
        arduino_comms_.getVelocityAndSteerValues();

        // Update state variables
        velocity_state_ = (arduino_comms_.left_wheel_vel + arduino_comms_.right_wheel_vel) / 2.0;
        steering_state_ = arduino_comms_.steering_angle;

        RCLCPP_DEBUG(rclcpp::get_logger("ArduinoCarHardware"),
                     "Read values - Velocity: %f, Steering: %f", velocity_state_, steering_state_);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("ArduinoCarHardware"),
                     "Error reading from Arduino: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArduinoCarHardware::write(const rclcpp::Time &, const rclcpp::Duration &) {
    try {
        // Write commands to Arduino
        arduino_comms_.setMotorValues(velocity_command_, steering_command_);

        RCLCPP_DEBUG(rclcpp::get_logger("ArduinoCarHardware"),
                     "Sent commands - Velocity: %f, Steering: %f", velocity_command_, steering_command_);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("ArduinoCarHardware"),
                     "Error writing to Arduino: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
}
