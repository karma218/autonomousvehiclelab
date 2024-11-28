#ifndef ARDUINO_CAR_HARDWARE_H
#define ARDUINO_CAR_HARDWARE_H

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "arduino_comms.h"
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_status_values.hpp>


class ArduinoCarHardware : public hardware_interface::SystemInterface {
public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override;
    hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
    hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
    ArduinoComms arduino_comms_;
    double velocity_command_ = 0.0;  // Command: forward velocity
    double steering_command_ = 0.0; // Command: steering angle
    double velocity_state_ = 0.0;   // State: current velocity
    double steering_state_ = 0.0;  // State: current steering angle
};

#endif // ARDUINO_CAR_HARDWARE_H
