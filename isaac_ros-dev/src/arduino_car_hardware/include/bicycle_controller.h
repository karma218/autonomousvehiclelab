#ifndef BICYCLE_CONTROLLER_H
#define BICYCLE_CONTROLLER_H

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

class BicycleController : public controller_interface::ControllerInterface {
public:
    BicycleController() = default;

    // Lifecycle methods
    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    double target_velocity_ = 0.0;        // Target velocity command
    double target_steering_angle_ = 0.0; // Target steering angle command
};

#endif // BICYCLE_CONTROLLER_H
