#include "bicycle_controller.h"

controller_interface::CallbackReturn BicycleController::on_init() {
    RCLCPP_INFO(rclcpp::get_logger("BicycleController"), "Initializing BicycleController...");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BicycleController::on_configure(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_node()->get_logger(), "Configuring BicycleController...");

    // Subscribe to /cmd_vel topic
    cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&BicycleController::cmdVelCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_node()->get_logger(), "Subscribed to /cmd_vel topic.");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BicycleController::on_activate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_node()->get_logger(), "Activating BicycleController...");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BicycleController::on_deactivate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_node()->get_logger(), "Deactivating BicycleController...");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BicycleController::on_cleanup(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_node()->get_logger(), "Cleaning up BicycleController...");
    cmd_vel_sub_.reset(); // Release subscription
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BicycleController::on_error(const rclcpp_lifecycle::State &) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error in BicycleController!");
    return controller_interface::CallbackReturn::ERROR;
}

controller_interface::CallbackReturn BicycleController::on_shutdown(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_node()->get_logger(), "Shutting down BicycleController...");
    return controller_interface::CallbackReturn::SUCCESS;
}

void BicycleController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Extract velocity and steering angle from Twist message
    target_velocity_ = msg->linear.x;
    target_steering_angle_ = msg->angular.z;

    RCLCPP_DEBUG(get_node()->get_logger(), "Received cmd_vel - Velocity: %f, Steering: %f",
                 target_velocity_, target_steering_angle_);
}

controller_interface::return_type BicycleController::update(const rclcpp::Time &time, const rclcpp::Duration &period) {
    auto command_interfaces = get_command_interfaces();

    // Ensure there are enough command interfaces available
    if (command_interfaces.size() < 2) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Insufficient command interfaces! Expected 2, but got %zu.",
                     command_interfaces.size());
        return controller_interface::return_type::ERROR;
    }

    // Update command interfaces with target velocity and steering angle
    command_interfaces[0].set_value(target_velocity_);
    command_interfaces[1].set_value(target_steering_angle_);

    RCLCPP_DEBUG(get_node()->get_logger(),
                 "Updated command interfaces - Velocity: %f, Steering: %f",
                 target_velocity_, target_steering_angle_);
    return controller_interface::return_type::OK;
}
