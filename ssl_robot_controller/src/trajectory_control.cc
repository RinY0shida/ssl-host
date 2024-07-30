/**
 * @file trajectory_control.cc
 * @author RinYoshida (tororo1219@gmail.com)
 * @brief ロボットの軌跡制御のクラス実装
 * @date 2024-07-26
 */

#include <cstdio>
#include <memory>
#include <thread>

#include "ssl_robot_controller/trajectory_control.h"
#include "ssl_robot_controller_interfaces/action/trajectory_control.hpp"

namespace ssl_robot_controller
{
TrajectoryControl::TrajectoryControl(const rclcpp::NodeOptions & options)
    : Node("trajectory_control", options)
{
    action_server_ = rclcpp_action::create_server<ssl_robot_controller_interfaces::action::TrajectoryControl>(
        this,
        "trajectory_control",
        std::bind(&TrajectoryControl::TrajectoryControlGoalResponse, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&TrajectoryControl::TrajectoryControlCancelRsponse, this, std::placeholders::_1),
        std::bind(&TrajectoryControl::TrajectoryControlAcceptedResponse, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse TrajectoryControl::TrajectoryControlGoalResponse(
    const rclcpp_action::GoalUUID & uuid,
    [[maybe_unused]]std::shared_ptr<const ssl_robot_controller_interfaces::action::TrajectoryControl::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %f", goal->x_axis);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TrajectoryControl::TrajectoryControlCancelRsponse(
    [[maybe_unused]]const std::shared_ptr<rclcpp_action::ServerGoalHandle<ssl_robot_controller_interfaces::action::TrajectoryControl>> goal_handle)
{
    //RCLCPP_INFO(this->get_logger(), "Received request to cancel goal with order %d", goal_handle->get_goal()->order);
    return rclcpp_action::CancelResponse::ACCEPT;
}
void TrajectoryControl::TrajectoryControlAcceptedResponse(
    [[maybe_unused]]const std::shared_ptr<rclcpp_action::ServerGoalHandle<ssl_robot_controller_interfaces::action::TrajectoryControl>> goal_handle)
{
    //RCLCPP_INFO(this->get_logger(), "Goal has been accepted with order %d", goal_handle->get_goal()->y_axis);
}

} // namespace ssl_robot_controller

RCLCPP_COMPONENTS_REGISTER_NODE(ssl_robot_controller::TrajectoryControl)