/**
 * @file robot_controller_component.cc
 * @author RinYoshida (tororo1219@gmail.com)
 * @brief ロボットの制御系の通信周りのクラス実装
 * @date 2024-07-26
 */

#include <cstdio>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robocup_ssl_msgs/msg/detection_robot.hpp"
#include "robocup_ssl_msgs/msg/commands.hpp"
#include "robocup_ssl_msgs/msg/robot_command.hpp"
#include "ssl_robot_controller/robot_controller_component.h"
#include "ssl_robot_controller_interfaces/action/trajectory_control.hpp"

namespace ssl_robot_controller
{
RobotController::RobotController(const rclcpp::NodeOptions & options)
    : Node("robot_controller_component", options)
{
    robot_command_publisher_ = this->create_publisher<robocup_ssl_msgs::msg::Commands>("commands", 10);
    robot_detection_subscription_ = this->create_subscription<robocup_ssl_msgs::msg::DetectionRobot>(
        "robot", 10, std::bind(&RobotController::OdometryCallback, this, std::placeholders::_1));


    action_server_ = rclcpp_action::create_server<ssl_robot_controller_interfaces::action::TrajectoryControl>(
        this,
        "robot_controller_component",
        std::bind(&RobotController::TrajectoryControlGoalResponse, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&RobotController::TrajectoryControlCancelRsponse, this, std::placeholders::_1),
        std::bind(&RobotController::TrajectoryControlAcceptedResponse, this, std::placeholders::_1));

    trajectory_controller = std::make_shared<TrajectoryController>(robot_command_publisher_, current_odomery_, &mutex_);
}

void RobotController::OdometryCallback(const robocup_ssl_msgs::msg::DetectionRobot::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    RCLCPP_INFO(this->get_logger(), "OdometryCallback");
    current_odomery_ = msg;
}

rclcpp_action::GoalResponse RobotController::TrajectoryControlGoalResponse(
    const rclcpp_action::GoalUUID & uuid,
    [[maybe_unused]]std::shared_ptr<const ssl_robot_controller_interfaces::action::TrajectoryControl::Goal> goal)
{
    //trajectory_controller.TrajectoryControl();
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %f", goal->x_axis);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotController::TrajectoryControlCancelRsponse(
    [[maybe_unused]]const std::shared_ptr<rclcpp_action::ServerGoalHandle<ssl_robot_controller_interfaces::action::TrajectoryControl>> goal_handle)
{
    //RCLCPP_INFO(this->get_logger(), "Received request to cancel goal with order %d", goal_handle->get_goal()->order);
    return rclcpp_action::CancelResponse::ACCEPT;
}
void RobotController::TrajectoryControlAcceptedResponse(
    [[maybe_unused]]const std::shared_ptr<rclcpp_action::ServerGoalHandle<ssl_robot_controller_interfaces::action::TrajectoryControl>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Goal has been accepted with order %d", goal_handle->get_goal()->y_axis);
    trajectory_controller->TrajectoryControl();
    
}

} // namespace ssl_robot_controller

RCLCPP_COMPONENTS_REGISTER_NODE(ssl_robot_controller::RobotController)