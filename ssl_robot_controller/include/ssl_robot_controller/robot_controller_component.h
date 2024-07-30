/**
 * @file robot_controller_component.h
 * @author RinYoshida (tororo1219@gmail.com)
 * @brief ロボットの軌跡制御のクラスヘッダ
 * @version 0.1
 * @date 2024-07-26
 */

#ifndef TRAJECTORY_CONTROL_H
#define TRAJECTORY_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robocup_ssl_msgs/msg/detection_robot.hpp"
#include "robocup_ssl_msgs/msg/commands.hpp"
#include "robocup_ssl_msgs/msg/robot_command.hpp"
#include "ssl_robot_controller_interfaces/action/trajectory_control.hpp"


namespace ssl_robot_controller
{

class RobotController : public rclcpp::Node
{
public:
    explicit RobotController(const rclcpp::NodeOptions & options);

private:
    rclcpp_action::Server<ssl_robot_controller_interfaces::action::TrajectoryControl>::SharedPtr action_server_;

    rclcpp_action::GoalResponse TrajectoryControlGoalResponse(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ssl_robot_controller_interfaces::action::TrajectoryControl::Goal> goal);

    rclcpp_action::CancelResponse TrajectoryControlCancelRsponse(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ssl_robot_controller_interfaces::action::TrajectoryControl>> goal_handle);

    void TrajectoryControlAcceptedResponse(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ssl_robot_controller_interfaces::action::TrajectoryControl>> goal_handle);
};

}  // namespace ssl_robot_controller
#endif  // TRAJECTORY_CONTROL_H
