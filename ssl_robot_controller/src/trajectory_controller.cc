/**
 * @file trajectory_controller.cc
 * @author RinYoshida
 * @brief ロボットの軌跡制御をするためのクラス実装
 * @date 2024-07-31
 */

#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

#include "ssl_robot_controller/trajectory_controller.h"
namespace ssl_robot_controller
{
TrajectoryController::TrajectoryController(){
    trajectory_controller_node_ = rclcpp::Node::make_shared("trajectory_controller");
    robot_command_publisher_ = trajectory_controller_node_->create_publisher<robocup_ssl_msgs::msg::Commands>("robot_command", 10);
    robot_detection_subscription_ = trajectory_controller_node_->create_subscription<robocup_ssl_msgs::msg::DetectionRobot>(
        "detection_robot", 10, std::bind(&TrajectoryController::OdometryCallback, this, std::placeholders::_1));
}

TrajectoryController::~TrajectoryController(){
}

void TrajectoryController::OdometryCallback(const robocup_ssl_msgs::msg::DetectionRobot::SharedPtr msg){
    std::lock_guard<std::mutex> lock(mutex_);
    RCLCPP_INFO(trajectory_controller_node_->get_logger(), "OdometryCallback");
    current_odomery_ = *msg;
}

int8_t TrajectoryController::TrajectoryControl(){
    // auto msg = robocup_ssl_msgs::msg::Commands();
    // msg.isteamyellow = false;
    // msg.isteamyellow = true;
    // robocup_ssl_msgs::msg::RobotCommand command;
    // command.id = 9;
    // command.wheelsspeed = true;
    // command.wheel1[0] = 1.0;
    // command.wheel2[0] = 1.0;
    // command.wheel3[0] = -1.0;
    // command.wheel4[0] = -1.0;
    // msg.robot_commands.push_back(command);
    // robot_command_publisher_->publish(msg);
    auto message = robocup_ssl_msgs::msg::Commands();
        message.timestamp = 0;
        message.isteamyellow = false;
        robocup_ssl_msgs::msg::RobotCommand command;
        command.id = 0;
        command.wheelsspeed = true;
        command.wheel1.resize(1);
        command.wheel2.resize(1);
        command.wheel3.resize(1);
        command.wheel4.resize(1);
        command.wheel1[0] = 1.0;
        command.wheel2[0] = 1.0;
        command.wheel3[0] = -1.0;
        command.wheel4[0] = -1.0;
        // 他のフィールドも必要に応じて初期化できます
        message.robot_commands.push_back(command);
        robot_command_publisher_->publish(message);
        RCLCPP_INFO(trajectory_controller_node_->get_logger(), "TrajectoryControl");
    return 0;
}
} // namespace ssl_robot_controller
