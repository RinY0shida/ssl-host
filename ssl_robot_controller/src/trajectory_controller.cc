/**
 * @file trajectory_controller.cc
 * @author RinYoshida
 * @brief ロボットの軌跡制御をするためのクラス実装
 * @date 2024-07-31
 */

#include <cstdio>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include "ssl_robot_controller/trajectory_controller.h"

namespace ssl_robot_controller
{
TrajectoryController::TrajectoryController(rclcpp::Publisher<robocup_ssl_msgs::msg::Commands>::SharedPtr robot_command_publisher,
                                           std::shared_ptr<robocup_ssl_msgs::msg::DetectionRobot> current_odomery,
                                           std::mutex *mutex)
                                            : robot_command_publisher_(robot_command_publisher),
                                              current_odomery_(current_odomery), 
                                              mutex_(mutex){
}

TrajectoryController::~TrajectoryController(){
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
        //RCLCPP::INFO(this->get_logger(), "TrajectoryControl");
    return 0;
}
} // namespace ssl_robot_controller
