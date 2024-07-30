/**
 * @file trajectory_controller.h
 * @author RinYoshida
 * @brief ロボットの軌跡制御をするためのクラス
 * @date 2024-07-30
 */

#ifndef TRAJECTORY_CONTROLLER_H
#define TRAJECTORY_CONTROLLER_H

#include <cstdint>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "robocup_ssl_msgs/msg/detection_robot.hpp"
#include "robocup_ssl_msgs/msg/commands.hpp"
#include "robocup_ssl_msgs/msg/robot_command.hpp"

namespace ssl_robot_controller
{
class TrajectoryController{
public:
    TrajectoryController();
    ~TrajectoryController();

    int8_t TrajectoryControl();
private:
    rclcpp::Node::SharedPtr trajectory_controller_node_;
    rclcpp::Publisher<robocup_ssl_msgs::msg::Commands>::SharedPtr robot_command_publisher_;
    rclcpp::Subscription<robocup_ssl_msgs::msg::DetectionRobot>::SharedPtr robot_detection_subscription_;

    robocup_ssl_msgs::msg::DetectionRobot current_odomery_;
    std::mutex mutex_;

    void OdometryCallback(const robocup_ssl_msgs::msg::DetectionRobot::SharedPtr msg);
};
}
#endif  // TRAJECTORY_CONTROLLER_H
