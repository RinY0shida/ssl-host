/**
 * @file trajectory_controller.h
 * @author RinYoshida (tororo1219@gmail.com)
 * @brief ロボットの軌跡制御をするためのクラス
 * @date 2024-07-30
 */

#ifndef TRAJECTORY_CONTROLLER_H
#define TRAJECTORY_CONTROLLER_H

#include <cstdint>
#include <mutex>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "robocup_ssl_msgs/msg/detection_robot.hpp"
#include "robocup_ssl_msgs/msg/commands.hpp"
#include "robocup_ssl_msgs/msg/robot_command.hpp"

namespace ssl_robot_controller
{
class TrajectoryController{
public:
    TrajectoryController(rclcpp::Publisher<robocup_ssl_msgs::msg::Commands>::SharedPtr robot_command_publisher,
                         std::shared_ptr<robocup_ssl_msgs::msg::DetectionRobot> current_odomery,
                         std::mutex *mutex);
    ~TrajectoryController();

    int8_t TrajectoryControl();
private:
    rclcpp::Publisher<robocup_ssl_msgs::msg::Commands>::SharedPtr robot_command_publisher_;
    std::shared_ptr<robocup_ssl_msgs::msg::DetectionRobot> current_odomery_;
    std::mutex *mutex_;
};
}
#endif  // TRAJECTORY_CONTROLLER_H
