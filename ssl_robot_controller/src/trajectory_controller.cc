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

int8_t TrajectoryController::TrajectoryControl(uint8_t robot_id ,int32_t target_x_axis, int32_t target_y_axis){
    int32_t current_x_axis = 0;
    int32_t current_y_axis = 0;
    std::lock_guard<std::mutex> lock(*mutex_);
    while(true){ // TODO (RinYoshida) 何故か1度でgrsimから来るdetectionが受け取れないので、受け取れるまで回実装になっているので、原因を解明する。
        std::lock_guard<std::mutex> lock(*mutex_);
        if(current_odomery_->robot_id == robot_id){
            current_x_axis = current_odomery_->x;
            current_y_axis = current_odomery_->y;
        }
        else{
            lock.unlock();
            continue;
        }
    }
    x_vector = target_x_axis - current_x_axis;
    y_vector = target_y_axis - current_y_axis;
    double theta = atan2(y_vector, x_vector);
    double distance = sqrt(pow(x_vector, 2) + pow(y_vector, 2));
    theta = fsin(theta); // 時計回りに0~π　半時計回りに0~-π
    if(theta)
    


    
    
    


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
