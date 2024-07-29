import launch 
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='ssl_robot_controller',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='ssl_robot_controller',
                plugin='ssl_robot_controller::TrajectoryControl',
                name='trajectory_controller',
            ),
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])        