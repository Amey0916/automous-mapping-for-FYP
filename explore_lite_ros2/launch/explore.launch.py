from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包路径
    explore_dir = get_package_share_directory('explore_lite_ros2')
    # 加载参数文件
    params_file = "/home/fu/ros2_ws/src/explore_lite_ros2/config/explore_params.yaml"
    
    # 适配TB3仿真的参数
    explore_node = Node(
        package="explore_lite_ros2",
        executable="explore_node",
        name="explore_lite_node",
        output="screen",
        parameters=[params_file],  # 改用参数文件
    )

    # 延迟启动（确保Nav2完全加载，TB3仿真建议延迟60s）
    delayed_explore = TimerAction(
        period=60.0,
        actions=[
            LogInfo(msg="=== Starting Explore Lite Node for TB3 SLAM ==="),
            explore_node
        ]
    )

    return LaunchDescription([
        delayed_explore,
    ])
