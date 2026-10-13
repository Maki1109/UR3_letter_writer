"""Phần 2: SLAM Toolbox (config/slam.yaml) + RViz2. Lái robot bằng: ./scripts/teleop.sh

Cần sim đang chạy (ros2 launch launch/sim.launch.py). Lưu bản đồ: ./scripts/save_map.sh
    ros2 launch launch/mapping.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))


def generate_launch_description():
    nav = get_package_share_directory('turtlebot4_navigation')
    viz = get_package_share_directory('turtlebot4_viz')
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav, 'launch', 'slam.launch.py')),
        launch_arguments={'use_sim_time': 'true',
                          'params': os.path.join(REPO, 'config', 'slam.yaml')}.items())
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(viz, 'launch', 'view_robot.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items())
    return LaunchDescription([slam, TimerAction(period=5.0, actions=[rviz])])
