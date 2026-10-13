"""Phần 3 + 4: AMCL + map_server (load bản đồ đã lưu), Nav2 và RViz2.

Cần sim đang chạy. Sau khi RViz lên: đặt vị trí ban đầu (2D Pose Estimate hoặc
./scripts/set_initial_pose.sh 0 0 0), rồi đặt Nav2 Goal.
    ros2 launch launch/navigation.launch.py
    ros2 launch launch/navigation.launch.py map:=/duong/dan/map.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))


def generate_launch_description():
    nav = get_package_share_directory('turtlebot4_navigation')
    viz = get_package_share_directory('turtlebot4_viz')
    map_arg = DeclareLaunchArgument('map', default_value=os.path.join(REPO, 'maps', 'map.yaml'),
                                    description='Đường dẫn map.yaml (map.pgm cùng thư mục)')
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav, 'launch', 'localization.launch.py')),
        launch_arguments={'use_sim_time': 'true', 'map': LaunchConfiguration('map')}.items())
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav, 'launch', 'nav2.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items())
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(viz, 'launch', 'view_robot.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items())
    return LaunchDescription([map_arg, localization,
                              TimerAction(period=10.0, actions=[nav2]),
                              TimerAction(period=5.0, actions=[rviz])])
