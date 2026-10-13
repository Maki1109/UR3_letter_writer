"""Phần 1: TurtleBot 4 trong Ignition Gazebo, world 'room' (phòng nhỏ 7 x 5 m, sinh bởi scripts/make_room_world.py).

Chạy từ terminal đã `source scripts/env.sh` (nạp overlay + sửa lỗi môi trường):
    ros2 launch launch/sim.launch.py            # world mặc định: room
    ros2 launch launch/sim.launch.py world:=warehouse
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    bringup = get_package_share_directory('turtlebot4_ignition_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='room',
                              description='room | warehouse | depot | maze'),
        DeclareLaunchArgument('model', default_value='standard', description='standard | lite'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'turtlebot4_ignition.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world'),
                              'model': LaunchConfiguration('model'),
                              'rviz': 'false'}.items()),
    ])
