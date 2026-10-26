"""Bài 06: Gazebo (world open_arena) + robot_behavior_node + gesture_recognition_node trong một lệnh.

Chạy từ terminal đã `source btth6/scripts/env.sh` (nạp overlay, world open_arena, .pydeps có mediapipe + numpy<2):
    ros2 launch btth6/launch/gesture_control.launch.py
    ros2 launch btth6/launch/gesture_control.launch.py sim:=false camera:=1   # sim đã chạy sẵn, webcam 1

Hai node là script Python (không phải gói colcon) nên được chạy bằng ExecuteProcess với /usr/bin/python3.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

B6_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
PY = '/usr/bin/python3'


def node(script, *ros_args):
    return ExecuteProcess(
        cmd=[PY, os.path.join(B6_DIR, 'nodes', script), '--ros-args', *ros_args],
        additional_env={'PYTHONPATH': os.path.join(B6_DIR, '.pydeps') + os.pathsep + os.environ.get('PYTHONPATH', ''),
                        'GLOG_minloglevel': '2'},
        output='screen')


def generate_launch_description():
    sim = LaunchConfiguration('sim')
    bringup = get_package_share_directory('turtlebot4_ignition_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true', description='khởi động Gazebo + TurtleBot 4'),
        DeclareLaunchArgument('camera', default_value='0', description='chỉ số webcam'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'turtlebot4_ignition.launch.py')),
            launch_arguments={'world': 'open_arena', 'model': 'standard', 'rviz': 'false'}.items(),
            condition=IfCondition(sim)),
        # behavior node tự đợi /dock_status rồi undock, nên có thể chạy ngay cùng Gazebo
        node('robot_behavior_node.py'),
        TimerAction(period=5.0, actions=[node('gesture_recognition_node.py', '-p',
                                              ['camera:=', LaunchConfiguration('camera')])]),
    ])
