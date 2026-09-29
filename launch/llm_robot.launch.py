"""llm_robot.launch.py — khởi động toàn bộ hệ thống bài thực hành 02.

  Gazebo Sim (gz-sim / Ignition Fortress, qua ros_gz — world có plugin
    physics + user-commands, cho phép "/world/.../set_pose")
    + UR3e (ros2_control qua plugin ign_ros2_control nhúng trong URDF)
    + bàn, 3 khối, 4 vùng (spawn từ config/scene.yaml bằng "ros_gz_sim create")
  MoveIt 2 (ur_moveit_config, use_sim_time) + RViz
  [tuỳ chọn] llm_commander trong cửa sổ xterm riêng (start_commander:=true)

Phần robot bringup tương đương ur_simulation_gz/ur_sim_control.launch.py
(bản Humble), chỉ khác là truyền được world file riêng — launch gốc của UR
chỉ nhận tên/đường dẫn world, không spawn thêm bàn/khối/vùng.

Chạy:
  ros2 launch ur3_llm_control llm_robot.launch.py
  ros2 launch ur3_llm_control llm_robot.launch.py ur_type:=ur3 start_commander:=true
"""
import os

import yaml
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from ur3_llm_control.gazebo_bridge import DEFAULT_WORLD_NAME

PKG = "ur3_llm_control"


# ---------------------------------------------------------------------------
# SDF cho bàn / khối / vùng (sinh từ scene.yaml) — định dạng gz-sim (SDF 1.9)
# ---------------------------------------------------------------------------
def _box_sdf(name, size, rgba, static=True, kinematic=False, collision=True):
    sx, sy, sz = size
    color = " ".join(str(c) for c in rgba)
    geom = f"<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
    coll = f"<collision name='collision'>{geom}</collision>" if collision else ""
    inertial = "" if static else (
        "<inertial><mass>0.05</mass><inertia><ixx>1.3e-5</ixx><iyy>1.3e-5</iyy>"
        "<izz>1.3e-5</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>")
    # kinematic + gravity=false: vật không rơi/không bị va chạm xô lệch trong gz-sim,
    # chỉ đổi chỗ khi gazebo_bridge.py gọi service "/world/.../set_pose".
    kin_tag = f"<kinematic>{'true' if kinematic else 'false'}</kinematic>"
    gravity_tag = "<gravity>false</gravity>" if kinematic else ""
    return f"""<?xml version='1.0'?>
<sdf version='1.9'>
  <model name='{name}'>
    <static>{'true' if static else 'false'}</static>
    <link name='link'>
      {kin_tag}
      {gravity_tag}
      {inertial}
      {coll}
      <visual name='visual'>{geom}
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""


def spawn_scene(context, *args, **kwargs):
    scene_path = LaunchConfiguration("scene_config").perform(context)
    with open(scene_path, "r", encoding="utf-8") as f:
        sc = yaml.safe_load(f)
    out_dir = "/tmp/ur3_llm_scene"
    os.makedirs(out_dir, exist_ok=True)

    table_c, table_s = sc["table"]["center"], sc["table"]["size"]
    top_z = table_c[2] + table_s[2] / 2.0
    cube = float(sc["cube_size"])
    zone_sz = float(sc.get("zone_size", 0.08))

    items = [("table", table_s, [0.55, 0.40, 0.25, 1.0], True, False, True, table_c)]
    for zname, z in sc["zones"].items():
        # tấm màu mỏng chỉ để nhìn (không collision)
        items.append((f"pad_{zname}", [zone_sz, zone_sz, 0.001], z.get("rgba", [0.6] * 3 + [1.0]),
                      True, False, False, [z["center"][0], z["center"][1], top_z + 0.0005]))
    for oname, o in sc["objects"].items():
        # khối kinematic: đứng yên, không bị xô đẩy; chỉ di chuyển khi node đặt pose
        items.append((oname, [cube] * 3, o["rgba"], False, True, True,
                      [o["start"][0], o["start"][1], top_z + cube / 2.0]))

    nodes = []
    for name, size, rgba, static, kin, coll, pos in items:
        path = os.path.join(out_dir, f"{name}.sdf")
        with open(path, "w") as f:
            f.write(_box_sdf(name, size, rgba, static, kin, coll))
        nodes.append(Node(
            package="ros_gz_sim", executable="create", output="screen",
            name=f"spawn_{name}",
            arguments=["-world", DEFAULT_WORLD_NAME, "-name", name, "-file", path,
                       "-x", str(pos[0]), "-y", str(pos[1]), "-z", str(pos[2])]))
    return nodes


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    launch_rviz = LaunchConfiguration("launch_rviz")
    gazebo_gui = LaunchConfiguration("gazebo_gui")

    args = [
        DeclareLaunchArgument("ur_type", default_value="ur3e",
                              choices=["ur3", "ur3e"], description="UR3 hoặc UR3e"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        DeclareLaunchArgument("start_commander", default_value="false",
                              description="Mở llm_commander trong xterm (cần: sudo apt install xterm)"),
        DeclareLaunchArgument("commander_delay", default_value="20.0",
                              description="Giây chờ Gazebo + MoveIt khởi động trước khi mở commander"),
        DeclareLaunchArgument("scene_config", default_value=PathJoinSubstitution(
            [FindPackageShare(PKG), "config", "scene.yaml"])),
    ]

    # ---------------- Robot description (giống ur_sim_control.launch.py của ur_simulation_gz) ----
    controllers = PathJoinSubstitution(
        [FindPackageShare("ur_simulation_gz"), "config", "ur_controllers.yaml"])
    initial_positions = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "initial_positions.yaml"])
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]), " ",
        "safety_limits:=true safety_pos_margin:=0.15 safety_k_position:=20 ",
        "name:=ur ur_type:=", ur_type, " prefix:=\"\" sim_ignition:=true ",
        "simulation_controllers:=", controllers, " ",
        "initial_positions_file:=", initial_positions,
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               output="both", parameters=[{"use_sim_time": True}, robot_description])

    world_path = PathJoinSubstitution([FindPackageShare(PKG), "worlds", "llm_table.world"])

    gazebo_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch", "/gz_sim.launch.py"]),
        launch_arguments={"gz_args": [" -r -v 3 ", world_path]}.items(),
        condition=IfCondition(gazebo_gui))
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch", "/gz_sim.launch.py"]),
        launch_arguments={"gz_args": [" -s -r -v 3 ", world_path]}.items(),
        condition=UnlessCondition(gazebo_gui))

    spawn_robot = Node(package="ros_gz_sim", executable="create", name="spawn_ur", output="screen",
                       arguments=["-world", DEFAULT_WORLD_NAME, "-name", "ur",
                                  "-string", robot_description_content, "-allow_renaming", "false"])

    # /clock (sim time) + service "set_pose" dùng bởi gazebo_bridge.py (tay gắp ảo)
    clock_bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"])
    pose_bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                       arguments=[f"/world/{DEFAULT_WORLD_NAME}/set_pose@ros_gz_interfaces/srv/SetEntityPose"])

    jsb = Node(package="controller_manager", executable="spawner",
               arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"])
    jtc = Node(package="controller_manager", executable="spawner",
               arguments=["joint_trajectory_controller", "-c", "/controller_manager"])

    # ---------------- MoveIt 2 (dùng lại ur_moveit_config) ----------------
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_moveit_config"), "/launch", "/ur_moveit.launch.py"]),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": "true",
            "use_sim_time": "true",
            "launch_rviz": launch_rviz,
        }.items())

    # Spawn bàn/khối/vùng sau khi robot đã được spawn xong
    scene_after_robot = RegisterEventHandler(OnProcessExit(
        target_action=spawn_robot, on_exit=[OpaqueFunction(function=spawn_scene)]))

    commander = TimerAction(
        period=LaunchConfiguration("commander_delay"),
        condition=IfCondition(LaunchConfiguration("start_commander")),
        actions=[Node(package=PKG, executable="llm_commander", name="llm_commander",
                      output="screen", prefix="xterm -fa Monospace -fs 11 -geometry 110x45 -e",
                      parameters=[{"use_sim_time": True}])])

    return LaunchDescription(args + [
        rsp, gazebo_with_gui, gazebo_headless, spawn_robot, clock_bridge, pose_bridge,
        jsb, jtc, scene_after_robot, moveit, commander,
    ])
