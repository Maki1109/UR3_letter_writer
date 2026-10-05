"""llm_robot.launch.py — khởi động toàn bộ hệ thống bài thực hành 03.

  Gazebo Sim (gz-sim / Ignition Fortress, qua ros_gz)
    + UR3e có TAY GẮP song song 2 ngón (urdf/ur_with_gripper.urdf.xacro),
      tay máy và tay gắp điều khiển bằng ros2_control (plugin ign_ros2_control)
    + bàn, 5 khối (vật động lực học thật: có khối lượng, trọng lực, ma sát),
      3 tấm zone, 1 CAMERA RGB nhìn xuống bàn — tất cả sinh từ config/scene.yaml
  ros_gz_bridge: /clock, ảnh + camera_info của camera
  Controllers: joint_state_broadcaster, joint_trajectory_controller (tay máy),
               gripper_controller (2 ngón)
  MoveIt 2 move_group (URDF có tay gắp + SRDF bổ sung) + RViz
  [tuỳ chọn] llm_commander trong cửa sổ xterm riêng (start_commander:=true)

Không có cầu nối "set_pose": chương trình KHÔNG được đổi pose của vật; khối chỉ
di chuyển khi bị tay gắp kẹp (ma sát) và mang đi.

Chạy:
  ros2 launch ur3_llm_control llm_robot.launch.py
  ros2 launch ur3_llm_control llm_robot.launch.py gazebo_gui:=false start_commander:=true
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

PKG = "ur3_llm_control"
WORLD_NAME = "llm_table_world"   # phải khớp <world name="..."> trong worlds/llm_table.world


def _load_yaml(package, rel_path):
    from ament_index_python.packages import get_package_share_directory
    with open(os.path.join(get_package_share_directory(package), rel_path), "r") as f:
        return yaml.safe_load(f)


# ---------------------------------------------------------------------------
# SDF cho bàn / zone / khối / camera (sinh từ scene.yaml) — gz-sim (SDF 1.9)
# ---------------------------------------------------------------------------
def _static_box_sdf(name, size, rgba, collision=True):
    sx, sy, sz = size
    color = " ".join(str(c) for c in rgba)
    geom = f"<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
    coll = (f"<collision name='collision'>{geom}<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2>"
            f"</ode></friction></surface></collision>") if collision else ""
    return f"""<?xml version='1.0'?>
<sdf version='1.9'>
  <model name='{name}'>
    <static>true</static>
    <link name='link'>
      {coll}
      <visual name='visual'>{geom}
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""


def _cube_sdf(name, edge, rgba, mass, mu):
    """Khối động lực học: có quán tính, trọng lực, ma sát -> chỉ di chuyển khi bị gắp/đẩy."""
    color = " ".join(str(c) for c in rgba)
    i = mass * edge * edge / 6.0
    geom = f"<geometry><box><size>{edge} {edge} {edge}</size></box></geometry>"
    return f"""<?xml version='1.0'?>
<sdf version='1.9'>
  <model name='{name}'>
    <static>false</static>
    <link name='link'>
      <inertial><mass>{mass}</mass><inertia><ixx>{i}</ixx><iyy>{i}</iyy><izz>{i}</izz>
        <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <collision name='collision'>{geom}
        <surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>
      </collision>
      <visual name='visual'>{geom}
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""


def _camera_sdf(cam):
    return f"""<?xml version='1.0'?>
<sdf version='1.9'>
  <model name='{cam["name"]}'>
    <static>true</static>
    <link name='link'>
      <sensor name='{cam["name"]}' type='camera'>
        <topic>{cam["image_topic"].lstrip("/")}</topic>
        <update_rate>{cam.get("update_rate", 10)}</update_rate>
        <always_on>1</always_on>
        <camera>
          <horizontal_fov>{cam["horizontal_fov"]}</horizontal_fov>
          <image><width>{cam["width"]}</width><height>{cam["height"]}</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>5.0</far></clip>
        </camera>
      </sensor>
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

    # (tên, sdf, [x, y, z, yaw])
    items = [("table", _static_box_sdf("table", table_s, sc["table"].get("rgba", [0.8, 0.78, 0.72, 1])),
              [*table_c, 0.0])]
    for zname, z in sc["zones"].items():
        # tấm mỏng chỉ để nhìn (không collision)
        items.append((f"pad_{zname}",
                      _static_box_sdf(f"pad_{zname}", [zone_sz, zone_sz, 0.001], z["rgba"], False),
                      [z["center"][0], z["center"][1], top_z + 0.0005, 0.0]))
    for oname, o in sc["objects"].items():
        x, y, yaw = (list(o["spawn"]) + [0.0])[:3]
        items.append((oname, _cube_sdf(oname, cube, o["rgba"], float(sc.get("cube_mass", 0.05)),
                                       float(sc.get("cube_friction", 1.0))),
                      [x, y, top_z + cube / 2.0 + 0.001, yaw]))

    nodes = []
    for name, sdf, (x, y, z, yaw) in items:
        path = os.path.join(out_dir, f"{name}.sdf")
        with open(path, "w") as f:
            f.write(sdf)
        nodes.append(Node(
            package="ros_gz_sim", executable="create", output="screen", name=f"spawn_{name}",
            arguments=["-world", WORLD_NAME, "-name", name, "-file", path,
                       "-x", str(x), "-y", str(y), "-z", str(z), "-Y", str(yaw)]))

    cam = sc["camera"]
    path = os.path.join(out_dir, "camera.sdf")
    with open(path, "w") as f:
        f.write(_camera_sdf(cam))
    cx, cy, cz, cr, cp, cyaw = cam["pose"]
    nodes.append(Node(
        package="ros_gz_sim", executable="create", output="screen", name="spawn_camera",
        arguments=["-world", WORLD_NAME, "-name", cam["name"], "-file", path,
                   "-x", str(cx), "-y", str(cy), "-z", str(cz),
                   "-R", str(cr), "-P", str(cp), "-Y", str(cyaw)]))
    # Cầu nối ảnh camera sang ROS 2
    nodes.append(Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen", name="camera_bridge",
        arguments=[f"{cam['image_topic']}@sensor_msgs/msg/Image[ignition.msgs.Image",
                   f"{cam['info_topic']}@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo"],
        parameters=[{"use_sim_time": True}]))
    return nodes


def moveit_nodes(context, *args, **kwargs):
    """move_group + RViz (tương đương ur_moveit_config/ur_moveit.launch.py) nhưng dùng
    URDF có tay gắp và SRDF bổ sung các cặp link tay gắp không cần kiểm tra va chạm."""
    ur_type = LaunchConfiguration("ur_type").perform(context)
    robot_description = {"robot_description": ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare(PKG), "urdf", "ur_with_gripper.urdf.xacro"]), " ",
        "safety_limits:=true safety_pos_margin:=0.15 safety_k_position:=20 ",
        f"name:=ur ur_type:={ur_type} prefix:=\"\" ",
    ]), value_type=str)}
    robot_description_semantic = {"robot_description_semantic": ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare(PKG), "srdf", "ur_with_gripper.srdf.xacro"]), " ",
        "name:=ur prefix:=\"\"",
    ]), value_type=str)}
    kinematics = _load_yaml("ur_moveit_config", "config/kinematics.yaml")["/**"]["ros__parameters"]
    planning = {"robot_description_planning": _load_yaml("ur_moveit_config", "config/joint_limits.yaml")}
    ompl = {"move_group": {
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization "
                            "default_planner_request_adapters/FixWorkspaceBounds "
                            "default_planner_request_adapters/FixStartStateBounds "
                            "default_planner_request_adapters/FixStartStateCollision "
                            "default_planner_request_adapters/FixStartStatePathConstraints",
        "start_state_max_bounds_error": 0.1,
    }}
    ompl["move_group"].update(_load_yaml("ur_moveit_config", "config/ompl_planning.yaml"))
    ctrl = _load_yaml("ur_moveit_config", "config/controllers.yaml")
    ctrl["scaled_joint_trajectory_controller"]["default"] = False
    ctrl["joint_trajectory_controller"]["default"] = True
    controllers = {"moveit_simple_controller_manager": ctrl,
                   "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"}
    execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    }
    psm = {"publish_planning_scene": True, "publish_geometry_updates": True,
           "publish_state_updates": True, "publish_transforms_updates": True}
    common = [robot_description, robot_description_semantic, kinematics, planning, ompl,
              {"use_sim_time": True}]
    move_group = Node(package="moveit_ros_move_group", executable="move_group", output="screen",
                      parameters=common + [execution, controllers, psm,
                                           {"publish_robot_description_semantic": True}])
    rviz = Node(package="rviz2", executable="rviz2", name="rviz2_moveit", output="log",
                condition=IfCondition(LaunchConfiguration("launch_rviz")),
                arguments=["-d", PathJoinSubstitution(
                    [FindPackageShare("ur_moveit_config"), "rviz", "view_robot.rviz"])],
                parameters=common)
    return [move_group, rviz]


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    gazebo_gui = LaunchConfiguration("gazebo_gui")

    args = [
        DeclareLaunchArgument("ur_type", default_value="ur3e",
                              choices=["ur3", "ur3e"], description="UR3 hoặc UR3e"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        DeclareLaunchArgument("start_commander", default_value="false",
                              description="Mở llm_commander trong xterm (cần: sudo apt install xterm)"),
        DeclareLaunchArgument("commander_delay", default_value="25.0",
                              description="Giây chờ Gazebo + MoveIt khởi động trước khi mở commander"),
        DeclareLaunchArgument("scene_config", default_value=PathJoinSubstitution(
            [FindPackageShare(PKG), "config", "scene.yaml"])),
    ]

    # ---------------- Robot description: UR + tay gắp, chạy trong Gazebo ----------------
    controllers = PathJoinSubstitution([FindPackageShare(PKG), "config", "ur_gripper_controllers.yaml"])
    initial_positions = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "initial_positions.yaml"])
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare(PKG), "urdf", "ur_with_gripper.urdf.xacro"]), " ",
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
        PythonLaunchDescriptionSource([FindPackageShare("ros_gz_sim"), "/launch", "/gz_sim.launch.py"]),
        launch_arguments={"gz_args": [" -r -v 3 ", world_path]}.items(),
        condition=IfCondition(gazebo_gui))
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("ros_gz_sim"), "/launch", "/gz_sim.launch.py"]),
        launch_arguments={"gz_args": [" -s -r -v 3 ", world_path]}.items(),
        condition=UnlessCondition(gazebo_gui))

    spawn_robot = Node(package="ros_gz_sim", executable="create", name="spawn_ur", output="screen",
                       arguments=["-world", WORLD_NAME, "-name", "ur",
                                  "-string", robot_description_content, "-allow_renaming", "false"])

    clock_bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"])

    def spawner(name):
        return Node(package="controller_manager", executable="spawner",
                    arguments=[name, "--controller-manager", "/controller_manager"])

    # Bàn/khối/zone/camera spawn sau khi robot đã được spawn xong
    scene_after_robot = RegisterEventHandler(OnProcessExit(
        target_action=spawn_robot, on_exit=[OpaqueFunction(function=spawn_scene)]))

    commander = TimerAction(
        period=LaunchConfiguration("commander_delay"),
        condition=IfCondition(LaunchConfiguration("start_commander")),
        actions=[Node(package=PKG, executable="llm_commander", name="llm_commander",
                      output="screen", prefix="xterm -fa Monospace -fs 11 -geometry 110x45 -e",
                      parameters=[{"use_sim_time": True}])])

    return LaunchDescription(args + [
        rsp, gazebo_with_gui, gazebo_headless, spawn_robot, clock_bridge,
        spawner("joint_state_broadcaster"), spawner("joint_trajectory_controller"),
        spawner("gripper_controller"),
        scene_after_robot, OpaqueFunction(function=moveit_nodes), commander,
    ])
