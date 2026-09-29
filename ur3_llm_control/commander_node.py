"""llm_commander: ROS 2 node nhận câu lệnh ngôn ngữ tự nhiên và điều khiển UR3e.

Luồng:  User command -> LLMPlanner (9Router) -> JSON plan -> TaskValidator
        -> SkillExecutor -> RobotSkills -> MoveIt 2 -> UR3e (Gazebo)

Cách dùng:
  ros2 run ur3_llm_control llm_commander                     # chế độ gõ lệnh tương tác
  ros2 run ur3_llm_control llm_commander --ros-args -p command:="Put the red cube in zone B"
  ros2 topic pub --once /user_command std_msgs/String "{data: 'Move the blue cube to zone C'}"
     (topic chỉ được xử lý khi chạy với -p interactive:=false)

Lệnh đặc biệt trong chế độ tương tác:
  /state        in trạng thái thế giới      /student  in nhiệm vụ cá nhân
  /plan {json}  bỏ qua LLM, đưa thẳng plan vào Validator (demo từ chối plan sai)
  /home         về home                      /quit     thoát
"""
from __future__ import annotations

import json
import os
import queue
import sys
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.srv import GetCartesianPath
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .gazebo_bridge import DEFAULT_WORLD_NAME, GazeboBridge
from .llm_planner import LLMConfig, LLMPlanner
from .robot_skills import RobotSkills
from .scene_config import WorldState, load_scene, load_student
from .skill_executor import SkillExecutor
from .task_validator import TaskValidator, format_step

BAR = "=" * 60


def out(msg: str = "") -> None:
    print(msg, flush=True)


class LLMCommander(Node):
    def __init__(self):
        super().__init__("llm_commander")
        # Gazebo chạy với /clock -> node cũng phải dùng sim time (TF, timestamps MoveIt)
        if not self.get_parameter("use_sim_time").value:
            self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        share = get_package_share_directory("ur3_llm_control")
        p = self.declare_parameter
        scene_path = p("scene_config", os.path.join(share, "config", "scene.yaml")).value
        student_path = p("student_config", os.path.join(share, "config", "student_config.yaml")).value
        llm_path = p("llm_config", os.path.join(share, "config", "llm_config.yaml")).value
        prompt_path = p("prompt_file", os.path.join(share, "prompts", "system_prompt.txt")).value
        self.command = p("command", "").value
        self.interactive = p("interactive", True).value
        self.use_gazebo = p("use_gazebo", True).value
        group = p("planning_group", "ur_manipulator").value

        self.scene = load_scene(scene_path)
        self.student = load_student(student_path, self.scene)
        self.world = WorldState(self.scene)
        self.validator = TaskValidator(self.scene)
        with open(prompt_path, "r", encoding="utf-8") as f:
            template = f.read()
        self.llm_cfg = LLMConfig.load(llm_path)
        self.planner = LLMPlanner(self.llm_cfg, template, self.scene, self.student, self.validator)

        def resting_positions():
            z = self.scene.cube_center_z()
            return {obj: (x, y, z) for obj, (x, y) in self.world.positions.items()}

        self.gazebo = GazeboBridge(self, self.scene.frame_id, world_name=DEFAULT_WORLD_NAME,
                                   enabled=self.use_gazebo, resting_positions_fn=resting_positions)
        self.skills = RobotSkills(self, self.scene, self.world, self.gazebo, group_name=group)
        self.executor_ = SkillExecutor(self.skills, printer=out)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, "scene_markers", latched)
        self.status_pub = self.create_publisher(String, "task_status", 10)
        self.cmd_queue: "queue.Queue[str]" = queue.Queue()
        self.create_subscription(String, "user_command", lambda m: self.cmd_queue.put(m.data), 10)

    # ------------------------------------------------------------------
    def _warmup_llm(self) -> None:
        t0 = time.time()
        try:
            self.planner.warmup(self.world)
            self.get_logger().info(f"LLM warm-up done in {time.time() - t0:.1f}s")
        except Exception as e:
            self.get_logger().warn(f"LLM warm-up skipped: {e}")

    def startup(self) -> None:
        threading.Thread(target=self._warmup_llm, daemon=True).start()
        cli = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.get_logger().info("Waiting for MoveIt move_group ...")
        while not cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("  ... move_group not ready yet")
        if self.use_gazebo:
            self.gazebo.wait_ready(10.0)   # vị trí ban đầu lấy từ scene.yaml (xem gazebo_bridge.py)
        self.skills.setup_planning_scene()
        self.publish_markers()
        self.print_student()
        out(BAR)
        out("CURRENT WORLD STATE:")
        out(self.world.describe())

    def print_student(self) -> None:
        s = self.student
        out(BAR)
        out(f"STUDENT: {s.student_name}  |  ID: {s.student_id}")
        out(f"P = {s.last_two:02d} mod 6 = {s.p}")
        for z, c in s.zone_to_color.items():
            out(f"  {z.replace('_', ' ').title()} -> {c.capitalize()}")

    def publish_markers(self) -> None:
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for i, z in enumerate(self.scene.zones.values()):
            sq = Marker()
            sq.header.frame_id, sq.header.stamp = self.scene.frame_id, stamp
            sq.ns, sq.id, sq.type, sq.action = "zones", i, Marker.CUBE, Marker.ADD
            sq.pose.position.x, sq.pose.position.y = z.center_xy
            sq.pose.position.z = self.scene.table_top_z + 0.001
            sq.pose.orientation.w = 1.0
            sq.scale.x = sq.scale.y = self.scene.zone_size
            sq.scale.z = 0.002
            sq.color.r, sq.color.g, sq.color.b = 0.2, 0.8, 0.2
            sq.color.a = 0.6
            tx = Marker()
            tx.header = sq.header
            tx.ns, tx.id, tx.type, tx.action = "zone_labels", i, Marker.TEXT_VIEW_FACING, Marker.ADD
            tx.pose.position.x, tx.pose.position.y = z.center_xy
            tx.pose.position.z = self.scene.table_top_z + 0.09
            tx.pose.orientation.w = 1.0
            tx.scale.z = 0.03
            tx.color.r = tx.color.g = tx.color.b = tx.color.a = 1.0
            tx.text = z.label
            arr.markers += [sq, tx]
        self.marker_pub.publish(arr)

    # ------------------------------------------------------------------
    def handle_command(self, command: str) -> bool:
        command = command.strip()
        if not command:
            return True
        if command in ("/quit", "/exit"):
            return False
        if command == "/state":
            out(self.world.describe())
            return True
        if command == "/student":
            self.print_student()
            return True
        if command == "/home":
            out(f"home() ... {self.skills.home().value}")
            return True

        out(BAR)
        out("USER COMMAND:")
        out(command)
        out("")

        if command.startswith("/plan"):
            # Kênh kiểm thử: đưa plan JSON thủ công vào Validator (không qua LLM)
            result = self.validator.validate_text(command[len("/plan"):], self.world)
            raw = [command[len("/plan"):].strip()]
            attempts = 0
        else:
            t0 = time.time()
            outcome = self.planner.plan(command, self.world)
            result, raw, attempts = outcome.result, outcome.raw_responses, outcome.attempts
            self.get_logger().info(f"LLM answered in {time.time() - t0:.1f}s, attempts={attempts}")

        out("LLM PLAN:")
        if result.plan:
            for step in result.plan:
                out(format_step(step))
        else:
            out(raw[-1] if raw else "(no response)")
        if result.message:
            out(f"(LLM note: {result.message})")
        for w in result.warnings:
            out(f"[validator warning] {w}")
        out("")

        if not result.ok:
            out("VALIDATION: REJECTED")
            for e in result.errors:
                out(f"  - {e}")
            out("")
            out("TASK REJECTED (nothing was executed)")
            self._publish_status("REJECTED", result.errors)
            return True

        out("VALIDATION: PASSED")
        out("")
        ok, results = self.executor_.run(result.plan)
        self._publish_status("SUCCESS" if ok else "FAILED",
                             [f"{label}: {st.value}" for label, st in results])
        return True

    def _publish_status(self, status: str, details) -> None:
        self.status_pub.publish(String(data=json.dumps({"status": status, "details": details})))


def main(args=None):
    rclpy.init(args=args)
    node = LLMCommander()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        node.startup()
        if node.command:                                    # chế độ 1 lệnh
            node.handle_command(node.command)
        elif node.interactive:                              # chế độ gõ lệnh
            out(BAR)
            out("Nhập câu lệnh (tiếng Việt/English). /help để xem lệnh đặc biệt.")
            while rclpy.ok():
                try:
                    line = input("\nCommand> ")
                except EOFError:
                    break
                if line.strip() == "/help":
                    out(__doc__)
                    continue
                if not node.handle_command(line):
                    break
        else:                                               # chế độ nhận lệnh qua topic
            out("Listening on /user_command ...")
            while rclpy.ok():
                try:
                    node.handle_command(node.cmd_queue.get(timeout=0.5))
                except queue.Empty:
                    pass
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()
