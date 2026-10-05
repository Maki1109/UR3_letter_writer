"""llm_commander: ROS 2 node nhận câu lệnh ngôn ngữ tự nhiên và điều khiển UR3e + tay gắp.

Luồng:  Camera -> WorldState  ┐
        User command ─────────┴-> LLMPlanner -> JSON plan -> TaskValidator
        -> SkillExecutor -> RobotSkills -> MoveIt 2 + gripper -> UR3e (Gazebo)
        -> Camera kiểm tra lại kết quả

Cách dùng:
  ros2 run ur3_llm_control llm_commander                     # chế độ gõ lệnh tương tác
  ros2 run ur3_llm_control llm_commander --ros-args -p command:="Put the red cube in zone B"
  ros2 topic pub --once /user_command std_msgs/String "{data: 'Move the blue cube to zone C'}"
     (topic chỉ được xử lý khi chạy với -p interactive:=false)

Lệnh đặc biệt trong chế độ tương tác:
  /state        chụp ảnh, in trạng thái (kèm toạ độ)   /student  in nhiệm vụ cá nhân
  /plan {json}  bỏ qua LLM, đưa thẳng plan vào Validator (demo từ chối plan sai)
  /home         về home      /open, /close   mở / đóng tay gắp      /quit  thoát
"""
from __future__ import annotations

import json
import os
import queue
import sys
import threading
import time
from typing import Dict, List

import rclpy
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.srv import GetCartesianPath
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from .camera import CameraPerception
from .llm_planner import LLMConfig, LLMPlanner
from .robot_skills import RobotSkills
from .scene_config import TEMP_POSITION, WorldState, load_scene, load_student
from .skill_executor import SkillExecutor
from .task_validator import TaskValidator, format_step

BAR = "=" * 64


def out(msg: str = "") -> None:
    print(msg, flush=True)


class LLMCommander(Node):
    def __init__(self):
        super().__init__("llm_commander")
        # Gazebo chạy với /clock -> node cũng phải dùng sim time
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
        auto_resolve = p("auto_resolve_occupied_zones", True).value
        group = p("planning_group", "ur_manipulator").value

        self.scene = load_scene(scene_path)
        self.student = load_student(student_path, self.scene)
        self.world = WorldState(self.scene)                 # rỗng: chỉ camera điền vào
        self.validator = TaskValidator(self.scene, auto_resolve=auto_resolve)
        with open(prompt_path, "r", encoding="utf-8") as f:
            template = f.read()
        self.llm_cfg = LLMConfig.load(llm_path)
        self.planner = LLMPlanner(self.llm_cfg, template, self.scene, self.student, self.validator)

        self.camera = CameraPerception(self, self.scene)
        self.skills = RobotSkills(self, self.scene, self.world, self.camera.capture, group_name=group)
        self.executor_ = SkillExecutor(self.skills, printer=out)

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
        cli = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.get_logger().info("Waiting for MoveIt move_group ...")
        while not cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("  ... move_group not ready yet")
        self.get_logger().info("Waiting for camera images ...")
        if not self.camera.wait_ready(60.0):
            self.get_logger().error("No camera image on %s — check the camera bridge."
                                    % self.scene.camera.get("image_topic"))
        self.skills.setup_planning_scene()
        self.skills.gripper.open()
        out("Moving to home and looking at the table ...")
        self.skills.home()
        self.observe()
        threading.Thread(target=self._warmup_llm, daemon=True).start()
        self.print_student()
        out(BAR)
        out("CAMERA — CURRENT WORLD STATE:")
        out(self.world.describe(with_coords=True))

    def observe(self) -> bool:
        ok = self.skills.refresh_world() is not None
        self.camera.publish_world(self.world)
        return ok

    def print_student(self) -> None:
        s = self.student
        out(BAR)
        out(f"STUDENT: {s.student_name}  |  ID: {s.student_id}")
        out(f"P = {s.last_two:02d} mod 6 = {s.p}")
        for z, c in s.zone_to_color.items():
            out(f"  {z.replace('_', ' ').title()} -> {c.capitalize()}")

    # ------------------------------------------------------------------
    def handle_command(self, command: str) -> bool:
        command = command.strip()
        if not command:
            return True
        if command in ("/quit", "/exit"):
            return False
        if command in ("/state", "/detect"):
            self.observe()
            out(self.world.describe(with_coords=True))
            return True
        if command == "/student":
            self.print_student()
            return True
        if command == "/home":
            out(f"home() ... {self.skills.home().value}")
            return True
        if command in ("/open", "/close"):
            gap = self.skills.gripper.open() if command == "/open" else self.skills.gripper.close()
            out(f"gripper gap = {gap * 1000:.1f} mm" if gap is not None else "gripper: no joint state")
            return True

        out(BAR)
        out("USER COMMAND:")
        out(command)
        out("")

        # 1. Camera: trạng thái thế giới mới nhất (đầu vào cho LLM + validator)
        if not self.observe():
            out("CAMERA: no image — cannot plan without perception.")
            return True
        out("CAMERA (world state):")
        out(self.world.describe())
        out("")

        # 2. LLM -> plan -> validator
        if command.startswith("/plan"):
            # Kênh kiểm thử: đưa plan JSON thủ công vào Validator (không qua LLM)
            result = self.validator.validate_text(command[len("/plan"):], self.world)
            raw = [command[len("/plan"):].strip()]
        else:
            t0 = time.time()
            outcome = self.planner.plan(command, self.world)
            result, raw = outcome.result, outcome.raw_responses
            self.get_logger().info(f"LLM answered in {time.time() - t0:.1f}s, attempts={outcome.attempts}")
            if outcome.attempts > 1:
                out(f"(LLM plan was repaired after validator feedback, attempts={outcome.attempts})")

        out("LLM PLAN:" if not result.inserted else "PLAN (LLM + validator):")
        if result.plan:
            for k, step in enumerate(result.plan):
                mark = "   <- added by validator (zone occupied)" if k in result.inserted else ""
                out(f"  {format_step(step)}{mark}")
        else:
            out(raw[-1] if raw else "(no response)")
        if result.message:
            out(f"(LLM note: {result.message})")
        for w in result.warnings:
            out(f"[validator] {w}")
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
        # 3. Thực thi
        ok, results = self.executor_.run(result.plan)
        # 4. Camera kiểm tra lại kết quả cuối cùng
        self.verify(result.plan)
        self._publish_status("SUCCESS" if ok else "FAILED",
                             [f"{label}: {st.value}" for label, st in results])
        return True

    def verify(self, plan: List[Dict[str, str]]) -> None:
        if not self.skills.at_home:
            self.skills.home()
        self.observe()
        final: Dict[str, str] = {}
        for step in plan:
            if step["skill"] == "place":
                final[step["object"]] = step["zone"]
        out("")
        out("CAMERA VERIFICATION:")
        for obj, zone in final.items():
            loc = self.world.location_of(obj)
            good = loc == zone or (zone == TEMP_POSITION and loc == "table")
            out(f"  {obj}: expected {zone}, camera sees {loc} ... {'OK' if good else 'MISMATCH'}")
        out("  zones: " + ", ".join(f"{z}={self.world.occupant_of(z) or 'free'}" for z in self.scene.zones))

    def _publish_status(self, status: str, details) -> None:
        self.status_pub.publish(String(data=json.dumps({"status": status, "details": details})))


def main(args=None):
    rclpy.init(args=args)
    node = LLMCommander()
    executor = MultiThreadedExecutor(num_threads=6)
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
