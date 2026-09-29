"""Robot Skills: các hành vi mức cao, mỗi skill thực hiện qua MoveIt 2.

Skill mức cao (LLM được phép gọi, xem task_validator.SKILL_SPECS):
    home(), pick(object), place(object, zone), move_above(object), move_to_zone(zone)
Skill nội bộ (dùng bên trong pick/place, LLM KHÔNG gọi trực tiếp):
    open_gripper(), close_gripper(object)

Mọi chuyển động đều đi qua MoveIt 2 (thư viện pymoveit2 gọi action
/move_action, service /compute_cartesian_path, action /execute_trajectory),
nên MoveIt tự kiểm tra: giới hạn khớp, self-collision, va chạm với bàn và
các khối (được thêm vào Planning Scene dưới dạng collision object).

Mỗi skill trả về một SkillStatus.
"""
from __future__ import annotations

import math
from enum import Enum
from typing import Dict, List, Optional, Tuple

from pymoveit2 import MoveIt2
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from .gazebo_bridge import GazeboBridge
from .scene_config import SceneConfig, WorldState

UR_JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]


class SkillStatus(str, Enum):
    SUCCESS = "SUCCESS"
    FAILED = "FAILED"
    INVALID_OBJECT = "INVALID_OBJECT"
    INVALID_ZONE = "INVALID_ZONE"
    PRECONDITION_FAILED = "PRECONDITION_FAILED"   # vd: place khi không cầm vật
    ZONE_OCCUPIED = "ZONE_OCCUPIED"
    PLANNING_FAILED = "PLANNING_FAILED"
    EXECUTION_FAILED = "EXECUTION_FAILED"


class RobotSkills:
    def __init__(self, node: Node, scene: SceneConfig, world: WorldState,
                 gazebo: GazeboBridge, group_name: str = "ur_manipulator",
                 base_link: str = "base_link", ee_link: str = "tool0"):
        self.node = node
        self.log = node.get_logger()
        self.scene = scene
        self.world = world
        self.gazebo = gazebo
        self.ee_link = ee_link
        self.base_link = base_link

        m = scene.motion
        self.approach_h = float(m.get("approach_height", 0.12))
        self.grasp_clear = float(m.get("grasp_clearance", 0.01))
        self.place_clear = float(m.get("place_clearance", 0.002))
        self.quat = tuple(m.get("tool_down_quat_xyzw", [1.0, 0.0, 0.0, 0.0]))
        self.home_joints = list(m.get("home_joints", [0, -1.5708, 1.5708, -1.5708, -1.5708, 0]))
        self.cart_step = float(m.get("cartesian_max_step", 0.005))
        self.cart_min_fraction = float(m.get("cartesian_min_fraction", 0.95))
        # pymoveit2.plan(..., timeout_sec=None) chờ VÔ THỜI HẠN nếu service/action của
        # move_group không bao giờ trả lời (vd. sau một lần thực thi trước bị huỷ giữa
        # chừng) — luôn chặn trên bằng planning_time * planning_attempts (+ dư) để một
        # lần planning "treo" không làm treo cả SkillExecutor.
        self.plan_timeout = float(m.get("planning_time", 5.0)) * int(m.get("planning_attempts", 10)) + 10.0
        self.free_move_attempts = 8

        self.moveit2 = MoveIt2(
            node=node,
            joint_names=UR_JOINTS,
            base_link_name=base_link,
            end_effector_name=ee_link,
            group_name=group_name,
            callback_group=ReentrantCallbackGroup(),
        )
        # Scale vận tốc/gia tốc để chuyển động mượt, an toàn
        self._try_set("max_velocity", float(m.get("max_velocity", 0.3)))
        self._try_set("max_acceleration", float(m.get("max_acceleration", 0.3)))
        self._try_set("allowed_planning_time", float(m.get("planning_time", 5.0)))
        self._try_set("num_planning_attempts", int(m.get("planning_attempts", 10)))

    def _try_set(self, attr: str, value) -> None:
        try:
            setattr(self.moveit2, attr, value)
        except Exception as e:  # khác biệt giữa các phiên bản pymoveit2
            self.log.warn(f"pymoveit2: cannot set {attr}: {e}")

    # ==================================================================
    # Planning scene (bàn + 3 khối) để MoveIt tránh va chạm môi trường
    # ==================================================================
    def setup_planning_scene(self) -> None:
        s = self.scene
        # Hạ mặt bàn trong planning scene xuống 4 mm so với thực tế: khối đang nằm
        # sát mặt bàn khi vừa được attach sẽ không bị MoveIt coi là "đang va chạm"
        # (contact distance = 0) làm hỏng start state của đoạn nhấc lên.
        margin = 0.004
        size = (s.table_size[0], s.table_size[1], s.table_size[2] - margin)
        center = (s.table_center[0], s.table_center[1], s.table_center[2] - margin / 2.0)
        self.moveit2.add_collision_box(
            id="table", size=size, position=center, quat_xyzw=(0.0, 0.0, 0.0, 1.0),
            frame_id=s.frame_id)
        for obj in s.objects:
            self._update_object_collision(obj)
        self.log.info("Planning scene: table + %d cubes added." % len(s.objects))

    def _update_object_collision(self, obj: str) -> None:
        x, y = self.world.positions[obj]
        c = self.scene.cube_size
        self.moveit2.add_collision_box(
            id=obj, size=(c, c, c), position=(x, y, self.scene.cube_center_z()),
            quat_xyzw=(0.0, 0.0, 0.0, 1.0), frame_id=self.scene.frame_id)

    # ==================================================================
    # Primitive chuyển động (đều qua MoveIt 2)
    # ==================================================================
    def _tool_z_above_cube_top(self, clearance: float) -> float:
        """Độ cao tool0 khi flange cách mặt trên khối (đặt trên bàn) một khoảng clearance."""
        return self.scene.table_top_z + self.scene.cube_size + clearance

    def _move_free(self, xyz: Tuple[float, float, float]) -> SkillStatus:
        """Chuyển động tự do (OMPL) tới pose, tool chúc xuống.

        Nếu để OMPL tự lấy mẫu IK từ pose, nó chọn ngẫu nhiên trong nhiều nhánh nghiệm
        (elbow gập ngược, khớp lệch 2π...) -> tay xoay cả vòng, đoạn Cartesian sau đó bị
        PATH_TOLERANCE_VIOLATED hoặc plan trả INVALID_MOTION_PLAN. Vì vậy giải IK với seed là
        tư thế home (luôn ra nhánh elbow-up hướng ra trước) rồi plan theo khớp.
        """
        goal = self._ik_from_home(xyz)
        for _ in range(self.free_move_attempts):
            if goal is not None:
                traj = self.moveit2.plan(joint_positions=goal, joint_names=UR_JOINTS,
                                         timeout_sec=self.plan_timeout)
            else:
                traj = self.moveit2.plan(position=xyz, quat_xyzw=self.quat,
                                         frame_id=self.scene.frame_id, target_link=self.ee_link,
                                         timeout_sec=self.plan_timeout)
            if traj is None:
                self.log.info("free move: plan failed, retrying")
            elif not self._is_compact(traj):
                self.log.info("free move: rejected wide-swing plan, retrying "
                              f"(final={[round(v, 2) for v in traj.points[-1].positions]})")
            else:
                return self._execute(traj)
            if goal is None:
                goal = self._ik_from_home(xyz)
        return SkillStatus.PLANNING_FAILED

    def _ik_from_home(self, xyz: Tuple[float, float, float]):
        js = self.moveit2.compute_ik(position=xyz, quat_xyzw=self.quat,
                                     ik_link_name=self.ee_link,
                                     start_joint_state=self.home_joints, timeout_sec=5.0)
        if js is None:
            return None
        by_name = dict(zip(js.name, js.position))
        if not all(j in by_name for j in UR_JOINTS):
            return None
        goal = [float(by_name[j]) for j in UR_JOINTS]
        return goal if all(abs(v) <= math.pi + 0.05 for v in goal) else None

    @staticmethod
    def _is_compact(traj) -> bool:
        pts = traj.points
        if not pts:
            return False
        for i in range(len(pts[0].positions)):
            vals = [p.positions[i] for p in pts]
            if max(vals) - min(vals) > 3.3 or abs(vals[-1]) > math.pi + 0.05:
                return False
        return True

    def _move_linear(self, xyz: Tuple[float, float, float]) -> SkillStatus:
        """Chuyển động thẳng Cartesian (hạ/nâng tool) — dùng compute_cartesian_path."""
        traj = self.moveit2.plan(position=xyz, quat_xyzw=self.quat,
                                 frame_id=self.scene.frame_id, target_link=self.ee_link,
                                 cartesian=True, cartesian_max_step=self.cart_step,
                                 cartesian_fraction_threshold=self.cart_min_fraction,
                                 timeout_sec=self.plan_timeout)
        return self._execute(traj)

    def _execute(self, traj, timeout_sec: float = 45.0) -> SkillStatus:
        if traj is None:
            return SkillStatus.PLANNING_FAILED
        self.moveit2.execute(traj)
        # timeout_sec bắt buộc: nếu controller/action server treo (vd. sau một
        # lần thực thi trước bị huỷ giữa chừng), skill phải FAILED thay vì
        # đứng im vô thời hạn và làm treo cả SkillExecutor.
        ok = self.moveit2.wait_until_executed(timeout_sec=timeout_sec, cancel_on_timeout=True)
        return SkillStatus.SUCCESS if ok in (True, None) else SkillStatus.EXECUTION_FAILED

    # ==================================================================
    # Skill nội bộ: tay gắp ảo
    # ==================================================================
    def open_gripper(self) -> SkillStatus:
        """UR3 không có gripper: mở = tách vật khỏi tool0 (nếu đang cầm)."""
        obj = self.world.holding
        if obj is None:
            return SkillStatus.SUCCESS
        self.moveit2.detach_collision_object(id=obj)
        self.gazebo.stop_carry()
        self.world.holding = None
        return SkillStatus.SUCCESS

    def close_gripper(self, obj: str) -> SkillStatus:
        """Đóng = gắn collision object của khối vào tool0 trong MoveIt (để planner
        tính cả khối khi tránh va chạm) + cho khối bám tool0 trong Gazebo."""
        self.moveit2.attach_collision_object(
            id=obj, link_name=self.ee_link,
            touch_links=[self.ee_link, "wrist_3_link", "flange"])
        self.gazebo.start_carry(obj, z_offset=self.grasp_clear + self.scene.cube_size / 2.0)
        self.world.holding = obj
        return SkillStatus.SUCCESS

    # ==================================================================
    # Skill mức cao
    # ==================================================================
    def home(self) -> SkillStatus:
        traj = self.moveit2.plan(joint_positions=self.home_joints, joint_names=UR_JOINTS,
                                 timeout_sec=self.plan_timeout)
        return self._execute(traj)

    def move_above(self, obj: str) -> SkillStatus:
        if obj not in self.scene.objects:
            return SkillStatus.INVALID_OBJECT
        x, y = self.world.positions[obj]
        return self._move_free((x, y, self._tool_z_above_cube_top(self.approach_h)))

    def move_to_zone(self, zone: str) -> SkillStatus:
        if zone not in self.scene.zones:
            return SkillStatus.INVALID_ZONE
        x, y = self.scene.zones[zone].center_xy
        extra = self.scene.cube_size if self.world.holding else 0.0
        return self._move_free((x, y, self._tool_z_above_cube_top(self.approach_h) + extra))

    def pick(self, obj: str) -> SkillStatus:
        if obj not in self.scene.objects:
            return SkillStatus.INVALID_OBJECT
        if self.world.holding is not None:
            return SkillStatus.PRECONDITION_FAILED
        x, y = self.world.positions[obj]
        z_pre = self._tool_z_above_cube_top(self.approach_h)
        z_grasp = self._tool_z_above_cube_top(self.grasp_clear)

        for step in (lambda: self.move_above(obj),            # 1. tới phía trên vật
                     self.open_gripper,                        # 2. mở tay
                     lambda: self._move_linear((x, y, z_grasp))):  # 3. hạ thẳng xuống
            st = step()
            if st != SkillStatus.SUCCESS:
                return st
        self.close_gripper(obj)                                # 4. gắp
        st = self._move_linear((x, y, z_pre))                  # 5. nhấc thẳng lên
        return st

    def place(self, obj: str, zone: str) -> SkillStatus:
        if obj not in self.scene.objects:
            return SkillStatus.INVALID_OBJECT
        if zone not in self.scene.zones:
            return SkillStatus.INVALID_ZONE
        if self.world.holding != obj:
            return SkillStatus.PRECONDITION_FAILED
        occ = self.world.occupant_of(zone)
        if occ is not None and occ != obj:
            return SkillStatus.ZONE_OCCUPIED

        x, y = self.scene.zones[zone].center_xy
        z_pre = self._tool_z_above_cube_top(self.approach_h) + self.scene.cube_size
        z_release = self._tool_z_above_cube_top(self.grasp_clear + self.place_clear)

        st = self._move_free((x, y, z_pre))                    # 1. tới phía trên zone
        if st != SkillStatus.SUCCESS:
            return st
        st = self._move_linear((x, y, z_release))              # 2. hạ thẳng xuống
        if st != SkillStatus.SUCCESS:
            return st
        self.open_gripper()                                    # 3. thả
        self.world.positions[obj] = (x, y)
        self._update_object_collision(obj)                     # cập nhật planning scene
        self.gazebo.set_pose(obj, x, y, self.scene.cube_center_z())  # đặt chuẩn trong Gazebo
        return self._move_linear((x, y, z_pre))                # 4. rút lên
