"""Skill Executor: chạy tuần tự kế hoạch ĐÃ ĐƯỢC VALIDATE, in kết quả từng bước.

Chỉ các skill có trong whitelist (task_validator.SKILL_SPECS) mới được ánh xạ
sang hàm của RobotSkills; LLM không bao giờ gọi trực tiếp MoveIt.
Dừng ngay tại bước đầu tiên không SUCCESS.
"""
from __future__ import annotations

from typing import Callable, Dict, List, Tuple

from .robot_skills import RobotSkills, SkillStatus
from .task_validator import SKILL_SPECS, format_step

LABEL_WIDTH = 26


def dotted(label: str, status: str, width: int = LABEL_WIDTH) -> str:
    """'pick(red_cube)' -> 'pick(red_cube) ........... SUCCESS'"""
    pad = max(1, width - len(label))
    return f"{label} {'.' * pad} {status}"


class SkillExecutor:
    def __init__(self, skills: RobotSkills, printer: Callable[[str], None] = print):
        self.skills = skills
        self.print = printer
        # Bảng điều phối: tên skill -> hàm thực thi (khoá khớp đúng SKILL_SPECS)
        self.dispatch: Dict[str, Callable[..., SkillStatus]] = {
            "home": skills.home,
            "pick": skills.pick,
            "place": skills.place,
            "move_above": skills.move_above,
            "move_to_zone": skills.move_to_zone,
        }
        assert set(self.dispatch) == set(SKILL_SPECS), "dispatch table out of sync"

    def run(self, plan: List[Dict[str, str]]) -> Tuple[bool, List[Tuple[str, SkillStatus]]]:
        results: List[Tuple[str, SkillStatus]] = []
        self.print("EXECUTION:")
        for step in plan:
            name = step["skill"]
            label = format_step(step)
            fn = self.dispatch.get(name)
            if fn is None:                              # phòng thủ lớp 2
                status = SkillStatus.FAILED
            else:
                params = SKILL_SPECS[name][0]
                try:
                    status = fn(*[step[p] for p in params])
                except Exception as e:                  # lỗi bất ngờ -> FAILED, không crash
                    self.skills.log.error(f"{label} raised: {e}")
                    status = SkillStatus.FAILED
            results.append((label, status))
            self.print(dotted(label, status.value))
            if status != SkillStatus.SUCCESS:
                self.print("")
                self.print(f"TASK FAILED at {label}: {status.value}")
                return False, results
        self.print("")
        self.print("TASK SUCCESS")
        return True, results
