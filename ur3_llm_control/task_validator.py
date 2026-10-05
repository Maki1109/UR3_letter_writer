"""Plan Validator: kiểm tra kế hoạch JSON do LLM sinh ra TRƯỚC khi thực thi.

Hai tầng kiểm tra:
  1. Cú pháp / schema  : đúng JSON, có "plan" là list, mỗi bước có "skill"
                          nằm trong whitelist, đúng & đủ tham số, không có
                          khoá lạ (vd. "joints", "trajectory" -> bị chặn).
  2. Ngữ nghĩa         : mô phỏng kế hoạch trên bản sao WorldState (lấy từ CAMERA)
                          để phát hiện object/zone không tồn tại, vật camera không
                          thấy, pick khi tay đang cầm vật, place vật không cầm,
                          đặt vào zone đang bị chiếm, kết thúc mà vẫn cầm vật...

Xử lý zone bị chiếm (Bài 03): nếu một bước place(X, zone) gặp zone đang có vật Y
mà kế hoạch chưa dời Y đi, validator CHÈN thêm (trước bước pick(X) tương ứng):
    check_zone(zone) -> pick(Y) -> place(Y, temporary_position)
và ghi cảnh báo. Có thể tắt bằng auto_resolve=False (khi đó kế hoạch bị từ chối
với lỗi ZONE_OCCUPIED và được gửi lại cho LLM sửa).

Module thuần Python (không import rclpy) -> unit test được offline.
"""
from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from .scene_config import TEMP_POSITION, SceneConfig, WorldState

# ---------------------------------------------------------------------------
# Whitelist skill: tên -> (danh sách tham số bắt buộc, mô tả cho LLM)
# Đây là NGUỒN DUY NHẤT định nghĩa skill hợp lệ; prompt cũng sinh từ đây.
# ---------------------------------------------------------------------------
SKILL_SPECS: Dict[str, Tuple[List[str], str]] = {
    "detect_objects": ([], "take a new camera image and update where every object is"),
    "check_zone": (["zone"], "use the camera to check whether a zone is free or which object occupies it"),
    "pick": (["object"], "locate the object with the camera, move above it, open the gripper, "
                         "descend, close the gripper on it, lift it"),
    "place": (["object", "zone"],
              "carry the HELD object to the zone, lower it, open the gripper, retreat. "
              f"zone may be \"{TEMP_POSITION}\" = a free spot on the table found automatically"),
    "home": ([], "move the arm to the safe home configuration"),
}

# Khoá được chấp nhận ngoài tham số (LLM hay thêm lời giải thích)
TOLERATED_STEP_KEYS = {"reason", "comment"}
MAX_PLAN_LENGTH = 40


@dataclass
class ValidationResult:
    ok: bool
    plan: List[Dict[str, str]] = field(default_factory=list)   # plan đã chuẩn hoá
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    message: str = ""                                           # "message" của LLM
    inserted: List[int] = field(default_factory=list)           # chỉ số bước do validator chèn


def skills_prompt_block() -> str:
    lines = []
    for name, (params, desc) in SKILL_SPECS.items():
        lines.append(f"- {name}({', '.join(params)}): {desc}")
    return "\n".join(lines)


def format_step(step: Dict[str, str]) -> str:
    """{"skill":"place","object":"red_cube","zone":"zone_b"} -> place(red_cube, zone_b)"""
    params = SKILL_SPECS.get(step.get("skill", ""), ([], ""))[0]
    args = [str(step.get(p, "?")) for p in params]
    return f"{step.get('skill', '?')}({', '.join(args)})"


def extract_json(text: str) -> Any:
    """Lấy object JSON từ câu trả lời LLM (chịu được ```json fences / text thừa)."""
    if text is None:
        raise ValueError("empty LLM response")
    t = text.strip()
    fence = re.search(r"```(?:json)?\s*(.*?)```", t, re.S | re.I)
    if fence:
        t = fence.group(1).strip()
    try:
        return json.loads(t)
    except json.JSONDecodeError:
        pass
    start, end = t.find("{"), t.rfind("}")
    if start == -1 or end <= start:
        raise ValueError("no JSON object found in LLM response")
    return json.loads(t[start:end + 1])


# Cách gọi khác của temporary_position mà LLM hay dùng
_TEMP_ALIASES = {"temporary_position", "temp", "temp_zone", "temporary", "temporary_zone",
                 "free_position", "free_space", "table", "buffer"}


class TaskValidator:
    def __init__(self, scene: SceneConfig, auto_resolve: bool = True):
        self.scene = scene
        self.objects = set(scene.objects)
        self.zones = set(scene.zones)
        self.auto_resolve = auto_resolve

    # ------------------------------------------------------------------
    def validate_text(self, llm_text: str, world: WorldState) -> ValidationResult:
        try:
            data = extract_json(llm_text)
        except (ValueError, json.JSONDecodeError) as e:
            return ValidationResult(False, errors=[f"Response is not valid JSON: {e}"])
        return self.validate(data, world)

    # ------------------------------------------------------------------
    def _normalize(self, data: Any, res: ValidationResult) -> List[Dict[str, str]]:
        if not isinstance(data, dict):
            res.errors.append("Top level must be a JSON object with key 'plan'.")
            return []
        res.message = str(data.get("message", ""))
        extra_top = set(data) - {"plan", "message"}
        if extra_top:
            res.warnings.append(f"Ignored top-level keys: {sorted(extra_top)}")
        plan = data.get("plan")
        if not isinstance(plan, list):
            res.errors.append("'plan' must be a list.")
            return []
        if len(plan) == 0:
            res.errors.append("LLM returned an empty plan"
                              + (f": {res.message}" if res.message else "."))
            return []
        if len(plan) > MAX_PLAN_LENGTH:
            res.errors.append(f"Plan too long ({len(plan)} > {MAX_PLAN_LENGTH} steps).")
            return []

        normalized: List[Dict[str, str]] = []
        for i, step in enumerate(plan, start=1):
            if not isinstance(step, dict):
                res.errors.append(f"Step {i}: must be an object, got {type(step).__name__}.")
                continue
            skill = str(step.get("skill", "")).strip().lower()
            if skill not in SKILL_SPECS:
                res.errors.append(f"Step {i}: skill '{step.get('skill')}' is not allowed "
                                  f"(allowed: {', '.join(SKILL_SPECS)}).")
                continue
            required = SKILL_SPECS[skill][0]
            unknown = set(step) - {"skill"} - set(required) - TOLERATED_STEP_KEYS
            if unknown:
                res.errors.append(f"Step {i} ({skill}): forbidden keys {sorted(unknown)}.")
            clean = {"skill": skill}
            for p in required:
                if p not in step:
                    res.errors.append(f"Step {i} ({skill}): missing parameter '{p}'.")
                    continue
                val = str(step[p]).strip().lower().replace(" ", "_")
                if p == "object" and val not in self.objects:
                    res.errors.append(f"Step {i} ({skill}): INVALID_OBJECT '{step[p]}' "
                                      f"(valid: {', '.join(sorted(self.objects))}).")
                if p == "zone":
                    if skill == "place" and val in _TEMP_ALIASES:
                        val = TEMP_POSITION
                    elif val not in self.zones:
                        valid = sorted(self.zones) + ([TEMP_POSITION] if skill == "place" else [])
                        res.errors.append(f"Step {i} ({skill}): INVALID_ZONE '{step[p]}' "
                                          f"(valid: {', '.join(valid)}).")
                clean[p] = val
            normalized.append(clean)
        return normalized

    # ------------------------------------------------------------------
    def validate(self, data: Any, world: WorldState) -> ValidationResult:
        res = ValidationResult(ok=False)
        normalized = self._normalize(data, res)
        if res.errors:
            return res

        # Luôn kết thúc bằng home() (đề yêu cầu); tự bổ sung nếu LLM quên
        if normalized[-1]["skill"] != "home":
            normalized.append({"skill": "home"})
            res.warnings.append("Plan did not end with home(); home() was appended.")

        # ---------- 2. Ngữ nghĩa: mô phỏng trên bản sao world (từ camera) ----------
        sim = world.copy()
        picked_from: Dict[str, Optional[str]] = {}
        kept: List[Dict[str, str]] = []
        inserted_steps: List[Dict[str, str]] = []
        last_pick_idx: Optional[int] = None          # vị trí trong kept của bước pick gần nhất
        for i, step in enumerate(normalized, start=1):
            s = step["skill"]
            if s == "pick":
                obj = step["object"]
                if sim.holding is not None:
                    res.errors.append(f"Step {i} pick({obj}): robot is already holding "
                                      f"{sim.holding}; place it first.")
                    break
                if not sim.is_known(obj):
                    res.errors.append(f"Step {i} pick({obj}): OBJECT_NOT_FOUND, the camera does "
                                      f"not see {obj}.")
                    break
                picked_from[obj] = sim.zone_of(obj)
                sim.holding = obj
                last_pick_idx = len(kept)
            elif s == "place":
                obj, zone = step["object"], step["zone"]
                if sim.holding != obj:
                    res.errors.append(f"Step {i} place({obj}, {zone}): robot is not holding "
                                      f"{obj} (holding: {sim.holding or 'nothing'}).")
                    break
                if zone != TEMP_POSITION and picked_from.get(obj) == zone:
                    # Vật đã ở đúng zone: pick+place liền kề là bước thừa -> bỏ đi
                    if kept and kept[-1] == {"skill": "pick", "object": obj}:
                        kept.pop()
                        sim.holding = None
                        sim.positions[obj] = world.positions.get(obj)
                        res.warnings.append(f"Removed redundant pick/place: {obj} is already "
                                            f"in {zone}.")
                        continue
                    res.errors.append(f"Step {i} place({obj}, {zone}): REDUNDANT_MOVE, {obj} "
                                      f"was already in {zone}.")
                    break
                if zone != TEMP_POSITION:
                    occ = sim.occupant_of(zone)
                    if occ is not None:
                        if not self.auto_resolve or last_pick_idx is None:
                            res.errors.append(f"Step {i} place({obj}, {zone}): ZONE_OCCUPIED by "
                                              f"{occ}; move {occ} to {TEMP_POSITION} first.")
                            break
                        # Chèn bước dọn zone ngay trước pick(obj) — lúc đó tay đang rỗng
                        fix = [{"skill": "check_zone", "zone": zone},
                               {"skill": "pick", "object": occ},
                               {"skill": "place", "object": occ, "zone": TEMP_POSITION}]
                        kept[last_pick_idx:last_pick_idx] = fix
                        inserted_steps += fix
                        last_pick_idx += len(fix)
                        sim.positions[occ] = None          # trên bàn, chỗ trống do hệ thống chọn
                        res.warnings.append(
                            f"{zone} is occupied by {occ} (camera): inserted "
                            f"pick({occ}) -> place({occ}, {TEMP_POSITION}) before pick({obj}).")
                    sim.positions[obj] = (*self.scene.zones[zone].center_xy, 0.0)
                else:
                    sim.positions[obj] = None
                sim.holding = None
            # detect_objects / check_zone / home không đổi trạng thái
            kept.append(step)

        if not res.errors and sim.holding is not None:
            res.errors.append(f"Plan ends while still holding {sim.holding}.")

        res.plan = kept if not res.errors else normalized
        if not res.errors:
            ids = {id(s) for s in inserted_steps}
            res.inserted = [k for k, s in enumerate(kept) if id(s) in ids]
        res.ok = not res.errors
        return res
