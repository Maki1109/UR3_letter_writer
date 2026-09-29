"""Plan Validator: kiểm tra kế hoạch JSON do LLM sinh ra TRƯỚC khi thực thi.

Hai tầng kiểm tra:
  1. Cú pháp / schema  : đúng JSON, có "plan" là list, mỗi bước có "skill"
                          nằm trong whitelist, đúng & đủ tham số, không có
                          khoá lạ (vd. "joints", "trajectory" -> bị chặn).
  2. Ngữ nghĩa         : mô phỏng kế hoạch trên bản sao WorldState để phát hiện
                          object/zone không tồn tại, pick khi tay đang cầm vật,
                          place vật không cầm, đặt vào zone đã bị chiếm, đặt vật vào
                          đúng zone nó đang nằm (REDUNDANT_MOVE), kết thúc mà vẫn cầm vật...

Module thuần Python (không import rclpy) -> unit test được offline.
"""
from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from .scene_config import SceneConfig, WorldState

# ---------------------------------------------------------------------------
# Whitelist skill: tên -> (danh sách tham số bắt buộc, mô tả cho LLM)
# Đây là NGUỒN DUY NHẤT định nghĩa skill hợp lệ; prompt cũng sinh từ đây.
# ---------------------------------------------------------------------------
SKILL_SPECS: Dict[str, Tuple[List[str], str]] = {
    "home": ([], "move the arm to the safe home configuration"),
    "pick": (["object"], "move above the object, descend, grasp it, lift it"),
    "place": (["object", "zone"],
              "carry the HELD object above the zone, lower it, release it, retreat"),
    "move_above": (["object"], "move the tool above an object without grasping (e.g. to point at it)"),
    "move_to_zone": (["zone"], "move the tool above a zone without releasing anything"),
}

# Khoá được chấp nhận ngoài tham số (LLM hay thêm lời giải thích)
TOLERATED_STEP_KEYS = {"reason", "comment"}
MAX_PLAN_LENGTH = 30


@dataclass
class ValidationResult:
    ok: bool
    plan: List[Dict[str, str]] = field(default_factory=list)   # plan đã chuẩn hoá
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    message: str = ""                                           # "message" của LLM


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


class TaskValidator:
    def __init__(self, scene: SceneConfig):
        self.scene = scene
        self.objects = set(scene.objects)
        self.zones = set(scene.zones)

    # ------------------------------------------------------------------
    def validate_text(self, llm_text: str, world: WorldState) -> ValidationResult:
        try:
            data = extract_json(llm_text)
        except (ValueError, json.JSONDecodeError) as e:
            return ValidationResult(False, errors=[f"Response is not valid JSON: {e}"])
        return self.validate(data, world)

    # ------------------------------------------------------------------
    def validate(self, data: Any, world: WorldState) -> ValidationResult:
        res = ValidationResult(ok=False)

        # ---------- 1. Schema ----------
        if not isinstance(data, dict):
            res.errors.append("Top level must be a JSON object with key 'plan'.")
            return res
        res.message = str(data.get("message", ""))
        extra_top = set(data) - {"plan", "message"}
        if extra_top:
            res.warnings.append(f"Ignored top-level keys: {sorted(extra_top)}")
        plan = data.get("plan")
        if not isinstance(plan, list):
            res.errors.append("'plan' must be a list.")
            return res
        if len(plan) == 0:
            res.errors.append("LLM returned an empty plan"
                              + (f": {res.message}" if res.message else "."))
            return res
        if len(plan) > MAX_PLAN_LENGTH:
            res.errors.append(f"Plan too long ({len(plan)} > {MAX_PLAN_LENGTH} steps).")
            return res

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
                val = str(step[p]).strip().lower()
                if p == "object" and val not in self.objects:
                    res.errors.append(f"Step {i} ({skill}): INVALID_OBJECT '{step[p]}' "
                                      f"(valid: {', '.join(sorted(self.objects))}).")
                if p == "zone" and val not in self.zones:
                    res.errors.append(f"Step {i} ({skill}): INVALID_ZONE '{step[p]}' "
                                      f"(valid: {', '.join(sorted(self.zones))}).")
                clean[p] = val
            normalized.append(clean)

        if res.errors:
            return res

        # Luôn kết thúc bằng home() (đề yêu cầu); tự bổ sung nếu LLM quên
        if normalized[-1]["skill"] != "home":
            normalized.append({"skill": "home"})
            res.warnings.append("Plan did not end with home(); home() was appended.")

        # ---------- 2. Ngữ nghĩa: mô phỏng trên bản sao world ----------
        sim = world.copy()
        picked_from: Dict[str, Optional[str]] = {}
        kept: List[Dict[str, str]] = []
        for i, step in enumerate(normalized, start=1):
            s = step["skill"]
            if s == "pick":
                obj = step["object"]
                if sim.holding is not None:
                    res.errors.append(f"Step {i} pick({obj}): robot is already holding "
                                      f"{sim.holding}; place it first.")
                    break
                picked_from[obj] = self.scene.zone_at(sim.positions[obj])
                sim.holding = obj
            elif s == "place":
                obj, zone = step["object"], step["zone"]
                if sim.holding != obj:
                    res.errors.append(f"Step {i} place({obj}, {zone}): robot is not holding "
                                      f"{obj} (holding: {sim.holding or 'nothing'}).")
                    break
                if picked_from.get(obj) == zone:
                    # Vật đã ở đúng vùng: pick+place liền kề là bước thừa -> bỏ đi (không đổi
                    # kết quả cuối, chỉ bớt chuyển động); không liền kề thì từ chối.
                    if kept and kept[-1] == {"skill": "pick", "object": obj}:
                        kept.pop()
                        sim.holding = None
                        res.warnings.append(f"Removed redundant pick/place: {obj} is already "
                                            f"in {zone}.")
                        continue
                    res.errors.append(f"Step {i} place({obj}, {zone}): REDUNDANT_MOVE, {obj} "
                                      f"was already in {zone}; do not move objects that are "
                                      f"already where they should be.")
                    break
                occ = sim.occupant_of(zone)
                if occ is not None:
                    res.errors.append(f"Step {i} place({obj}, {zone}): ZONE_OCCUPIED by {occ}.")
                    break
                sim.positions[obj] = self.scene.zones[zone].center_xy
                sim.holding = None
            elif s == "home":
                pass
            # move_above / move_to_zone không đổi trạng thái
            kept.append(step)
        normalized = kept if not res.errors else normalized

        if not res.errors and sim.holding is not None:
            res.errors.append(f"Plan ends while still holding {sim.holding}.")

        res.plan = normalized
        res.ok = not res.errors
        return res
