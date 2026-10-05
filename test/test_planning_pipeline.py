"""Unit test offline (không cần ROS/Gazebo): nhiệm vụ cá nhân, Plan Validator (kể cả tự
chèn bước dọn zone bị chiếm), nhận dạng khối từ ảnh camera, tìm chỗ trống, prompt,
vòng lặp LLM tự sửa.

Chạy:  cd ur3_llm_control && python3 -m pytest test -v
  hoặc trong workspace:  colcon test --packages-select ur3_llm_control
"""
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from ur3_llm_control.free_space import find_free_position  # noqa: E402
from ur3_llm_control.llm_planner import LLMConfig, LLMPlanner, build_system_prompt  # noqa: E402
from ur3_llm_control.perception import CubeDetector, rpy_to_matrix  # noqa: E402
from ur3_llm_control.scene_config import (TEMP_POSITION, WorldState,  # noqa: E402
                                          compute_student_task, load_scene)
from ur3_llm_control.task_validator import TaskValidator, extract_json, format_step  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..")
SCENE = load_scene(os.path.join(ROOT, "config", "scene.yaml"))
TEMPLATE = open(os.path.join(ROOT, "prompts", "system_prompt.txt"), encoding="utf-8").read()


def zone(z):
    return (*SCENE.zones[z].center_xy, 0.0)


def world(**overrides):
    """Trạng thái giống cảnh demo: blue ở zone_b, green ở zone_c, còn lại trên bàn."""
    pos = {"red_cube": (0.24, 0.12, 0.26), "yellow_cube": (0.25, -0.10, -0.2),
           "blue_cube": zone("zone_b"), "green_cube": zone("zone_c"),
           "purple_cube": (0.27, 0.26, 0.4)}
    pos.update(overrides)
    return WorldState(SCENE, {k: v for k, v in pos.items() if v != "missing"})


def v(plan_obj, w=None, auto_resolve=True):
    return TaskValidator(SCENE, auto_resolve=auto_resolve).validate(plan_obj, w or world())


def pick(o):
    return {"skill": "pick", "object": o}


def place(o, z):
    return {"skill": "place", "object": o, "zone": z}


HOME = {"skill": "home"}


# ----------------------------- nhiệm vụ cá nhân -----------------------------
def test_student_example_from_assignment():
    t = compute_student_task("Nguyen Van An", "23020123", SCENE)
    assert t.p == 5
    assert t.zone_to_object == {"zone_a": "blue_cube", "zone_b": "yellow_cube", "zone_c": "red_cube"}


def test_all_p_values():
    expected = {0: ("red", "yellow", "blue"), 1: ("red", "blue", "yellow"),
                2: ("yellow", "red", "blue"), 3: ("yellow", "blue", "red"),
                4: ("blue", "red", "yellow"), 5: ("blue", "yellow", "red")}
    for xx in range(100):
        t = compute_student_task("x", f"230201{xx:02d}")
        assert t.p == xx % 6
        assert tuple(t.zone_to_color.values()) == expected[xx % 6]


# ----------------------------- world state (từ camera) -----------------------------
def test_world_state_zone_occupancy():
    w = world()
    assert w.occupant_of("zone_b") == "blue_cube"
    assert w.occupant_of("zone_c") == "green_cube"
    assert w.free_zones() == ["zone_a"]
    assert w.location_of("red_cube") == "table"
    w2 = world(purple_cube="missing")
    assert w2.location_of("purple_cube") == "not detected"


# ----------------------------- validator: hợp lệ -----------------------------
def test_basic_plan_passes():
    r = v({"plan": [{"skill": "check_zone", "zone": "zone_a"}, pick("red_cube"),
                    place("red_cube", "zone_a"), HOME]})
    assert r.ok, r.errors
    assert [format_step(s) for s in r.plan] == [
        "check_zone(zone_a)", "pick(red_cube)", "place(red_cube, zone_a)", "home()"]
    assert r.inserted == []


def test_home_appended_and_case_normalized():
    r = v({"plan": [{"skill": "PICK", "object": "Red_Cube"},
                    {"skill": "place", "object": "red_cube", "zone": "ZONE_A"}]})
    assert r.ok, r.errors
    assert r.plan[-1] == HOME and r.warnings


def test_llm_clears_occupied_zone_itself():
    """Kế hoạch đúng như đề: dời blue_cube ra vị trí tạm rồi mới đặt red_cube vào zone_b."""
    r = v({"plan": [{"skill": "check_zone", "zone": "zone_b"}, pick("blue_cube"),
                    place("blue_cube", TEMP_POSITION), pick("red_cube"),
                    place("red_cube", "zone_b"), HOME]})
    assert r.ok, r.errors
    assert r.inserted == []


def test_validator_inserts_clearing_steps_for_occupied_zone():
    r = v({"plan": [pick("red_cube"), place("red_cube", "zone_b"), HOME]})
    assert r.ok, r.errors
    assert [format_step(s) for s in r.plan] == [
        "check_zone(zone_b)", "pick(blue_cube)", "place(blue_cube, temporary_position)",
        "pick(red_cube)", "place(red_cube, zone_b)", "home()"]
    assert r.inserted == [0, 1, 2]
    assert any("occupied by blue_cube" in w for w in r.warnings)


def test_occupied_zone_rejected_when_auto_resolve_off():
    r = v({"plan": [pick("red_cube"), place("red_cube", "zone_b"), HOME]}, auto_resolve=False)
    assert not r.ok and "ZONE_OCCUPIED" in r.errors[0]


def test_temp_position_aliases():
    for alias in ("temp", "temp_zone", "free_space", "temporary position"):
        r = v({"plan": [pick("red_cube"), place("red_cube", alias), HOME]})
        assert r.ok, (alias, r.errors)
        assert r.plan[1]["zone"] == TEMP_POSITION


def test_arrangement_with_blockers():
    # P=2: A<-yellow, B<-red, C<-blue; blue đang ở B, green ở C
    r = v({"plan": [pick("blue_cube"), place("blue_cube", TEMP_POSITION),
                    pick("green_cube"), place("green_cube", TEMP_POSITION),
                    pick("yellow_cube"), place("yellow_cube", "zone_a"),
                    pick("red_cube"), place("red_cube", "zone_b"),
                    pick("blue_cube"), place("blue_cube", "zone_c"), HOME]})
    assert r.ok, r.errors and r.inserted == []


# ----------------------------- validator: từ chối -----------------------------
def test_reject_unknown_skill():
    r = v({"plan": [{"skill": "throw", "object": "red_cube"}, HOME]})
    assert not r.ok and "not allowed" in r.errors[0]


def test_reject_invalid_object_and_zone():
    r = v({"plan": [pick("orange_cube"), place("orange_cube", "zone_d")]})
    assert not r.ok
    assert any("INVALID_OBJECT" in e for e in r.errors)
    assert any("INVALID_ZONE" in e for e in r.errors)


def test_temp_position_only_valid_for_place():
    r = v({"plan": [{"skill": "check_zone", "zone": TEMP_POSITION}, HOME]})
    assert not r.ok and "INVALID_ZONE" in r.errors[0]


def test_reject_joint_commands():
    r = v({"plan": [{"skill": "home", "joints": [0, 0, 0, 0, 0, 0]}]})
    assert not r.ok and "forbidden keys" in r.errors[0]
    r = v({"plan": [{"skill": "pick", "object": "red_cube", "trajectory": [[0, 1]]}]})
    assert not r.ok and "forbidden keys" in r.errors[0]


def test_reject_object_not_seen_by_camera():
    r = v({"plan": [pick("purple_cube"), place("purple_cube", "zone_a"), HOME]},
          world(purple_cube="missing"))
    assert not r.ok and "OBJECT_NOT_FOUND" in r.errors[0]


def test_reject_double_pick_and_place_without_pick():
    r = v({"plan": [pick("red_cube"), pick("blue_cube")]})
    assert not r.ok and "already holding" in r.errors[0]
    r = v({"plan": [place("red_cube", "zone_a")]})
    assert not r.ok and "not holding" in r.errors[0]


def test_reject_empty_and_garbage():
    assert not TaskValidator(SCENE).validate_text("Sure! I will move it.", world()).ok
    r = TaskValidator(SCENE).validate_text('{"plan": [], "message": "no orange cube"}', world())
    assert not r.ok and "no orange cube" in r.errors[0]


def test_extract_json_with_fences():
    assert extract_json('Here:\n```json\n{"plan": []}\n```') == {"plan": []}


def test_redundant_pick_place_is_pruned():
    r = v({"plan": [pick("blue_cube"), place("blue_cube", "zone_b"), HOME]})
    assert r.ok and r.plan == [HOME]
    assert any("redundant" in x.lower() for x in r.warnings)


def test_reject_non_adjacent_redundant_move():
    r = v({"plan": [pick("blue_cube"), {"skill": "check_zone", "zone": "zone_a"},
                    place("blue_cube", "zone_b"), HOME]})
    assert not r.ok and "REDUNDANT_MOVE" in r.errors[0]


# ----------------------------- camera: nhận dạng khối -----------------------------
def render_synthetic(cubes, width=640, height=480):
    """Vẽ ảnh nhìn từ camera trong scene.yaml: nền bàn + mặt trên của từng khối (đúng phối cảnh)."""
    det = CubeDetector(SCENE)
    hfov = SCENE.camera["horizontal_fov"]
    fx, fy, cx, cy = det.default_intrinsics(width, height, hfov)
    cam = np.array(SCENE.camera["pose"][:3])
    R = rpy_to_matrix(*SCENE.camera["pose"][3:6])
    img = np.full((height, width, 3), 205, np.uint8)
    u, vv = np.meshgrid(np.arange(width) + 0.5, np.arange(height) + 0.5)
    d = np.stack([np.ones_like(u), -(u - cx) / fx, -(vv - cy) / fy], -1) @ R.T
    top = SCENE.table_top_z + SCENE.cube_size
    t = (top - cam[2]) / d[..., 2]
    xy = cam[:2] + t[..., None] * d[..., :2]
    half = SCENE.cube_size / 2
    for name, (x, y, yaw) in cubes.items():
        dx, dy = xy[..., 0] - x, xy[..., 1] - y
        a = dx * math.cos(yaw) + dy * math.sin(yaw)
        b = -dx * math.sin(yaw) + dy * math.cos(yaw)
        m = (np.abs(a) <= half) & (np.abs(b) <= half)
        img[m] = (np.array(SCENE.objects[name].rgba[:3]) * 255).astype(np.uint8)
    return img


def test_detector_finds_all_cubes_and_zone_state():
    truth = {"red_cube": (0.24, 0.12, math.radians(15)), "yellow_cube": (0.25, -0.10, math.radians(-12)),
             "blue_cube": (0.36, 0.00, 0.0), "green_cube": (0.36, -0.15, math.radians(5)),
             "purple_cube": (0.27, 0.26, math.radians(30))}
    dets = CubeDetector(SCENE).detect(render_synthetic(truth))
    assert set(dets) == set(truth)
    for n, (x, y, yaw) in truth.items():
        d = dets[n]
        assert math.dist((d.x, d.y), (x, y)) < 0.003, n
        assert abs(d.yaw - yaw) < math.radians(3), (n, math.degrees(d.yaw))
    w = WorldState(SCENE, {n: d.pose for n, d in dets.items()})
    assert w.occupant_of("zone_b") == "blue_cube" and w.occupant_of("zone_a") is None


def test_detector_reports_missing_cube():
    dets = CubeDetector(SCENE).detect(render_synthetic({"red_cube": (0.3, 0.1, 0.0)}))
    assert set(dets) == {"red_cube"}


# ----------------------------- find_free_position -----------------------------
def test_free_position_is_clear_and_reachable():
    w = world()
    fs = SCENE.free_space
    p = find_free_position(SCENE, w, "blue_cube")
    assert p is not None
    for o, pos in w.positions.items():
        if o != "blue_cube":
            assert math.dist(p, pos[:2]) >= fs["min_dist_objects"] - 1e-9
    for z in SCENE.zones.values():
        assert math.dist(p, z.center_xy) >= fs["min_dist_zones"] - 1e-9
    assert math.hypot(*p) <= fs["max_reach"]


def test_free_position_none_when_table_full():
    xs = np.arange(0.18, 0.62, 0.05)
    ys = np.arange(-0.38, 0.38, 0.05)
    clutter = {f"o{i}": (x, y, 0.0) for i, (x, y) in enumerate((x, y) for x in xs for y in ys)}
    w = WorldState(SCENE, clutter)
    assert find_free_position(SCENE, w, None) is None


# ----------------------------- prompt + repair loop -----------------------------
def test_prompt_contains_everything():
    t = compute_student_task("Nguyen Van An", "23020123", SCENE)
    p = build_system_prompt(TEMPLATE, SCENE, t, world())
    for s in ("pick(object)", "place(object, zone)", "check_zone(zone)", "detect_objects()",
              "home()", "purple_cube", "zone_c", TEMP_POSITION, "P = 23 mod 6 = 5",
              "zone_a <- blue_cube", "zone_b: occupied by blue_cube"):
        assert s in p, s
    assert "{{" not in p
    # phần động (camera) nằm cuối để Ollama tái dùng KV-cache
    assert p.index("=== CURRENT WORLD STATE") > p.index("=== OUTPUT FORMAT")


def test_arrangement_status():
    t = compute_student_task("a", "23020762", SCENE)      # P=2: A<-yellow, B<-red, C<-blue
    p = build_system_prompt(TEMPLATE, SCENE, t, world(red_cube=zone("zone_b"), blue_cube=(0.25, 0, 0)))
    assert "red_cube: in zone_b, target zone_b -> already correct" in p
    assert "blue_cube: table, target zone_c -> MUST MOVE (zone_c is occupied by green_cube" in p


def test_repair_loop_fixes_bad_plan():
    t = compute_student_task("a", "23020123", SCENE)
    answers = iter([
        json.dumps({"plan": [pick("red")]}),                                     # sai tên
        json.dumps({"plan": [pick("red_cube"), place("red_cube", "zone_a"), HOME]}),
    ])
    calls = []

    def fake_chat(msgs):
        calls.append(msgs)
        return next(answers)

    planner = LLMPlanner(LLMConfig(max_repair_attempts=1), TEMPLATE, SCENE, t,
                         TaskValidator(SCENE), chat_fn=fake_chat)
    out = planner.plan("Đưa khối màu đỏ vào vùng A", world())
    assert out.result.ok and out.attempts == 2
    assert "REJECTED" in calls[1][-1]["content"]


if __name__ == "__main__":
    import pytest
    sys.exit(pytest.main([__file__, "-v"]))
