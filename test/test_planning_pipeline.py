"""Unit test offline (không cần ROS): Plan Validator, nhiệm vụ cá nhân, prompt, LLM repair loop.

Chạy:  cd ur3_llm_control && python3 -m pytest test -v
  hoặc trong workspace:  colcon test --packages-select ur3_llm_control
"""
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from ur3_llm_control.llm_planner import LLMConfig, LLMPlanner, build_system_prompt  # noqa: E402
from ur3_llm_control.scene_config import (WorldState, compute_student_task,  # noqa: E402
                                          load_scene)
from ur3_llm_control.task_validator import TaskValidator, extract_json, format_step  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..")
SCENE = load_scene(os.path.join(ROOT, "config", "scene.yaml"))
TEMPLATE = open(os.path.join(ROOT, "prompts", "system_prompt.txt"), encoding="utf-8").read()


def world():
    return WorldState(SCENE)


def v(plan_obj, w=None):
    return TaskValidator(SCENE).validate(plan_obj, w or world())


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


# ----------------------------- validator: hợp lệ -----------------------------
def test_basic_plan_passes():
    r = v({"plan": [{"skill": "pick", "object": "red_cube"},
                    {"skill": "place", "object": "red_cube", "zone": "zone_b"},
                    {"skill": "home"}]})
    assert r.ok, r.errors
    assert [format_step(s) for s in r.plan] == ["pick(red_cube)", "place(red_cube, zone_b)", "home()"]


def test_home_appended_and_case_normalized():
    r = v({"plan": [{"skill": "PICK", "object": "Red_Cube"},
                    {"skill": "place", "object": "red_cube", "zone": "ZONE_B"}]})
    assert r.ok, r.errors
    assert r.plan[-1] == {"skill": "home"}
    assert r.warnings


def test_advanced_arrangement_with_temp_zone():
    # red đang ở zone_a, blue cần vào zone_a -> dời red qua temp_zone trước
    w = world()
    w.positions["red_cube"] = SCENE.zones["zone_a"].center_xy
    plan = {"plan": [
        {"skill": "pick", "object": "red_cube"}, {"skill": "place", "object": "red_cube", "zone": "temp_zone"},
        {"skill": "pick", "object": "blue_cube"}, {"skill": "place", "object": "blue_cube", "zone": "zone_a"},
        {"skill": "pick", "object": "yellow_cube"}, {"skill": "place", "object": "yellow_cube", "zone": "zone_b"},
        {"skill": "pick", "object": "red_cube"}, {"skill": "place", "object": "red_cube", "zone": "zone_c"},
        {"skill": "home"}]}
    r = v(plan, w)
    assert r.ok, r.errors


# ----------------------------- validator: từ chối -----------------------------
def test_reject_unknown_skill():
    r = v({"plan": [{"skill": "throw", "object": "red_cube"}, {"skill": "home"}]})
    assert not r.ok and "not allowed" in r.errors[0]


def test_reject_invalid_object_and_zone():
    r = v({"plan": [{"skill": "pick", "object": "green_cube"},
                    {"skill": "place", "object": "green_cube", "zone": "zone_d"}]})
    assert not r.ok
    assert any("INVALID_OBJECT" in e for e in r.errors)
    assert any("INVALID_ZONE" in e for e in r.errors)


def test_reject_joint_commands():
    r = v({"plan": [{"skill": "home", "joints": [0, 0, 0, 0, 0, 0]}]})
    assert not r.ok and "forbidden keys" in r.errors[0]


def test_reject_occupied_zone():
    w = world()
    w.positions["yellow_cube"] = SCENE.zones["zone_b"].center_xy
    r = v({"plan": [{"skill": "pick", "object": "red_cube"},
                    {"skill": "place", "object": "red_cube", "zone": "zone_b"}, {"skill": "home"}]}, w)
    assert not r.ok and "ZONE_OCCUPIED" in r.errors[0]


def test_reject_double_pick_and_place_without_pick():
    r = v({"plan": [{"skill": "pick", "object": "red_cube"}, {"skill": "pick", "object": "blue_cube"}]})
    assert not r.ok and "already holding" in r.errors[0]
    r = v({"plan": [{"skill": "place", "object": "red_cube", "zone": "zone_a"}]})
    assert not r.ok and "not holding" in r.errors[0]


def test_reject_empty_and_garbage():
    assert not TaskValidator(SCENE).validate_text("Sure! I will move it.", world()).ok
    r = TaskValidator(SCENE).validate_text('{"plan": [], "message": "no green cube"}', world())
    assert not r.ok and "no green cube" in r.errors[0]


def test_extract_json_with_fences():
    assert extract_json('Here:\n```json\n{"plan": []}\n```') == {"plan": []}


def test_redundant_pick_place_is_pruned():
    w = world()
    w.positions["red_cube"] = SCENE.zones["zone_b"].center_xy
    r = v({"plan": [{"skill": "pick", "object": "red_cube"},
                    {"skill": "place", "object": "red_cube", "zone": "zone_b"}, {"skill": "home"}]}, w)
    assert r.ok and r.plan == [{"skill": "home"}]
    assert any("redundant" in x.lower() for x in r.warnings)


def test_pruning_keeps_real_steps_of_a_swap():
    w = world()
    w.positions["blue_cube"] = SCENE.zones["zone_a"].center_xy
    w.positions["red_cube"] = SCENE.zones["zone_b"].center_xy
    w.positions["yellow_cube"] = SCENE.zones["zone_c"].center_xy
    steps = [("blue_cube", "temp_zone"), ("yellow_cube", "zone_a"), ("blue_cube", "zone_c"),
             ("red_cube", "zone_b")]                      # bước cuối là thừa
    plan = {"plan": [x for o, z in steps for x in ({"skill": "pick", "object": o},
                                                     {"skill": "place", "object": o, "zone": z})]
            + [{"skill": "home"}]}
    r = v(plan, w)
    assert r.ok and len(r.plan) == 7 and r.plan[-2]["object"] == "blue_cube"


def test_reject_non_adjacent_redundant_move():
    w = world()
    w.positions["red_cube"] = SCENE.zones["zone_b"].center_xy
    r = v({"plan": [{"skill": "pick", "object": "red_cube"}, {"skill": "move_above", "object": "blue_cube"},
                    {"skill": "place", "object": "red_cube", "zone": "zone_b"}, {"skill": "home"}]}, w)
    assert not r.ok and "REDUNDANT_MOVE" in r.errors[0]


def test_swap_needs_temp_zone_and_passes():
    w = world()
    w.positions["blue_cube"] = SCENE.zones["zone_a"].center_xy
    w.positions["yellow_cube"] = SCENE.zones["zone_c"].center_xy
    plan = {"plan": [
        {"skill": "pick", "object": "blue_cube"}, {"skill": "place", "object": "blue_cube", "zone": "temp_zone"},
        {"skill": "pick", "object": "yellow_cube"}, {"skill": "place", "object": "yellow_cube", "zone": "zone_a"},
        {"skill": "pick", "object": "blue_cube"}, {"skill": "place", "object": "blue_cube", "zone": "zone_c"},
        {"skill": "home"}]}
    assert v(plan, w).ok


# ----------------------------- prompt + repair loop -----------------------------
def test_arrangement_status_flags_cycle():
    t = compute_student_task("a", "23020762", SCENE)      # P=2: A<-yellow, B<-red, C<-blue
    w = world()
    w.positions["blue_cube"] = SCENE.zones["zone_a"].center_xy
    w.positions["red_cube"] = SCENE.zones["zone_b"].center_xy
    w.positions["yellow_cube"] = SCENE.zones["zone_c"].center_xy
    p = build_system_prompt(TEMPLATE, SCENE, t, w)
    assert "red_cube: in zone_b, target zone_b -> already correct" in p
    assert "yellow_cube: in zone_c, target zone_a -> MUST MOVE" in p
    assert "(a cycle)" in p and "{{" not in p

def test_prompt_contains_everything():
    t = compute_student_task("Nguyen Van An", "23020123", SCENE)
    p = build_system_prompt(TEMPLATE, SCENE, t, world())
    for s in ("pick(object)", "place(object, zone)", "home()", "red_cube", "zone_c",
              "temp_zone", "P = 23 mod 6 = 5", "zone_a <- blue_cube"):
        assert s in p, s
    assert "{{" not in p


def test_repair_loop_fixes_bad_plan():
    t = compute_student_task("a", "23020123", SCENE)
    answers = iter([
        json.dumps({"plan": [{"skill": "pick", "object": "red"}]}),                  # sai tên
        json.dumps({"plan": [{"skill": "pick", "object": "red_cube"},
                             {"skill": "place", "object": "red_cube", "zone": "zone_b"},
                             {"skill": "home"}]}),
    ])
    calls = []

    def fake_chat(msgs):
        calls.append(msgs)
        return next(answers)

    planner = LLMPlanner(LLMConfig(max_repair_attempts=1), TEMPLATE, SCENE, t,
                         TaskValidator(SCENE), chat_fn=fake_chat)
    out = planner.plan("Đưa khối màu đỏ vào vùng B", world())
    assert out.result.ok and out.attempts == 2
    assert "REJECTED" in calls[1][-1]["content"]


if __name__ == "__main__":
    import pytest
    sys.exit(pytest.main([__file__, "-v"]))
