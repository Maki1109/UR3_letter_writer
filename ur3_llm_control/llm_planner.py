"""LLM Planner: câu lệnh tự nhiên -> kế hoạch JSON (qua 9Router hoặc Ollama).

- Dựng system prompt từ prompts/system_prompt.txt, điền danh sách skill,
  object, zone, trạng thái thế giới DO CAMERA ĐO và nhiệm vụ cá nhân theo MSSV.
- Gọi endpoint OpenAI-compatible /chat/completions (9Router) hoặc /api/chat (Ollama).
- Nếu Plan Validator từ chối, gửi lỗi lại cho LLM để nó tự sửa (tối đa
  max_repair_attempts lần). Sau đó vẫn sai -> từ chối thực thi.

Module không phụ thuộc rclpy (unit test được bằng cách inject hàm gọi LLM).
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional

import requests
import yaml

from .scene_config import SceneConfig, StudentTask, WorldState
from .task_validator import TaskValidator, ValidationResult, skills_prompt_block

ChatFn = Callable[[List[Dict[str, str]]], str]


@dataclass
class LLMConfig:
    base_url: str = "http://localhost:20128/v1"
    model: str = "kr/claude-sonnet-4.5"
    api_key_env: str = "NINEROUTER_API_KEY"
    temperature: float = 0.0
    max_tokens: int = 1024
    timeout_sec: float = 60.0
    max_repair_attempts: int = 1
    api_type: str = "openai"  # "openai" for 9Router, "ollama" for Ollama

    @staticmethod
    def load(path: str) -> "LLMConfig":
        with open(path, "r", encoding="utf-8") as f:
            d = yaml.safe_load(f) or {}
        return LLMConfig(**{k: v for k, v in d.items() if k in LLMConfig.__dataclass_fields__})


@dataclass
class PlanningOutcome:
    result: ValidationResult
    raw_responses: List[str] = field(default_factory=list)
    attempts: int = 0


class NineRouterClient:
    """Client for OpenAI-compatible API (9Router) and Ollama."""

    def __init__(self, cfg: LLMConfig):
        self.cfg = cfg
        self.api_key = os.environ.get(cfg.api_key_env, "")

    def chat(self, messages: List[Dict[str, str]], max_tokens: Optional[int] = None) -> str:
        if self.cfg.api_type.lower() == "ollama":
            return self._chat_ollama(messages, max_tokens)
        else:
            return self._chat_openai(messages, max_tokens)

    def _chat_openai(self, messages: List[Dict[str, str]], max_tokens: Optional[int] = None) -> str:
        """OpenAI-compatible API (9Router, or Ollama with /v1 compatibility)."""
        url = self.cfg.base_url.rstrip("/") + "/chat/completions"
        headers = {"Content-Type": "application/json"}
        if self.api_key:
            headers["Authorization"] = f"Bearer {self.api_key}"
        body = {
            "model": self.cfg.model,
            "messages": messages,
            "temperature": self.cfg.temperature,
            "max_tokens": max_tokens or self.cfg.max_tokens,
            "stream": False,
        }
        r = requests.post(url, json=body, headers=headers, timeout=self.cfg.timeout_sec)
        if r.status_code != 200:
            raise RuntimeError(f"LLM API HTTP {r.status_code}: {r.text[:300]}")
        data = r.json()
        try:
            return data["choices"][0]["message"]["content"]
        except (KeyError, IndexError, TypeError) as e:
            raise RuntimeError(f"Unexpected LLM response format: {str(data)[:300]}") from e

    def _chat_ollama(self, messages: List[Dict[str, str]], max_tokens: Optional[int] = None) -> str:
        """Ollama-specific API using /api/chat endpoint."""
        url = self.cfg.base_url.rstrip("/") + "/api/chat"
        headers = {"Content-Type": "application/json"}
        # Convert OpenAI messages format to Ollama format
        body = {
            "model": self.cfg.model,
            "messages": messages,
            "stream": False,
            "format": "json",
            "keep_alive": "30m",
            "options": {
                "temperature": self.cfg.temperature,
                "num_predict": max_tokens or self.cfg.max_tokens,
                "num_ctx": 4096,
            },
        }
        r = requests.post(url, json=body, headers=headers, timeout=self.cfg.timeout_sec)
        if r.status_code != 200:
            raise RuntimeError(f"Ollama HTTP {r.status_code}: {r.text[:300]}")
        data = r.json()
        try:
            return data["message"]["content"]
        except (KeyError, TypeError) as e:
            raise RuntimeError(f"Unexpected Ollama response format: {str(data)[:300]}") from e


def arrangement_status(student: StudentTask, world: WorldState) -> str:
    """So sánh trạng thái (camera) với bố trí đích theo MSSV (dữ kiện thuần tuý cho LLM)."""
    lines = []
    for zone, obj in student.zone_to_object.items():
        cur = world.location_of(obj)
        if cur == zone:
            lines.append(f"- {obj}: in {cur}, target {zone} -> already correct, do NOT move it")
            continue
        occ = world.occupant_of(zone)
        note = (f"{zone} is occupied by {occ}, so {occ} must leave first" if occ else f"{zone} is free")
        lines.append(f"- {obj}: {cur}, target {zone} -> MUST MOVE ({note})")
    return "\n".join(lines)


def build_system_prompt(template: str, scene: SceneConfig, student: StudentTask,
                        world: WorldState) -> str:
    objects = "\n".join(f"- {o.name} ({o.color} cube)" for o in scene.objects.values())
    zones = "\n".join(f"- {z.name} ({z.label})" for z in scene.zones.values())
    arrangement = "\n".join(f"- {z} <- {o}" for z, o in student.zone_to_object.items())
    student_block = (f"Student: {student.student_name}, student ID {student.student_id}. "
                     f"Last two digits {student.last_two:02d}, P = {student.last_two} mod 6 = "
                     f"{student.p}.\nTarget arrangement:\n{arrangement}")
    # Phần tĩnh trước, phần động (trạng thái camera) sau cùng -> Ollama tái dùng KV-cache
    return (template
            .replace("{{SKILLS}}", skills_prompt_block())
            .replace("{{OBJECTS}}", objects)
            .replace("{{ZONES}}", zones)
            .replace("{{STUDENT}}", student_block)
            .replace("{{WORLD_STATE}}", world.describe())
            .replace("{{ARRANGEMENT_STATUS}}", arrangement_status(student, world)))


class LLMPlanner:
    def __init__(self, cfg: LLMConfig, prompt_template: str, scene: SceneConfig,
                 student: StudentTask, validator: TaskValidator,
                 chat_fn: Optional[ChatFn] = None):
        self.cfg = cfg
        self.template = prompt_template
        self.scene = scene
        self.student = student
        self.validator = validator
        self.client = None if chat_fn else NineRouterClient(cfg)
        self.chat_fn: ChatFn = chat_fn or self.client.chat

    def warmup(self, world: WorldState) -> None:
        """Nạp mô hình + KV-cache phần prompt tĩnh (chạy nền lúc khởi động) để lệnh đầu tiên nhanh."""
        if self.client is None:
            return
        system = build_system_prompt(self.template, self.scene, self.student, world)
        self.client.chat([{"role": "system", "content": system},
                          {"role": "user", "content": "ping"}], max_tokens=1)

    def plan(self, command: str, world: WorldState) -> PlanningOutcome:
        messages = [
            {"role": "system",
             "content": build_system_prompt(self.template, self.scene, self.student, world)},
            {"role": "user", "content": command},
        ]
        outcome = PlanningOutcome(result=ValidationResult(False))
        for attempt in range(1 + max(0, int(self.cfg.max_repair_attempts))):
            outcome.attempts = attempt + 1
            try:
                text = self.chat_fn(messages)
            except Exception as e:  # lỗi mạng / 9Router chưa chạy / sai key...
                outcome.result = ValidationResult(False, errors=[f"LLM call failed: {e}"])
                return outcome
            outcome.raw_responses.append(text)
            res = self.validator.validate_text(text, world)
            outcome.result = res
            if res.ok:
                return outcome
            # LLM chủ động từ chối (plan rỗng) -> không ép sửa
            if res.errors and res.errors[0].startswith("LLM returned an empty plan"):
                return outcome
            messages += [
                {"role": "assistant", "content": text},
                {"role": "user", "content":
                    "Your plan was REJECTED by the validator:\n- " + "\n- ".join(res.errors)
                    + "\nReturn a corrected plan as ONE JSON object only."},
            ]
        return outcome
