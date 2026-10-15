"""Timed UI fixtures, not a robot FSM. Authored with OpenAI Codex, 2026-10-11.

ROS field names/enums follow the pinned mtc_interfaces contract; see README.md.
All observations and results from this module are explicitly Mock data.
"""

from copy import deepcopy
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import threading
import time
import uuid


COLORS = {1: "红", 2: "蓝", 3: "黄", 4: "绿"}
ACTIVE = {"ACCEPTED", "EXECUTING", "CANCELING"}
SCENARIOS = {"success", "perception_failure", "timeout", "unconfirmed_stop"}
PHASES = [
    ("OBSERVING", "观察目标", 1.0),
    ("APPROACHING", "接近目标", 1.0),
    ("ATTACHING", "工具接合", 1.0),
    ("LIFTING", "提起观察", 2.0),
    ("TRANSFERRING", "搬运到目标位", 1.0),
    ("RELEASING", "释放目标", 1.0),
    ("VERIFYING_PLACEMENT", "放置稳定观察", 3.0),
]


def iso_time(seconds):
    return datetime.fromtimestamp(seconds, timezone.utc).isoformat(timespec="milliseconds")


class ApiError(Exception):
    def __init__(self, status, code, message):
        self.status, self.code, self.message = status, code, message
        super().__init__(message)


class MockBackend:
    """A thread-safe, server-owned fixture timeline; never imports a ROS client."""

    def __init__(self, output_dir=None, clock=time.monotonic, wall_clock=time.time):
        self.clock, self.wall_clock = clock, wall_clock
        self.lock = threading.RLock()
        self.session_id = str(uuid.uuid4())
        self.tasks, self.events = {}, []
        self.current_id = None
        self.seq, self.snapshot_seq = 0, 0
        self.condition = "normal"
        self.stop_confirmed = True
        self.stop_due = None
        self.output_dir = Path(output_dir) if output_dir else None
        if self.output_dir:
            self.output_dir.mkdir(parents=True, exist_ok=False)
        self._event("SESSION_STARTED", "Mock 操作台已启动，未连接 ROS 或实体设备")

    def _event(self, kind, message, task=None, level="info"):
        self.seq += 1
        event = {"seq": self.seq, "at": iso_time(self.wall_clock()), "kind": kind,
                 "level": level, "message": message, "mode": "mock",
                 "task_id": task["task_id"] if task else None,
                 "goal_id": task["goal_id"] if task else None}
        self.events.append(event)
        if task:
            task["events"].append(event)
        if self.output_dir:
            with (self.output_dir / "events.jsonl").open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(event, ensure_ascii=False) + "\n")
            if task:
                self._persist(task)

    def _public(self, task):
        return deepcopy({k: v for k, v in task.items() if not k.startswith("_")})

    def _persist(self, task):
        if self.output_dir:
            target = self.output_dir / (task["task_id"] + ".json")
            temp = target.with_suffix(".tmp")
            temp.write_text(json.dumps(self._public(task), ensure_ascii=False, indent=2) + "\n",
                            encoding="utf-8")
            temp.replace(target)

    @staticmethod
    def validate(payload):
        if not isinstance(payload, dict):
            raise ApiError(400, "INVALID_TASK", "请求必须是 JSON 对象")
        allowed = {"task_id", "mode", "ordered_colors", "timeout_ms", "placement_stability_sec", "scenario"}
        if set(payload) != allowed:
            raise ApiError(400, "INVALID_TASK", "任务字段不完整或包含未知字段")
        task_id = payload["task_id"]
        if not isinstance(task_id, str) or not re.fullmatch(r"[a-zA-Z0-9_-]{1,80}", task_id):
            raise ApiError(400, "INVALID_TASK", "任务 ID 需为 1–80 位字母、数字、下划线或短横线")
        mode, colors = payload["mode"], payload["ordered_colors"]
        if type(mode) is not int or mode not in (1, 2):
            raise ApiError(400, "INVALID_TASK", "任务模式无效")
        if (not isinstance(colors, list) or len(colors) != (1 if mode == 1 else 3)
                or any(type(c) is not int or c not in COLORS for c in colors)
                or len(set(colors)) != len(colors)):
            raise ApiError(400, "INVALID_TASK", "基础任务选择一种颜色；序列任务选择三种不同颜色")
        if type(payload["timeout_ms"]) is not int or not 1000 <= payload["timeout_ms"] <= 600000:
            raise ApiError(400, "INVALID_TASK", "本地演练超时范围为 1–600 秒")
        # This UI supports the recorded 3 s placement observation, never a shorter value.
        if type(payload["placement_stability_sec"]) not in (int, float) or payload["placement_stability_sec"] != 3:
            raise ApiError(400, "INVALID_TASK", "此操作台的放置稳定观察固定为 3 秒")
        if not isinstance(payload["scenario"], str) or payload["scenario"] not in SCENARIOS:
            raise ApiError(400, "INVALID_TASK", "Mock 场景无效")

    def submit(self, payload):
        with self.lock:
            self._advance()
            try:
                self.validate(payload)
                if payload["task_id"] in self.tasks:
                    raise ApiError(409, "DUPLICATE_TASK", "任务 ID 已使用；不会重复提交")
                current = self.tasks.get(self.current_id)
                if current and current["status"] in ACTIVE:
                    raise ApiError(409, "TASK_BUSY", "已有任务正在运行")
                if self.condition != "normal" or self.stop_confirmed is not True:
                    raise ApiError(409, "NOT_READY", "状态缺失、过期或尚未确认停稳，不能提交")
            except ApiError as error:
                self._event("TASK_REJECTED", error.message, level="warning")
                raise
            now = self.clock()
            task = {"task_id": payload["task_id"], "goal_id": str(uuid.uuid4()),
                    "mode": "mock", "goal": {k: v for k, v in payload.items() if k != "scenario"},
                    "scenario": payload["scenario"], "status": "ACCEPTED", "state": "READY",
                    "substate": "", "detail": "Mock Goal 已受理，等待执行",
                    "completed_count": 0, "total_count": len(payload["ordered_colors"]),
                    "completed_slots": [], "current_index": 0, "current_slot": "",
                    "current_object_id": "", "last_error_code": 0,
                    "created_at": iso_time(self.wall_clock()), "finished_at": None,
                    "elapsed_ms": 0, "result": None, "events": [], "recordings": [],
                    "_start": now, "_next": now + 0.5, "_phase": -1,
                    "_cancel_at": None, "_ended": None}
            self.tasks[task["task_id"]] = task
            self.current_id = task["task_id"]
            self._event("GOAL_ACCEPTED", task["detail"], task)
            return {"task_id": task["task_id"], "goal_id": task["goal_id"], "status": task["status"]}

    def _finish(self, task, status, code, message, ended):
        task.update(status=status, state=status, detail=message, last_error_code=code,
                    finished_at=iso_time(self.wall_clock()), _ended=ended,
                    elapsed_ms=round((ended - task["_start"]) * 1000))
        task["result"] = {"success": status == "SUCCEEDED", "completed_count": task["completed_count"],
                          "completed_slots": list(task["completed_slots"]),
                          "error_code": code, "message": message}
        self._event("RESULT", message, task, "error" if status == "ABORTED" else "info")

    def _advance(self):
        now = self.clock()
        task = self.tasks.get(self.current_id)
        if self.stop_due is not None and now >= self.stop_due:
            self.stop_due = None
            self.stop_confirmed = True
            self._event("STOP_CONFIRMED", "收到独立 Mock 停稳反馈", task)
        if not task or task["status"] not in ACTIVE:
            return
        task["elapsed_ms"] = round((now - task["_start"]) * 1000)
        if task["_cancel_at"] is not None:
            if now - task["_cancel_at"] >= 0.7:
                failed = task["scenario"] == "unconfirmed_stop"
                self._finish(task, "ABORTED" if failed else "CANCELED", 520 if failed else 0,
                             "取消后停稳未确认（Mock）" if failed else "任务已取消；停稳反馈单独记录（Mock）",
                             task["_cancel_at"] + 0.7)
            return
        deadline = task["_start"] + task["goal"]["timeout_ms"] / 1000
        # Replay every scheduled fixture even if the browser was disconnected.
        while task["status"] in ACTIVE and task["_next"] <= min(now, deadline):
            moment = task["_next"]
            if task["scenario"] == "perception_failure" and task["_phase"] == 0:
                self.stop_confirmed = True
                self._finish(task, "ABORTED", 110, "未找到目标颜色，Mock 演练中止", moment)
                return
            task["_phase"] += 1
            if task["_phase"] == len(PHASES):
                task["completed_count"] += 1
                task["completed_slots"].append(task["current_slot"])
                self._event("PLACEMENT_CONFIRMED", f"{task['current_slot']} 完成 3 秒 Mock 放置观察", task)
                if task["completed_count"] == task["total_count"]:
                    self.stop_confirmed = True
                    self._finish(task, "SUCCEEDED", 0, "全部目标放置完成（Mock 演示结果）", moment)
                    return
                task["_phase"] = 0
            index = task["completed_count"]
            phase, label, duration = PHASES[task["_phase"]]
            color = task["goal"]["ordered_colors"][index]
            task.update(status="EXECUTING", state="EXECUTING", substate=phase, current_index=index,
                        current_slot="T0" if task["goal"]["mode"] == 1 else f"P{index+1}",
                        current_object_id=f"mock-battery-{color}", detail=f"{COLORS[color]}色 · {label}（Mock）")
            self.stop_confirmed = phase in ("OBSERVING", "VERIFYING_PLACEMENT")
            task["_next"] = moment + duration
            self._event("FEEDBACK", task["detail"], task)
        if task["status"] in ACTIVE and now >= deadline:
            self.stop_confirmed = False
            self.stop_due = now + 1.0
            self._finish(task, "ABORTED", 220, "任务演练超时（Mock），等待独立停稳反馈", deadline)

    def cancel(self, task_id, goal_id):
        with self.lock:
            self._advance()
            task = self.tasks.get(task_id)
            if not task or task_id != self.current_id or task["goal_id"] != goal_id:
                raise ApiError(409, "GOAL_MISMATCH", "取消请求与当前 task/goal ID 不匹配")
            if task["status"] not in ("ACCEPTED", "EXECUTING"):
                raise ApiError(409, "NOT_CANCELABLE", "任务已终止或正在取消，不重复发送")
            task.update(status="CANCELING", state="CANCELING", detail="取消请求已送达，等待结果与停稳反馈",
                        _cancel_at=self.clock())
            self.stop_confirmed = False
            self.stop_due = None if task["scenario"] == "unconfirmed_stop" else self.clock() + 2.0
            self._event("CANCEL_DELIVERED", task["detail"], task)
            return {"request_delivered": True, "stop_confirmed": False, "goal_id": goal_id}

    def configure(self, condition):
        with self.lock:
            if condition not in ("normal", "stale", "missing"):
                raise ApiError(400, "INVALID_CONDITION", "未知数据条件")
            self.condition = condition
            self._event("DEMO_CONDITION", {"normal": "恢复 Mock 状态数据", "stale": "Mock 状态标记为过期",
                                           "missing": "Mock 状态标记为缺失"}[condition])

    def confirm_stop(self):
        with self.lock:
            self._advance()
            task = self.tasks.get(self.current_id)
            if not task or task["status"] in ACTIVE:
                raise ApiError(409, "TASK_ACTIVE", "演练任务终止后才能补发 Mock 停稳反馈")
            self.stop_confirmed, self.stop_due = True, None
            self._event("STOP_CONFIRMED", "手动补发 Mock 停稳反馈，仅用于界面演练", task)

    def snapshot(self):
        with self.lock:
            self._advance()
            self.snapshot_seq += 1
            task = self.tasks.get(self.current_id)
            active = bool(task and task["status"] in ACTIVE)
            age = 15000 if self.condition == "stale" else 0
            available = self.condition != "missing"
            def sample(data):
                return {"source": "webui.mock_backend", "received_at": iso_time(self.wall_clock() - age / 1000),
                        "age_ms": age if available else None, "available": available,
                        "data": data if available else None}
            phase = task["substate"] if active else ""
            attached = phase in ("LIFTING", "TRANSFERRING")
            robot = {"communication_ok": True, "robot_ready": True, "motion_active": active and not self.stop_confirmed,
                     "safety_normal": True, "stop_confirmed": self.stop_confirmed, "simulation_mode": True,
                     "controller_mode": "mock", "detail": "独立 Mock 数据，未接入机器人"}
            tool = {"state": 3 if attached else (2 if phase == "ATTACHING" else 1),
                    "evidence_level": 3 if attached else (1 if phase == "ATTACHING" else 0),
                    "verified": attached, "tool_type": "mock_tool", "object_id": task["current_object_id"] if attached else "",
                    "detail": "证据等级及验证标记均为演示样例，不代表实体末端"}
            return {"schema_version": 1, "mode": "mock", "session_id": self.session_id,
                    "snapshot_seq": self.snapshot_seq, "event_seq": self.seq,
                    "generated_at": iso_time(self.wall_clock()), "freshness_limit_ms": 3000,
                    "condition": self.condition, "task": self._public(task) if task else None,
                    "robot": sample(robot), "tool": sample(tool),
                    "camera": {"available": False, "source": None, "received_at": None, "age_ms": None},
                    "operations": {"submit": not active and self.stop_confirmed is True and self.condition == "normal",
                                   "cancel": bool(task and task["status"] in ("ACCEPTED", "EXECUTING")),
                                   "confirm_mock_stop": bool(task and not active and not self.stop_confirmed)},
                    "history": [{k: t[k] for k in ("task_id", "goal_id", "status", "created_at", "completed_count", "total_count", "result")}
                                for t in reversed(list(self.tasks.values()))],
                    "events": deepcopy(self.events[-80:])}

    def task_record(self, task_id):
        with self.lock:
            self._advance()
            if task_id not in self.tasks:
                raise ApiError(404, "NOT_FOUND", "找不到该任务")
            return self._public(self.tasks[task_id])

    def export_session(self):
        with self.lock:
            return {"schema_version": 1, "mode": "mock", "snapshot": self.snapshot(),
                    "tasks": [self._public(t) for t in self.tasks.values()], "events": deepcopy(self.events)}
