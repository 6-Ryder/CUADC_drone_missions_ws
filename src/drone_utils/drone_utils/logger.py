"""
任务日志 — 记录每次飞行的完整数据.

功能:
  - JSONL 格式持久化, 每条事件一行, 便于流式分析和回放
  - 自动按日期/任务ID 创建日志目录
  - 同时输出到 ROS2 logger (rclpy)
  - 记录: 状态转换、视觉检测、投放事件、异常、遥测快照

日志目录结构:
  /tmp/drone_mission_logs/
    └── 2026-07-07/
        └── mission_001/
            ├── events.jsonl      # 结构化事件流
            ├── telemetry.jsonl   # 遥测采样 (每 5s 一行)
            └── summary.json      # 任务完成后的摘要
"""

import json
import os
import time
from datetime import datetime
from typing import Any, Dict, Optional


class TaskLogger:
    """飞行任务日志记录器."""

    def __init__(
        self,
        base_dir: str = "/tmp/drone_mission_logs",
        node_name: str = "mission",
    ):
        self._base_dir = base_dir
        self._node_name = node_name

        # 按日期+序号创建任务目录
        date_str = datetime.now().strftime("%Y-%m-%d")
        date_dir = os.path.join(base_dir, date_str)
        os.makedirs(date_dir, exist_ok=True)

        # 查找今天已有任务数量, 递增序号
        existing = [
            d for d in os.listdir(date_dir)
            if d.startswith("mission_") and os.path.isdir(os.path.join(date_dir, d))
        ]
        self._mission_id = len(existing) + 1
        self._mission_dir = os.path.join(date_dir, f"mission_{self._mission_id:03d}")
        os.makedirs(self._mission_dir, exist_ok=True)

        self._events_path = os.path.join(self._mission_dir, "events.jsonl")
        self._telem_path = os.path.join(self._mission_dir, "telemetry.jsonl")
        self._summary_path = os.path.join(self._mission_dir, "summary.json")

        self._event_count = 0
        self._start_time = time.time()
        self._ros_logger = None  # 延迟注入

        # 写入任务头
        self._write_event("mission_start", {
            "node": node_name,
            "mission_id": self._mission_id,
            "timestamp_iso": datetime.now().isoformat(),
        })

    # ---- 注入 ROS logger (避免循环依赖) ----
    def set_ros_logger(self, ros_logger):
        """注入 rclpy logger 实例, 日志同时输出到 ROS2."""
        self._ros_logger = ros_logger

    # ---- 核心记录方法 ----

    def log(self, event: str, data: Optional[Dict[str, Any]] = None):
        """
        记录一条结构化事件.

        Parameters
        ----------
        event : str
            事件类型, e.g. "state_transition", "cylinder_detected", "drop_executed"
        data : dict or None
            事件负载

        """
        self._write_event(event, data or {})

    def log_state(self, from_state: str, to_state: str, reason: str = ""):
        """记录状态转换."""
        self.log("state_transition", {
            "from": from_state,
            "to": to_state,
            "reason": reason,
        })

    def log_detection(self, detection_type: str, count: int, details: dict = None):
        """记录视觉检测."""
        self.log(f"{detection_type}_detected", {
            "count": count,
            "details": details or {},
        })

    def log_drop(self, cylinder_id: int, success: bool, error: str = ""):
        """记录投放事件."""
        self.log("drop_executed", {
            "cylinder_id": cylinder_id,
            "success": success,
            "error": error,
        })

    def log_telemetry(self, telemetry: Dict[str, Any]):
        """记录遥测快照."""
        entry = {
            "t": time.time() - self._start_time,
            **telemetry,
        }
        try:
            with open(self._telem_path, "a") as f:
                f.write(json.dumps(entry, ensure_ascii=False) + "\n")
        except OSError:
            pass

    def log_error(self, error_type: str, message: str, context: dict = None):
        """记录错误/异常."""
        self.log("error", {
            "type": error_type,
            "message": message,
            "context": context or {},
        })

    def write_summary(self, summary: Dict[str, Any]):
        """
        任务结束时写入汇总 JSON.

        Parameters
        ----------
        summary : dict
            包含: success, total_duration, drops_completed, hazards_classified,
                  landing_precision, coverage_ratio, error_count, ...

        """
        summary["mission_id"] = self._mission_id
        summary["total_duration_s"] = time.time() - self._start_time
        summary["event_count"] = self._event_count
        summary["timestamp_iso"] = datetime.now().isoformat()

        try:
            with open(self._summary_path, "w") as f:
                json.dump(summary, f, indent=2, ensure_ascii=False)
        except OSError:
            pass

        self._write_event("mission_end", summary)

    # ---- 内部 ----

    def _write_event(self, event: str, data: dict):
        self._event_count += 1
        entry = {
            "seq": self._event_count,
            "t": round(time.time() - self._start_time, 3),
            "event": event,
            "data": data,
        }
        try:
            with open(self._events_path, "a") as f:
                f.write(json.dumps(entry, ensure_ascii=False) + "\n")
        except OSError:
            pass

        if self._ros_logger:
            self._ros_logger.info(f"[EVENT #{self._event_count}] {event}: {data}")

    # ---- 属性 ----
    @property
    def mission_dir(self) -> str:
        return self._mission_dir

    @property
    def mission_id(self) -> int:
        return self._mission_id
