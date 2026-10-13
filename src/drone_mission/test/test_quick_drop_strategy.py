"""快速投放策略 (quick_drop_mode) 契约测试.

覆盖:
  1. FSM 投放目标库门控 (drop_target_source=marker → 只返回蓝底标识)
  2. CYLINDER_SEARCH quick 模式候选计数 (只数 marker, 忽略圆筒记忆)
  3. DropState 超时兜底四态 (_quick_fallback_allow_drop)
  4. 参数声明存在性 (AST: 新参数默认值 = 现状行为)
  5. quick 参数文件 YAML 契约 (真机/SITL 独立 + 舵机预算)
"""

import ast
import time
import math
from pathlib import Path

import pytest
import yaml

from drone_mission.mission_state_machine import MissionFSM
from drone_mission.states.state_cylinder_search import CylinderSearchState
from drone_mission.states.state_drop import DropState
from drone_utils.layout_frame import LayoutFrame


ROOT = Path(__file__).resolve().parents[2]


class _Logger:
    def info(self, _message):
        pass

    def warn(self, _message):
        pass

    def error(self, _message):
        pass


def _load_parameters(path):
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    return next(iter(config.values()))["ros__parameters"]


# ---------------------------------------------------------------------------
# 1. FSM 投放目标库门控
# ---------------------------------------------------------------------------

def _bare_fsm():
    """object.__new__ 绕过重量级 __init__, 只提供目标库与 get_param."""
    fsm = object.__new__(MissionFSM)
    fsm._known_cylinders = [
        {
            "world_position_valid": True,
            "world_x": 0.0,
            "world_y": 0.0,
            "observations": 1,
        },
        {
            "world_position_valid": True,
            "world_x": 0.3,
            "world_y": 0.0,
            "observations": 1,
        },
    ]
    fsm._known_markers = [
        {
            "world_position_valid": True,
            "world_x": 1.0,
            "world_y": 1.0,
            "observations": 1,
        },
        {
            "world_position_valid": True,
            "world_x": 1.05,
            "world_y": 1.0,
            "observations": 1,  # 0.05m 重复 → 去重
        },
        {
            "world_position_valid": True,
            "world_x": 3.0,
            "world_y": 3.0,
            "observations": 1,
        },
    ]
    params = {"marker_memory_min_observations": 1}
    fsm.get_param = lambda name, default=None: params.get(name, default)
    fsm._quick_params = params
    return fsm


def test_marker_gate_returns_only_deduplicated_markers():
    fsm = _bare_fsm()
    fsm._quick_params["drop_target_source"] = "marker"

    targets = fsm.get_known_drop_targets()

    assert len(targets) == 2  # 3 个 marker, 1 个 0.05m 重复被去重
    assert all("world_x" in t for t in targets)


def test_marker_gate_ignores_cylinder_memory_even_when_present():
    fsm = _bare_fsm()
    fsm._quick_params["drop_target_source"] = "marker"

    targets = fsm.get_known_drop_targets()

    # 圆筒记忆里有 2 个候选, quick 下仍只返回 markers
    assert all(t["world_x"] >= 1.0 for t in targets)


def test_default_source_prefers_cylinders_then_falls_back_to_markers():
    fsm = _bare_fsm()

    targets = fsm.get_known_drop_targets()
    assert targets == fsm._known_cylinders

    fsm._known_cylinders = []
    targets = fsm.get_known_drop_targets()
    assert len(targets) == 2  # markers 去重兜底


# ---------------------------------------------------------------------------
# 2. CYLINDER_SEARCH quick 候选计数
# ---------------------------------------------------------------------------

class _SearchFSM:
    def __init__(self, params):
        self._params = params
        self.layout_frame = LayoutFrame(float(params.get("layout_heading_deg", 0.0)))
        self._known_cylinders = [
            {
                "world_position_valid": True,
                "world_x": 0.0,
                "world_y": 0.0,
                "observations": 2,
            },
        ]
        self._known_markers = [
            {
                "world_position_valid": True,
                "world_x": 1.0,
                "world_y": 1.0,
                "observations": 1,
            },
        ]
        self.set_vision_calls = []

    def get_param(self, name, default=None):
        return self._params.get(name, default)

    def zone_bounds(self, _prefix):
        return (30.0, 35.0, -4.0, 4.0)

    def get_cylinders(self):
        return []

    def get_markers(self):
        return []

    def get_known_cylinders(self):
        return self._known_cylinders

    def get_known_markers(self):
        return self._known_markers

    def get_logger(self):
        return _Logger()

    def set_vision_mode(self, mode):
        self.set_vision_calls.append(mode)


def _search_state(fsm):
    state = CylinderSearchState(fsm)
    state._start_time = time.time()  # elapsed=0, 不触发 10s 扫描分支
    state._min_cylinders_required = 1
    state._scan_phase = 0
    state._scan_wp_idx = 0
    state._detection_stable_count = 0
    state._last_cylinder_count = -1
    state._target_progress = None
    return state


def test_quick_search_counts_only_markers_ignoring_cylinder_memory():
    fsm = _SearchFSM({
        "quick_drop_mode": True,
        "min_drop_targets_required": 1,
        "drop_marker_fallback_delay": 0.0,
    })
    state = _search_state(fsm)

    result = None
    for _ in range(5):
        result = state.execute()

    # 圆筒记忆存在 (1 个) 但不计入; marker 1 个 ≥ 1 → 5 帧稳定后进入选择
    assert result == "SELECT_TARGET"


def test_quick_search_enters_requests_marker_only_vision():
    fsm = _SearchFSM({
        "quick_drop_mode": True,
        "min_drop_targets_required": 1,
        "drop_marker_fallback_delay": 0.0,
    })
    fsm.logger = type("TL", (), {"log_state": staticmethod(lambda *a: None)})()
    goto_calls = []
    fsm.fc = type("FC", (), {
        "goto_ned": staticmethod(lambda *a, **kw: goto_calls.append((a, kw))),
        "altitude_to_ned": staticmethod(lambda a: -a),
    })()
    fsm.layout_frame = LayoutFrame(50.0)
    fsm.zone_bounds = lambda _prefix: (30.0, 35.0, -4.0, 4.0)
    fsm.clear_known_cylinders = lambda: None
    state = CylinderSearchState(fsm)
    state.enter()

    assert fsm.set_vision_calls == ["marker_only"]
    north, east, _down = goto_calls[-1][0]
    assert fsm.layout_frame.ned_to_local(north, east) == pytest.approx((32.5, 0.0))


def test_bucket_search_runs_both_models_without_requiring_blue_pair():
    fsm = _SearchFSM({
        "quick_drop_mode": True,
        "quick_drop_profile": "bucket",
        "min_drop_targets_required": 1,
    })
    now = time.time()
    fsm._known_cylinders[0].update(
        world_x=1.0, world_y=1.0, radius=0.075,
        bucket_search_samples={now - 0.2: 0.075, now - 0.1: 0.075, now: 0.075})
    fsm.logger = type("TL", (), {"log_state": staticmethod(lambda *a: None)})()
    fsm.fc = type("FC", (), {
        "goto_ned": staticmethod(lambda *a, **kw: None),
        "altitude_to_ned": staticmethod(lambda a: -a),
    })()
    fsm.clear_known_cylinders = lambda: None
    state = CylinderSearchState(fsm)
    state.enter()
    assert fsm.set_vision_calls == ["all"]
    for _ in range(4):
        assert state.execute() is None
    assert state.execute() == "SELECT_TARGET"
    fsm._known_markers.append(dict(world_x=1.1, world_y=1.0))
    state = _search_state(fsm)
    for _ in range(5):
        result = state.execute()
    assert result == "SELECT_TARGET"


def test_bucket_search_waits_for_three_frames_then_accepts_other_bucket_size():
    fsm = _SearchFSM({
        "quick_drop_mode": True,
        "quick_drop_profile": "bucket",
        "min_drop_targets_required": 1,
    })
    fsm._known_cylinders[0].update(world_x=1.0, world_y=1.0, radius=0.10)
    state = _search_state(fsm)
    for _ in range(5):
        assert state.execute() is None

    fsm._known_cylinders.append({
        "world_position_valid": True,
        "world_x": 2.0,
        "world_y": 1.0,
        "radius": 0.12,
        "observations": 2,
        "bucket_search_samples": {
            time.time() - 0.2: 0.12,
            time.time() - 0.1: 0.12,
            time.time(): 0.12,
        },
    })
    fsm._known_markers.append({"world_x": 2.0, "world_y": 1.0})
    for _ in range(4):
        assert state.execute() is None
    assert state.execute() == "SELECT_TARGET"


def test_bucket_search_reports_rejected_targets_even_when_eligible_count_stays_zero():
    fsm = _SearchFSM({"quick_drop_mode": True, "quick_drop_profile": "bucket"})
    fsm._known_cylinders[0].update(radius=0.10, small_radius_confirmed=False)
    messages = []
    fsm.get_logger = lambda: type("Log", (), {"info": staticmethod(messages.append)})()
    state = _search_state(fsm)
    state._last_cylinder_count = 0

    assert state.execute() is None
    diagnostic = next((m for m in messages if "BUCKET_SEARCH:" in m), "")
    assert "eligible=0" in diagnostic
    assert "search_frames=0" in diagnostic
    assert "r=0.100" in diagnostic


def test_search_exit_never_claims_target_selection_on_takeover():
    fsm = _SearchFSM({"quick_drop_mode": True, "quick_drop_profile": "bucket"})
    fsm.get_known_drop_targets = fsm.get_known_cylinders
    events = []
    messages = []
    fsm.logger = type("Log", (), {"log_state": staticmethod(lambda *a: events.append(a))})()
    fsm.get_logger = lambda: type("Log", (), {"info": staticmethod(messages.append)})()

    _search_state(fsm).exit()

    assert not any(len(event) >= 2 and event[1] == "SELECT_TARGET" for event in events)
    assert messages


def test_bucket_memory_keeps_small_rim_evidence_amid_disc_radius_outlier():
    fsm = _bare_fsm()
    fsm._known_cylinders = []
    fsm._next_known_cylinder_id = 1
    fsm.validate_drop_target_world_xy = lambda x, y: (x, y)
    for radius in (0.077, 0.078, 0.46, 0.076, 0.078, 0.077):
        fsm._remember_cylinders([{
            "confidence": 0.9,
            "world_position_valid": True,
            "world_x": 1.0,
            "world_y": 2.0,
            "radius": radius,
        }])
    target = fsm.get_known_cylinders()[0]
    assert target["radius"] > 0.09  # all-sample mean was misleading
    assert target["small_radius_confirmed"]


def test_default_search_does_not_request_marker_only():
    fsm = _SearchFSM({
        "quick_drop_mode": False,
        "min_drop_targets_required": 2,
    })
    fsm.logger = type("TL", (), {"log_state": staticmethod(lambda *a: None)})()
    fsm.fc = type("FC", (), {
        "goto_ned": staticmethod(lambda *a, **kw: None),
        "altitude_to_ned": staticmethod(lambda a: -a),
    })()
    fsm.clear_known_cylinders = lambda: None
    state = CylinderSearchState(fsm)
    state.enter()

    assert fsm.set_vision_calls == []


# ---------------------------------------------------------------------------
# 3. DropState 超时兜底四态
# ---------------------------------------------------------------------------

class _FixedClock:
    """固定时钟桩 — 真 FSM 由 get_clock() 提供时间戳.

    伺服用例只验证单帧 P 行为与分层下降速度, 时间不需要推进:
    PID 因此只体现 P 项, 与迁移前纯 P 的数值期望一致.
    """

    class _Now:
        nanoseconds = 1_000_000_000

    def now(self):
        return self._Now()


class _DropFSM:
    def __init__(self, params=None):
        self._params = params or {}

    def get_param(self, name, default=None):
        return self._params.get(name, default)

    def get_clock(self):
        return _FixedClock()

    def vision_measurement_stamp(self, _source):
        """测量时刻. 固定时钟 => 恒为同一时刻, 即"同一份测量在多个 tick 上
        被复用"; 与 _FixedClock 的意图一致 —— 这些用例只验证单帧 P 行为."""
        return self.get_clock().now().nanoseconds / 1e9

    def get_logger(self):
        return _Logger()


def _status(pixel_error):
    return type("Status", (), {"pixel_error": pixel_error})()


def test_fallback_disabled_when_quick_off():
    state = DropState(_DropFSM({"precision_align_fallback_max_pixel_error": 60.0}))
    assert state._quick_fallback_allow_drop(_status(30.0)) is False


def test_fallback_disabled_when_limit_zero():
    state = DropState(_DropFSM({"quick_drop_mode": True}))  # 默认门限 0.0 = 禁用
    assert state._quick_fallback_allow_drop(_status(30.0)) is False


def test_fallback_rejects_nonfinite_pixel_error():
    state = DropState(_DropFSM({
        "quick_drop_mode": True,
        "precision_align_fallback_max_pixel_error": 60.0,
    }))
    # TARGET_LOST 时 pixel_error=NaN, 兜底必须拒绝 (不盲投)
    assert state._quick_fallback_allow_drop(_status(math.nan)) is False


def test_fallback_allows_drop_only_within_pixel_limit():
    state = DropState(_DropFSM({
        "quick_drop_mode": True,
        "precision_align_fallback_max_pixel_error": 60.0,
    }))
    assert state._quick_fallback_allow_drop(_status(60.0)) is True
    assert state._quick_fallback_allow_drop(_status(59.5)) is True
    assert state._quick_fallback_allow_drop(_status(60.1)) is False
    assert state._quick_fallback_allow_drop(_status(200.0)) is False


def test_descend_speed_is_parameterized():
    fsm = _DropFSM({"drop_descend_speed": 0.8})
    state = DropState(fsm)
    assert float(state._fsm.get_param("drop_descend_speed", 0.5)) == 0.8
    # 默认 0.5 (原行为不变)
    assert float(DropState(_DropFSM())._fsm.get_param("drop_descend_speed", 0.5)) == 0.5


# ---------------------------------------------------------------------------
# 4. 参数声明存在性 (AST)
# ---------------------------------------------------------------------------

def _declare_param_defaults(source_path):
    tree = ast.parse(source_path.read_text(encoding="utf-8"))
    defaults = {}
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call):
            continue
        func = call.func
        if not (isinstance(func, ast.Attribute) and func.attr == "declare_parameter"):
            continue
        if len(call.args) < 2:
            continue
        name, default = call.args[0], call.args[1]
        if isinstance(name, ast.Constant) and isinstance(default, ast.Constant):
            defaults[name.value] = default.value
    return defaults


def test_quick_params_declared_with_current_behavior_defaults():
    defaults = _declare_param_defaults(
        ROOT / "drone_mission/drone_mission/mission_state_machine.py"
    )

    assert defaults["quick_drop_mode"] is False
    assert defaults["drop_target_source"] == "cylinder"
    assert defaults["drop_align_stall_threshold"] == 40.0
    assert defaults["drop_align_stall_stable_band"] == 5.0
    assert defaults["drop_align_stall_duration"] == 2.0
    assert defaults["precision_align_fallback_max_pixel_error"] == 0.0
    assert defaults["drop_descend_speed"] == 0.5


# ---------------------------------------------------------------------------
# 5. quick 参数文件 YAML 契约 (真机 / SITL 独立 + 舵机预算)
# ---------------------------------------------------------------------------

def _quick_params(name):
    return _load_parameters(ROOT / "drone_mission/config" / name)


def _servo_quick_params():
    return _load_parameters(ROOT / "drone_servo2/config/servo_params_quick.yaml")


@pytest.mark.parametrize("name", ["mission_params_quick.yaml", "mission_params_quick_sim.yaml"])
def test_quick_search_height_is_four_point_five_metres(name):
    assert _quick_params(name)["area1_search_altitude"] == 4.5


def test_quick_real_params_enable_strategy():
    p = _quick_params("mission_params_quick.yaml")

    assert p["quick_drop_mode"] is True
    assert p["drop_target_source"] == "marker"
    assert p["precision_align_target_source"] == "marker"
    assert p["precision_align_final_descent_enable"] is False
    assert p["drop_descend_height"] == 2.4
    assert p["drop_descend_speed"] == 0.8
    assert p["max_drops"] == 1
    assert p["min_drop_targets_required"] == 1
    assert p["marker_memory_min_observations"] == 1
    assert p["drop_marker_fallback_delay"] == 0.0


def test_quick_sim_params_enable_strategy_with_scaled_pixel_windows():
    p = _quick_params("mission_params_quick_sim.yaml")

    assert p["quick_drop_mode"] is True
    assert p["drop_target_source"] == "marker"
    assert p["precision_align_final_descent_enable"] is False
    assert p["drop_descend_height"] == 2.4
    assert p["drop_descend_speed"] == 0.8
    assert p["max_drops"] == 1
    # SITL 1280 宽 = 真机 640 宽 × 2 → 像素窗口等比放大, 角度对心效果一致
    real = _quick_params("mission_params_quick.yaml")
    assert p["precision_align_raw_pixel_tolerance"] == (
        real["precision_align_raw_pixel_tolerance"] * 2
    )
    assert p["precision_align_smooth_pixel_tolerance"] == (
        real["precision_align_smooth_pixel_tolerance"] * 2
    )
    assert p["precision_align_fallback_max_pixel_error"] == (
        real["precision_align_fallback_max_pixel_error"] * 2
    )
    assert p["precision_align_threshold_norm_x"] == real["precision_align_threshold_norm_x"]
    assert p["precision_align_threshold_norm_y"] == real["precision_align_threshold_norm_y"]


def test_quick_servo_budget_and_drop_count_contract():
    mission_real = _quick_params("mission_params_quick.yaml")
    servo = _servo_quick_params()

    positions = servo["pwm_positions"]
    # 一次全开双投: 只有两个位置, max_drops = 长度-1 = 1
    assert positions == [1150.0, 2150.0]
    assert mission_real["max_drops"] == len(positions) - 1
    # 预算: drop_timeout_s 3.0 < FSM 确认超时 5.0
    assert mission_real["drop_confirmation_timeout"] > servo["drop_timeout_s"]
    assert servo["drop_timeout_s"] - servo["drop_delay_ms"] / 1000.0 >= 2.0


def test_quick_stall_threshold_matches_relaxed_window():
    real = _quick_params("mission_params_quick.yaml")
    assert real["drop_align_stall_threshold"] == (
        real["precision_align_raw_pixel_tolerance"] + 5.0
    )
    sim = _quick_params("mission_params_quick_sim.yaml")
    assert sim["drop_align_stall_threshold"] == (
        sim["precision_align_raw_pixel_tolerance"] + 5.0
    )


def test_quick_fallback_limit_matches_user_decision():
    real = _quick_params("mission_params_quick.yaml")
    assert real["precision_align_fallback_max_pixel_error"] == 60.0
    assert real["precision_align_max_retries"] == 1
    assert real["precision_align_timeout"] == 15.0


def test_quick_real_and_sim_are_independent():
    real = _quick_params("mission_params_quick.yaml")
    sim = _quick_params("mission_params_quick_sim.yaml")

    # 话题: 真机 v1 (launch 覆盖 v4 仅 SITL); 图像: 真机 640x480, SITL 1280x960
    assert real["vehicle_status_topic"] == "/fmu/out/vehicle_status_v1"
    assert sim["vehicle_status_topic"] == "/fmu/out/vehicle_status_v1"  # SITL launch 覆盖
    assert real["precision_align_image_width"] == 640.0
    assert sim["precision_align_image_width"] == 1280.0
    assert real["camera_fx"] == 400.0
    assert sim["camera_fx"] == 539.9
    # 相机朝向 (image_top_faces_forward) 与各自基线一致
    assert real["image_top_faces_forward"] is True
    assert sim["image_top_faces_forward"] is True


def test_base_params_untouched_by_quick_strategy():
    """原参数文件无 quick 参数 → 默认 False, 行为零漂移."""
    base = _load_parameters(ROOT / "drone_mission/config/mission_params.yaml")
    base_real = _load_parameters(
        ROOT / "drone_mission/config/mission_params_real.yaml"
    )

    assert "quick_drop_mode" not in base
    assert "quick_drop_mode" not in base_real
    assert "drop_align_stall_threshold" not in base
    assert "drop_align_stall_threshold" not in base_real


# ---------------------------------------------------------------------------
# 6. 边伺服边下降 (quick 投放区降高段: 像素 P 控制水平 + 分层垂直)
# ---------------------------------------------------------------------------

class _ServoFC:
    def __init__(self, alt=5.6):
        self.alt = alt
        self.calls = []
        self.goto_calls = 0

    def ned_to_altitude(self, down):
        return float(self.alt)

    def altitude_to_ned(self, h):
        return -float(h)

    def set_velocity_body(self, f, r, d):
        self.calls.append((f, r, d))

    def stop_velocity(self):
        self.calls.append(("stop",))

    def goto_ned(self, *args, **kwargs):
        self.goto_calls += 1


class _ServoBridge:
    def __init__(self, alt=5.6):
        self._alt = alt

    def get_current_position(self):
        return type("P", (), {"north": 0.0, "east": 0.0, "down": -self._alt})()


def _marker(u, v, conf=0.9):
    return type("M", (), {"center_u": u, "center_v": v, "confidence": conf})()


def _servo_fsm(params):
    fsm = _DropFSM(params)
    fsm.fc = _ServoFC()
    fsm.bridge = _ServoBridge()
    fsm.get_markers = lambda: [_marker(640, 480)]
    fsm.cylinder_selector = type("CS", (), {"_selected_positions": []})()
    return fsm


def test_quick_descend_uses_servo_not_goto():
    fsm = _servo_fsm({"quick_drop_mode": True, "drop_descend_height": 2.4})
    state = DropState(fsm)
    state.enter()
    assert state._servo_descend_active is True
    assert fsm.fc.goto_calls == 0  # 不发纯垂直 goto


def test_default_descend_keeps_goto():
    fsm = _servo_fsm({})
    state = DropState(fsm)
    state.enter()
    assert state._servo_descend_active is False
    assert fsm.fc.goto_calls == 1  # 非 quick 保持原行为


def test_servo_descend_tick_p_control_and_fast_layer():
    # 像素域增益名已迁移 (原 landing_servo_kp_pixel / max_horizontal_vel),
    # 数值期望保持不变.
    fsm = _servo_fsm({
        "camera_cx": 640.0, "camera_cy": 480.0,
        "precision_align_min_confidence": 0.25,
        "servo_px_kp": 0.004,
        "servo_px_max_speed": 0.6,
        "landing_servo_deadzone_px": 12.0,
        "landing_servo_x_to_right_sign": 1.0,
        "landing_servo_y_to_forward_sign": -1.0,
        "drop_servo_descend_fast": 0.5,
        "drop_servo_descend_slow": 0.3,
    })
    fsm.get_markers = lambda: [_marker(640 + 50, 480)]  # du=50 → right=0.2
    state = DropState(fsm)
    state._servo_descend_tick(-5.6)  # >3m → fast 层
    fwd, right, d = fsm.fc.calls[0]
    assert fwd == 0.0
    assert abs(right - 0.2) < 1e-9
    assert d == 0.5


def test_servo_descend_tick_slow_layer_and_deadzone():
    fsm = _servo_fsm({
        "camera_cx": 640.0, "camera_cy": 480.0,
        "servo_px_kp": 0.004,
        "servo_px_max_speed": 0.6,
        "landing_servo_deadzone_px": 12.0,
        "drop_servo_descend_fast": 0.5,
        "drop_servo_descend_slow": 0.3,
    })
    fsm.fc = _ServoFC(alt=2.5)  # ≤3m → slow 层
    fsm.get_markers = lambda: [_marker(640, 480)]  # 中心 → 死区内零水平修正
    state = DropState(fsm)
    state._servo_descend_tick(-2.5)
    fwd, right, d = fsm.fc.calls[0]
    assert fwd == 0.0
    assert right == 0.0
    assert d == 0.3


def test_servo_descend_tick_lost_hovers():
    fsm = _servo_fsm({
        "camera_cx": 640.0, "camera_cy": 480.0,
        "precision_align_min_confidence": 0.25,
        "drop_servo_descend_fast": 0.5,
        "drop_servo_descend_slow": 0.3,
    })
    fsm.get_markers = lambda: []
    state = DropState(fsm)
    state._servo_descend_tick(-5.6)
    assert fsm.fc.calls[0] == (0.0, 0.0, 0.0)  # 丢失悬停等待恢复


def test_legacy_servo_descend_stops_when_arrived():
    fsm = _servo_fsm({
        "quick_drop_mode": True,
        "drop_descend_height": 2.4,
        "precision_align_enable": True,
        "precision_align_drop_height_tolerance": 0.5,
        "precision_align_final_descent_enable": False,
    })
    fsm.bridge = _ServoBridge(alt=2.0)  # 已到位 (误差 0.4 < 0.5)
    state = DropState(fsm)
    state.enter()
    assert state._servo_descend_active is True
    state._drop_target_is_centered = lambda: True  # 到位且已对心 → 直接投放
    result = state._execute_descend()
    assert result == "EXECUTE_DROP"
    assert state._servo_descend_active is False
    assert fsm.fc.calls[0] == ("stop",)  # 停速度悬停


def test_continuous_servo_descend_handoff_does_not_stop_velocity():
    fsm = _servo_fsm({
        "quick_drop_mode": True,
        "precision_align_continuous_enable": True,
        "drop_descend_height": 2.4,
        "precision_align_enable": True,
        "precision_align_drop_height_tolerance": 0.5,
        "precision_align_final_descent_enable": False,
    })
    fsm.bridge = _ServoBridge(alt=2.0)  # 已到位 (误差 0.4 < 0.5)
    state = DropState(fsm)
    state.enter()
    state._drop_target_is_centered = lambda: True  # 到位且已对心 → 直接投放

    result = state._execute_descend()

    assert result == "EXECUTE_DROP"
    assert state._servo_descend_active is False
    assert ("stop",) not in fsm.fc.calls


def test_servo_descend_params_declared():
    defaults = _declare_param_defaults(
        ROOT / "drone_mission/drone_mission/mission_state_machine.py"
    )
    assert defaults["drop_servo_descend_fast"] == 0.5
    assert defaults["drop_servo_descend_slow"] == 0.3


def test_quick_params_include_servo_descend_speeds():
    for name in ("mission_params_quick.yaml", "mission_params_quick_sim.yaml"):
        p = _quick_params(name)
        assert p["drop_servo_descend_fast"] > 0.0
        assert p["drop_servo_descend_slow"] > 0.0
        assert p["drop_servo_descend_fast"] > p["drop_servo_descend_slow"]


def test_quick_params_enable_continuous_ground_error_drop_servo():
    # 米制域 P 增益已迁移到 pid_tuning.yaml: 原 precision_align_ground_kp
    # 改名 servo_kp, 数值期望保持不变 (0.40). 每个 quick profile 都吃同一份
    # 增益 (4 个 launch 都把该文件排在参数列表最末位).
    tuning = _load_parameters(ROOT / "drone_mission/config/pid_tuning.yaml")
    for name in ("mission_params_quick.yaml", "mission_params_quick_sim.yaml"):
        p = _quick_params(name)
        assert p["precision_align_continuous_enable"] is True
        assert tuning["servo_kp"] == 0.40


def test_base_params_untouched_by_servo_descend():
    base = _load_parameters(ROOT / "drone_mission/config/mission_params.yaml")
    assert "drop_servo_descend_fast" not in base
    assert "drop_servo_descend_slow" not in base
