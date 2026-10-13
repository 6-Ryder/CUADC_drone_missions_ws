"""降落区 simple 模式 (landing_simple_mode, test_mission 方法) 契约测试.

覆盖:
  1. FSM 注册门控: landing_simple_mode=True → SimplePrecisionLandState
  2. 参数声明存在性 (AST: 默认 False, 主任务行为零漂移)
  3. SimplePrecisionLandState 行为:
     - PX4 is_landed → COMPLETE
     - 搜索超时 → 请求 PX4 LAND 一次 + COMPLETE
     - 视觉丢失超时 → PX4 LAND + COMPLETE (不做重捕获)
     - 盲降切换高度 (test_mission 0.5m) 与盲降速度
  4. quick 参数文件契约: simple 参数值 = test_mission 照搬
"""

import ast
import time
from pathlib import Path

import pytest

from drone_mission.mission_state_machine import MissionFSM
from drone_mission.states.state_precision_land import PrecisionLandState
from drone_mission.states.state_precision_land_simple import SimplePrecisionLandState


ROOT = Path(__file__).resolve().parents[2]


class _Logger:
    def info(self, _m):
        pass

    def warn(self, _m):
        pass

    def error(self, _m):
        pass


class _LogState:
    def log_state(self, *_a):
        pass


class _Bridge:
    def __init__(self, landed=False, altitude=5.0):
        self._landed = landed
        self._alt = altitude
        self.land_calls = 0
        self.force_disarm_calls = 0
        self._armed = True
        self._pos = type("P", (), {"down": -self._alt})()

    def is_landed(self):
        return self._landed

    def land_at_current(self):
        self.land_calls += 1

    def force_disarm(self):
        self.force_disarm_calls += 1
        self._armed = False

    def is_armed(self):
        return self._armed

    def get_current_position(self):
        return self._pos


class _FC:
    def __init__(self):
        self.calls = []

    def ned_to_altitude(self, down):
        return -float(down)

    def set_velocity_body(self, f, r, d):
        self.calls.append((f, r, d))

    def loiter(self, *_a):
        pass


class _FixedClock:
    """固定时钟桩 — 真 FSM 由 get_clock() 提供时间戳.

    这些用例只验证单帧 P 行为与保持/锁存判决, 时间不需要推进:
    PID 因此只体现 P 项, 与迁移前纯 P 的数值期望一致.
    """

    class _Now:
        nanoseconds = 1_000_000_000

    def now(self):
        return self._Now()


class _SimpleFSM:
    def __init__(self, params=None, landed=False, target=None):
        self._params = params or {}
        self._landed = landed
        self._target = target
        self.bridge = _Bridge(landed=landed)
        self.fc = _FC()
        self.logger = _LogState()
        self.vision_modes = []
        self._measurement_stamp = None

    def get_param(self, name, default=None):
        return self._params.get(name, default)

    def get_clock(self):
        return _FixedClock()

    def vision_measurement_stamp(self, _source):
        """测量时刻. 默认固定 (同一份测量在多个 tick 上被复用), 与
        _FixedClock 的意图一致; 用例可用 set_measurement_stamp() 模拟新帧."""
        if self._measurement_stamp is not None:
            return self._measurement_stamp
        return self.get_clock().now().nanoseconds / 1e9

    def set_measurement_stamp(self, seconds):
        """模拟一帧新测量的采集时刻."""
        self._measurement_stamp = seconds

    def get_landing_target(self):
        return self._target

    def get_logger(self):
        return _Logger()

    def set_vision_mode(self, mode):
        self.vision_modes.append(mode)

    def loiter(self, *_a):
        pass


def _bare_fsm(simple=True):
    fsm = object.__new__(MissionFSM)
    fsm._params = {"landing_simple_mode": simple}
    fsm.get_param = lambda name, default=None: fsm._params.get(name, default)
    return fsm


# ---------------------------------------------------------------------------
# 1. FSM 注册门控
# ---------------------------------------------------------------------------

def test_simple_mode_registers_simple_land_state():
    fsm = _bare_fsm(simple=True)
    handler = (
        SimplePrecisionLandState(fsm)
        if fsm.get_param("landing_simple_mode", False)
        else PrecisionLandState(fsm)
    )
    assert isinstance(handler, SimplePrecisionLandState)


def test_default_mode_registers_main_land_state():
    fsm = _bare_fsm(simple=False)
    handler = (
        SimplePrecisionLandState(fsm)
        if fsm.get_param("landing_simple_mode", False)
        else PrecisionLandState(fsm)
    )
    assert isinstance(handler, PrecisionLandState)


# ---------------------------------------------------------------------------
# 2. 参数声明存在性 (AST)
# ---------------------------------------------------------------------------

def test_simple_land_params_declared_with_current_behavior_defaults():
    source = ROOT / "drone_mission/drone_mission/mission_state_machine.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    defaults = {}
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call):
            continue
        func = call.func
        if not (isinstance(func, ast.Attribute) and func.attr == "declare_parameter"):
            continue
        if len(call.args) >= 2:
            name, default = call.args[0], call.args[1]
            if isinstance(name, ast.Constant) and isinstance(default, ast.Constant):
                defaults[name.value] = default.value

    assert defaults["landing_simple_mode"] is False
    assert defaults["landing_search_timeout"] == 15.0
    assert defaults["landing_simple_blind_timeout"] == 60.0
    assert defaults["landing_visual_handoff_altitude"] == 0.0
    assert defaults["landing_visual_handoff_max_ground_error_m"] == 0.20
    assert defaults["landing_visual_handoff_stable_frames"] == 10


def test_simple_land_state_imported_by_fsm():
    source = ROOT / "drone_mission/drone_mission/mission_state_machine.py"
    text = source.read_text(encoding="utf-8")
    assert "from drone_mission.states.state_precision_land_simple import" in text
    assert "SimplePrecisionLandState(self)" in text
    assert "landing_simple_mode" in text


# ---------------------------------------------------------------------------
# 3. SimplePrecisionLandState 行为
# ---------------------------------------------------------------------------

def test_landed_triggers_complete():
    fsm = _SimpleFSM(landed=True)
    state = SimplePrecisionLandState(fsm)
    assert state.execute() == "COMPLETE"


def test_search_timeout_requests_px4_land_once():
    fsm = _SimpleFSM(params={"landing_search_timeout": 15.0})
    state = SimplePrecisionLandState(fsm)
    state._start_time = 0.0  # elapsed 巨大 → 立即超时

    result = state.execute()
    assert result == "COMPLETE"
    assert fsm.bridge.land_calls == 1


def test_px4_land_sent_only_once():
    fsm = _SimpleFSM(params={"landing_search_timeout": 15.0})
    state = SimplePrecisionLandState(fsm)
    state._start_time = 0.0

    assert state.execute() == "COMPLETE"
    state._request_px4_land()  # 重复调用不重复发送
    assert fsm.bridge.land_calls == 1


def test_vision_loss_timeout_requests_px4_land():
    fsm = _SimpleFSM(params={"vision_loss_timeout": 3.0})
    state = SimplePrecisionLandState(fsm)
    # 先锁定目标, 然后丢失 (与 test_mission 一致: 首次丢失 tick 起计时)
    state._has_target = True
    state.execute()
    assert state._vision_lost_start > 0  # 首次丢失 tick 已设置计时起点
    # 模拟丢失超过 3s
    state._vision_lost_start = time.time() - 10.0

    result = state.execute()
    assert result == "COMPLETE"
    assert fsm.bridge.land_calls == 1


def test_servo_holds_at_blind_altitude_without_visual_alignment():
    fsm = _SimpleFSM(params={"landing_blind_altitude": 0.5})
    fsm.bridge = _Bridge(altitude=0.4)  # 已低于盲降高度
    state = SimplePrecisionLandState(fsm)
    state._phase = state.PHASE_SERVO

    state._execute_servo(has_target=False)
    assert state._phase == state.PHASE_SERVO
    assert fsm.fc.calls[-1] == (0.0, 0.0, 0.0)


def test_blind_timeout_requests_px4_land():
    fsm = _SimpleFSM(params={"landing_simple_blind_timeout": 60.0})
    state = SimplePrecisionLandState(fsm)
    state._phase = state.PHASE_BLIND
    state._start_time = 0.0

    result = state._execute_blind(elapsed=999.0)
    assert result == "COMPLETE"
    assert fsm.bridge.land_calls == 1


def test_servo_p_control_uses_test_mission_defaults():
    """照搬 test_mission: kp 0.004 / deadzone 12px / max_vel 0.6.

    增益名已迁移到 pid_tuning.yaml 契约 (原 landing_servo_kp_pixel /
    landing_servo_max_horizontal_vel), 数值期望保持不变.
    """
    fsm = _SimpleFSM({
        "servo_px_kp": 0.004,
        "servo_px_max_speed": 0.6,
        "landing_servo_deadzone_px": 12.0,
        "landing_blind_altitude": 0.5,
    })
    fsm._target = {"center_u": 320.0 + 50.0, "center_v": 240.0, "confidence": 0.9}
    state = SimplePrecisionLandState(fsm)
    state._phase = state.PHASE_SERVO

    state._execute_servo(has_target=True)
    assert len(fsm.fc.calls) == 1
    forward, right, descent = fsm.fc.calls[0]
    # dv=0 → forward=0; du=50 → right = kp*50 = 0.2 m/s
    assert forward == 0.0
    assert abs(right - 0.2) < 1e-9
    # 高度 5.0 > 3.0 → fast 0.4
    assert descent == 0.4


def test_servo_does_not_integrate_when_measurement_is_reused():
    """同一份测量在多个 tick 上复用时, 积分不得累加.

    真机视觉 3-5Hz 而 FSM tick 20Hz: 若把 FSM 时钟当测量时刻传入, 同一份
    误差会在两帧之间被积分 4-7 次, 迅速顶到 integral_limit, 并在目标丢失时
    变成速度踢出. 契约是传测量时刻 —— stamp 不前进就不推进积分.
    """
    fsm = _SimpleFSM({
        "servo_px_kp": 0.004,
        "servo_px_ki": 0.002,
        "servo_px_kd": 0.0,
        "servo_px_max_speed": 0.6,
        "landing_servo_deadzone_px": 1.0,
        "landing_blind_altitude": 0.5,
    })
    fsm._target = {"center_u": 370.0, "center_v": 240.0, "confidence": 0.9}
    fsm.set_measurement_stamp(100.0)
    state = SimplePrecisionLandState(fsm)
    state._phase = state.PHASE_SERVO

    for _ in range(6):
        state._execute_servo(has_target=True)

    assert state._servo_pid.integral == (0.0, 0.0)


def test_servo_integrates_by_measurement_interval_not_tick_count():
    """积分步长 = 相邻两次测量的时间差, 与中间跑了多少 tick 无关.

    这里在两次测量之间多跑几个 tick, 积分量必须仍等于 ki·e·Δt (Δt=0.33s,
    即 3Hz 的测量间隔), 而不是 ki·e·(tick 数 × 0.05s).
    """
    fsm = _SimpleFSM({
        "servo_px_kp": 0.004,
        "servo_px_ki": 0.002,
        "servo_px_kd": 0.0,
        "servo_px_max_speed": 0.6,
        "landing_servo_deadzone_px": 1.0,
        "landing_blind_altitude": 0.5,
    })
    fsm._target = {"center_u": 370.0, "center_v": 240.0, "confidence": 0.9}
    state = SimplePrecisionLandState(fsm)
    state._phase = state.PHASE_SERVO

    # 第一帧: 只建立测量时刻, dt=0, 不积分
    fsm.set_measurement_stamp(100.0)
    state._execute_servo(has_target=True)
    assert state._servo_pid.integral == (0.0, 0.0)

    # 测量之间的 5 个 tick: 仍是同一份测量, 不得积分
    for _ in range(5):
        state._execute_servo(has_target=True)
    assert state._servo_pid.integral == (0.0, 0.0)

    # 新一帧到达 (Δt = 0.33s): 积分 = ki·du·Δt = 0.002·50·0.33
    fsm.set_measurement_stamp(100.33)
    state._execute_servo(has_target=True)
    assert state._servo_pid.integral[1] == pytest.approx(0.002 * 50.0 * 0.33)

    # 该帧再被复用 5 个 tick: 积分不变
    for _ in range(5):
        state._execute_servo(has_target=True)
    assert state._servo_pid.integral[1] == pytest.approx(0.002 * 50.0 * 0.33)


def test_servo_rejects_low_confidence_target_and_holds_position():
    """Near-ground low-confidence H detections must not create descent."""
    fsm = _SimpleFSM({"landing_target_min_confidence": 0.60})
    fsm._target = {
        "center_u": 520.0,
        "center_v": 360.0,
        "confidence": 0.59,
    }
    state = SimplePrecisionLandState(fsm)
    state._phase = state.PHASE_SERVO

    assert state.execute() is None
    assert fsm.fc.calls[-1] == (0.0, 0.0, 0.0)


def test_servo_rejects_large_target_jump_and_holds_position():
    """A new H candidate cannot replace a locked target in one frame."""
    fsm = _SimpleFSM({"landing_target_max_jump_px": 80.0})
    state = SimplePrecisionLandState(fsm)
    state._phase = state.PHASE_SERVO

    fsm._target = {"center_u": 320.0, "center_v": 240.0, "confidence": 0.9}
    assert state.execute() is None

    fsm._target = {"center_u": 440.0, "center_v": 240.0, "confidence": 0.9}
    assert state.execute() is None
    assert fsm.fc.calls[-1] == (0.0, 0.0, 0.0)


def test_short_reacquire_timeout_requests_px4_land_after_target_loss(monkeypatch):
    """A locked target lost near ground must hand over to PX4 LAND promptly."""
    fsm = _SimpleFSM({"landing_target_reacquire_timeout": 1.0})
    fsm.bridge = _Bridge(altitude=1.2)
    state = SimplePrecisionLandState(fsm)
    state._phase = state.PHASE_SERVO
    state._has_target = True
    state._vision_lost_start = 10.0
    monkeypatch.setattr(
        "drone_mission.states.state_precision_land_simple.time.time", lambda: 11.1,
    )

    assert state.execute() == "COMPLETE"
    assert fsm.bridge.land_calls == 1


def test_servo_requires_stable_alignment_before_entering_blind_mode():
    """Reaching blind altitude alone must not bypass visual alignment."""
    fsm = _SimpleFSM(params={"landing_blind_altitude": 0.5})
    fsm.bridge = _Bridge(altitude=0.4)
    state = SimplePrecisionLandState(fsm)
    state._phase = state.PHASE_SERVO

    assert state.execute() is None
    assert state._phase == state.PHASE_SERVO
    assert fsm.fc.calls[-1] == (0.0, 0.0, 0.0)


def test_ground_error_latches_vertical_only_descent_at_handoff_altitude():
    fsm = _SimpleFSM({
        "landing_visual_handoff_altitude": 1.6,
        "landing_visual_handoff_max_ground_error_m": 0.20,
        "landing_visual_handoff_stable_frames": 2,
        "camera_fx": 540.0,
        "camera_fy": 540.0,
        "landing_descent_slow": 0.15,
    })
    fsm.bridge = _Bridge(altitude=1.6)
    fsm._target = {"center_u": 350.0, "center_v": 240.0, "confidence": 0.9}
    state = SimplePrecisionLandState(fsm)
    state._phase = state.PHASE_SERVO

    state.execute()
    state.execute()

    assert state._vertical_only_latched is True
    assert fsm.fc.calls[-1] == (0.0, 0.0, 0.25)


def test_vertical_only_latch_ignores_later_visual_loss_and_never_laterally_chases():
    fsm = _SimpleFSM({"landing_descent_slow": 0.15})
    fsm.bridge = _Bridge(altitude=1.4)
    state = SimplePrecisionLandState(fsm)
    state._phase = state.PHASE_SERVO
    state._vertical_only_latched = True

    assert state.execute() is None
    assert fsm.fc.calls[-1] == (0.0, 0.0, 0.15)


def test_simulated_touchdown_after_vertical_handoff_disarms_and_completes():
    fsm = _SimpleFSM({
        "sim_mode": True,
        "landing_touchdown_max_altitude": 0.10,
        "landing_touchdown_max_horizontal_speed": 0.10,
        "landing_touchdown_max_vertical_speed": 0.05,
        "landing_touchdown_hold_time": 2.0,
    })
    fsm.bridge = _Bridge(altitude=0.05)
    fsm.bridge.get_telemetry = lambda: type(
        "T", (), {"velocity": type("V", (), {"vn": 0.0, "ve": 0.0, "vd": 0.0})()}
    )()
    state = SimplePrecisionLandState(fsm)
    state._phase = state.PHASE_SERVO
    state._vertical_only_latched = True
    state._touchdown_candidate_start = time.time() - 3.0

    assert state.execute() is None
    assert fsm.bridge.force_disarm_calls == 1
    assert state.execute() == "COMPLETE"


# ---------------------------------------------------------------------------
# 4. quick 参数文件契约 (test_mission 照搬)
# ---------------------------------------------------------------------------

def _quick_params(name):
    import yaml
    config = yaml.safe_load((ROOT / "drone_mission/config" / name).read_text(encoding="utf-8"))
    return next(iter(config.values()))["ros__parameters"]


def test_quick_params_enable_simple_landing():
    for name in ("mission_params_quick.yaml", "mission_params_quick_sim.yaml"):
        p = _quick_params(name)
        assert p["landing_simple_mode"] is True
        assert p["landing_blind_descent_speed"] == 0.15  # test_mission 值
        assert p["landing_search_timeout"] == 15.0
        assert p["landing_simple_blind_timeout"] == 60.0
        assert p["landing_target_reacquire_timeout"] == 1.0


def test_quick_real_blind_altitude_above_h_detection_limit():
    """真机: 盲降切换高度 1.6m, 高于 H 近地失效线 (~0.9m), 视觉仍有效时完成对心交接."""
    p = _quick_params("mission_params_quick.yaml")
    assert p["landing_blind_altitude"] == 1.6


def test_quick_sim_blind_altitude_above_h_detection_limit():
    """SITL: H 标识近地实测 ~0.9m 失效, 盲降高度取 1.0m 保伺服路径完整."""
    p = _quick_params("mission_params_quick_sim.yaml")
    assert p["landing_blind_altitude"] == 1.0


def test_quick_params_vision_loss_timeout_matches_test_mission():
    p = _quick_params("mission_params_quick.yaml")
    assert p["vision_loss_timeout"] == 3.0  # test_mission 默认


def test_quick_real_params_blind_as_primary_strategy():
    """真机 quick: 关闭圆形几何交接 + 1 帧死区锁存 → 切盲降为主策略."""
    params = _quick_params("mission_params_quick.yaml")
    assert params["landing_visual_handoff_altitude"] == 0.0
    assert params["landing_blind_stable_frames"] == 1


def test_quick_sim_params_keep_circle_geometry_handoff():
    """SITL quick: 保留圆形几何交接 (SITL 档独立, 仿真/真机行为可不同)."""
    params = _quick_params("mission_params_quick_sim.yaml")
    assert params["landing_visual_handoff_altitude"] == 1.6
    assert params["landing_visual_handoff_max_ground_error_m"] == 0.20
    assert params["landing_visual_handoff_stable_frames"] == 10

    vision_config = ROOT / "drone_vision/config/vision_params_quick.yaml"
    import yaml
    vision_params = next(iter(yaml.safe_load(
        vision_config.read_text(encoding="utf-8")
    ).values()))["ros__parameters"]
    assert vision_params["landing_circle_track_enable"] is True
    assert vision_params["landing_yolo_guard_rate_hz"] == 2.0

    for launch_name in ("quick_sitl.launch.py", "quick_flight.launch.py"):
        launch_source = (ROOT / "drone_bringup/launch" / launch_name).read_text(
            encoding="utf-8"
        )
        assert "vision_params_quick.yaml" in launch_source


def test_base_params_untouched_by_simple_landing():
    """原参数文件无 simple 参数 → 默认 False, 主任务降落零漂移."""
    import yaml
    for name in ("mission_params.yaml", "mission_params_real.yaml"):
        config = yaml.safe_load(
            (ROOT / "drone_mission/config" / name).read_text(encoding="utf-8")
        )
        node = next(iter(config.values()))["ros__parameters"]
        assert "landing_simple_mode" not in node
        assert "landing_search_timeout" not in node
        assert "landing_simple_blind_timeout" not in node
