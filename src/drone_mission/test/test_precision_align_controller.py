import math

from drone_mission.precision_align_controller import PrecisionAlignController


class _ClockNow:
    def __init__(self, seconds):
        self.nanoseconds = int(seconds * 1_000_000_000)


class _Clock:
    def __init__(self):
        self.seconds = 1.0

    def now(self):
        return _ClockNow(self.seconds)

    def advance(self, seconds):
        self.seconds += seconds


class _Logger:
    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass


class _Velocity:
    def __init__(self, vn=0.0, ve=0.0, vd=0.0):
        self.vn = vn
        self.ve = ve
        self.vd = vd


class _Telemetry:
    def __init__(self, velocity=None):
        self.velocity = velocity or _Velocity()


class _Bridge:
    def get_telemetry(self):
        return _Telemetry(_Velocity(0.0, 0.0, 0.0))


class _Flight:
    def __init__(self):
        self.body_commands = []
        self.stop_count = 0

    def set_velocity_body(self, forward, right, down=0.0):
        self.body_commands.append((forward, right, down))

    def stop_velocity(self):
        self.stop_count += 1


class _Selector:
    _selected_positions = [(42.3, 5.5)]


class _Fsm:
    def __init__(self, params=None):
        self.params = {
            "camera_cx": 640.0,
            "camera_cy": 480.0,
            "precision_align_image_width": 1280.0,
            "precision_align_image_height": 960.0,
            "precision_align_alpha": 1.0,
            # 增益名已迁移到 pid_tuning.yaml 契约, 数值保持原测试取值:
            # precision_align_gain→servo_norm_kp, max_speed→servo_max_speed,
            # min_correction_speed→servo_min_speed.
            "servo_norm_kp": 0.65,
            "servo_max_speed": 1.0,
            "precision_align_fine_pixel_error": 80.0,
            "precision_align_fine_max_speed": 0.022,
            "servo_min_speed": 0.006,
            "precision_align_fine_step_s": 0.08,
            "precision_align_raw_pixel_tolerance": 6.0,
            "precision_align_smooth_pixel_tolerance": 5.0,
            "precision_align_threshold_norm_x": 0.006,
            "precision_align_threshold_norm_y": 0.006,
            "precision_align_stable_frames": 8,
            "precision_align_final_brake_s": 0.8,
            "precision_align_release_max_horizontal_speed": 0.02,
            "precision_align_target_world_gate": 2.0,
            "precision_align_image_x_to_right_sign": 1.0,
            "precision_align_image_y_to_forward_sign": -1.0,
        }
        if params:
            self.params.update(params)
        self.fc = _Flight()
        self.cylinder_selector = _Selector()
        self.clock = _Clock()
        self.bridge = _Bridge()
        self._frozen_measurement_stamp = None

    def get_param(self, key, default=None):
        return self.params.get(key, default)

    def get_clock(self):
        return self.clock

    def get_logger(self):
        return _Logger()

    def vision_measurement_stamp(self, source):
        """测量时刻 (秒). 默认跟随测试时钟, 即每 tick 到一份新测量 ——
        与这些用例的既有假设一致. 用 freeze_measurement() 模拟真机行为:
        同一份 3-5Hz 测量在多个 20Hz tick 上被复用."""
        if self._frozen_measurement_stamp is not None:
            return self._frozen_measurement_stamp
        return self.clock.seconds

    def freeze_measurement(self, seconds):
        """把测量时刻钉死在给定值: 之后的 tick 都算"无新测量"."""
        self._frozen_measurement_stamp = seconds


def test_default_image_y_sign_matches_cuadc_for_target_above_center():
    fsm = _Fsm()
    controller = PrecisionAlignController(fsm)
    controller.reset()

    controller.step(
        markers=[],
        cylinders=[{
            "confidence": 0.9,
            "center_u": 640.0,
            "center_v": 345.0,
            "world_position_valid": True,
            "world_x": 41.7,
            "world_y": 5.5,
        }],
        drop_height=2.0,
    )

    forward, right, down = fsm.fc.body_commands[-1]
    assert forward > 0.0
    assert abs(right) < 1e-9
    assert down == 0.0


def test_pixel_only_precision_servo_does_not_claim_invalid_world_position():
    fsm = _Fsm()
    controller = PrecisionAlignController(fsm)
    controller.reset()

    controller.step(
        markers=[],
        cylinders=[{
            "confidence": 0.9,
            "center_u": 680.0,
            "center_v": 480.0,
            "world_position_valid": False,
            "world_x": math.nan,
            "world_y": math.nan,
        }],
        drop_height=2.0,
    )

    assert fsm.fc.body_commands
    assert controller.last_status.target_world_xy is None


def test_image_y_sign_can_be_overridden_for_other_camera_mounts():
    fsm = _Fsm({"precision_align_image_y_to_forward_sign": 1.0})
    controller = PrecisionAlignController(fsm)
    controller.reset()

    controller.step(
        markers=[],
        cylinders=[{
            "confidence": 0.9,
            "center_u": 640.0,
            "center_v": 345.0,
            "world_position_valid": True,
            "world_x": 41.7,
            "world_y": 5.5,
        }],
        drop_height=2.0,
    )

    forward, _right, _down = fsm.fc.body_commands[-1]
    assert forward < 0.0


def test_locked_target_rejects_world_jump_even_when_new_marker_is_near_center():
    fsm = _Fsm({
        "precision_align_target_world_gate": 1.5,
        "precision_align_max_jump_diag_ratio": 0.12,
        "precision_align_reject_center_ratio": 0.20,
        "precision_align_reacquire_after_lost": 10,
    })
    controller = PrecisionAlignController(fsm)
    controller.reset(target_hint_xy=(43.0, 2.1))

    controller.step(
        markers=[],
        cylinders=[{
            "confidence": 0.9,
            "center_u": 808.0,
            "center_v": 385.0,
            "world_position_valid": True,
            "world_x": 43.03,
            "world_y": 2.15,
        }],
        drop_height=2.0,
    )

    fsm.clock.advance(1.0)

    controller.step(
        markers=[],
        cylinders=[{
            "confidence": 0.9,
            "center_u": 1180.0,
            "center_v": 850.0,
            "world_position_valid": True,
            "world_x": 44.4,
            "world_y": 4.1,
        }],
        drop_height=2.0,
    )

    fsm.clock.advance(0.2)

    result = controller.step(
        markers=[],
        cylinders=[{
            "confidence": 0.9,
            "center_u": 1180.0,
            "center_v": 850.0,
            "world_position_valid": True,
            "world_x": 44.4,
            "world_y": 4.1,
        }],
        drop_height=2.0,
    )

    assert result.value == "continue"
    assert controller.last_status.reason == "all_candidates_rejected"
    assert controller.last_status.lost_frames == 1
    assert controller.last_status.target_world_xy is None


def test_cylinder_precision_does_not_fallback_to_marker():
    fsm = _Fsm({
        "precision_align_target_source": "cylinder",
        "precision_align_target_world_gate": 1.5,
        "precision_align_reacquire_after_lost": 10,
    })
    controller = PrecisionAlignController(fsm)
    controller.reset(target_hint_xy=(43.50, 2.90))

    result = controller.step(
        markers=[{
            "confidence": 0.9,
            "center_u": 808.0,
            "center_v": 385.0,
            "world_position_valid": True,
            "world_x": 43.03,
            "world_y": 2.15,
        }],
        cylinders=[],
        drop_height=2.0,
    )

    assert result.value == "continue"
    assert controller.last_status.source == "cylinder"
    assert controller.last_status.reason == "no_candidates"
    assert controller.last_status.lost_frames == 1
    assert not fsm.fc.body_commands


def test_release_requires_raw_pixel_error_not_just_smoothed_error():
    fsm = _Fsm({
        "precision_align_alpha": 0.01,
        "precision_align_stable_frames": 2,
        "precision_align_final_brake_s": 0.0,
    })
    controller = PrecisionAlignController(fsm)
    controller.reset(target_hint_xy=(42.3, 5.5))

    for _ in range(10):
        controller.step(
            markers=[],
            cylinders=[{
                "confidence": 0.9,
                "center_u": 640.0,
                "center_v": 480.0,
                "world_position_valid": True,
                "world_x": 42.3,
                "world_y": 5.5,
            }],
            drop_height=2.0,
        )
        fsm.clock.advance(0.2)

    result = controller.step(
        markers=[],
        cylinders=[{
            "confidence": 0.9,
            "center_u": 698.0,
            "center_v": 480.0,
            "world_position_valid": True,
            "world_x": 42.3,
            "world_y": 5.5,
        }],
        drop_height=2.0,
    )

    assert result.value == "continue"
    assert not controller.last_status.release_ready
    assert controller.last_status.pixel_error > 6.0
    assert controller.last_status.stable_count == 0


def test_release_happens_only_after_stable_frames_and_brake_recheck():
    fsm = _Fsm({
        "precision_align_alpha": 1.0,
        "precision_align_stable_frames": 2,
        "precision_align_final_brake_s": 0.5,
    })
    controller = PrecisionAlignController(fsm)
    controller.reset(target_hint_xy=(42.3, 5.5))

    target = [{
        "confidence": 0.9,
        "center_u": 642.0,
        "center_v": 482.0,
        "world_position_valid": True,
        "world_x": 42.3,
        "world_y": 5.5,
    }]

    assert controller.step([], target, 2.0).value == "continue"
    fsm.clock.advance(0.2)
    assert controller.step([], target, 2.0).value == "continue"
    assert controller.last_status.reason == "final_brake"
    assert not controller.last_status.release_ready

    fsm.clock.advance(0.6)
    result = controller.step([], target, 2.0)

    assert result.value == "converged"
    assert controller.last_status.release_ready
    assert controller.last_status.reason == "ready"


def test_fine_alignment_uses_low_speed_cap():
    fsm = _Fsm({
        "precision_align_alpha": 1.0,
        "precision_align_fine_pixel_error": 80.0,
        "precision_align_fine_max_speed": 0.02,
        "precision_align_raw_pixel_tolerance": 6.0,
        "precision_align_smooth_pixel_tolerance": 5.0,
    })
    controller = PrecisionAlignController(fsm)
    controller.reset(target_hint_xy=(42.3, 5.5))

    controller.step(
        markers=[],
        cylinders=[{
            "confidence": 0.9,
            "center_u": 690.0,
            "center_v": 500.0,
            "world_position_valid": True,
            "world_x": 42.3,
            "world_y": 5.5,
        }],
        drop_height=2.0,
    )

    forward, right, _down = fsm.fc.body_commands[-1]
    assert (forward ** 2 + right ** 2) ** 0.5 <= 0.0200001


def test_ground_error_is_height_invariant_for_same_physical_offset():
    controller = PrecisionAlignController(_Fsm({
        "camera_fx": 400.0,
        "camera_fy": 400.0,
    }))

    forward_high, right_high = controller._ground_error_m(40.0, 0.0, 2.0)
    forward_low, right_low = controller._ground_error_m(80.0, 0.0, 1.0)

    assert forward_high == forward_low == 0.0
    assert right_high == right_low == 0.20


def test_bucket_servo_uses_camera_to_rim_distance_from_measured_heights():
    fsm = _Fsm({
        "quick_drop_profile": "bucket",
        "precision_align_continuous_enable": True,
        "bucket_home_body_height": 0.40,
        "bucket_camera_down": 0.365,
        "bucket_rim_height": 0.40,
        "bucket_front_forward": 0.045,
        "bucket_front_right": 0.0,
        "camera_fx": 400.0,
        "camera_fy": 400.0,
    })
    controller = PrecisionAlignController(fsm)
    controller.reset()

    controller.step(
        markers=[],
        cylinders=[{
            "confidence": 0.9,
            "center_u": 680.0,
            "center_v": 480.0,
            "world_position_valid": True,
            "world_x": 42.3,
            "world_y": 5.5,
        }],
        drop_height=1.6,
        altitude_m=1.6,
        descent_speed=0.10,
        allow_release=False,
    )

    assert math.isclose(controller.last_status.ground_right_error_m,
                        40.0 * 1.235 / 400.0, abs_tol=1e-9)


def test_bucket_low_altitude_servo_does_not_switch_to_blue_marker():
    fsm = _Fsm({
        "quick_drop_profile": "bucket",
        "precision_align_continuous_enable": True,
    })
    controller = PrecisionAlignController(fsm)
    controller.reset()
    controller.step(markers=[{
        "confidence": 0.99,
        "center_u": 800.0,
        "center_v": 480.0,
    }], cylinders=[], drop_height=1.6, altitude_m=1.6,
        descent_speed=0.0, allow_release=False)
    assert controller.last_status.source == "bucket"
    assert controller.last_status.reason == "no_candidates"
    assert fsm.fc.body_commands == []
    assert fsm.fc.stop_count == 1


def test_bucket_continuous_servo_speed_cap_decreases_with_height():
    for altitude, expected_cap in ((4.5, 0.25), (3.0, 0.14),
                                   (1.9, 0.065), (1.65, 0.065)):
        fsm = _Fsm({
            "quick_drop_profile": "bucket",
            "precision_align_continuous_enable": True,
            "servo_kp": 0.40,
            "servo_max_speed": 0.25,
            "bucket_servo_mid_max_speed": 0.14,
            "precision_align_fine_max_speed": 0.065,
            "bucket_home_body_height": 0.40,
            "bucket_camera_down": 0.365,
            "bucket_rim_height": 0.40,
            "bucket_front_forward": 0.0,
            "bucket_front_right": 0.0,
            "camera_fx": 400.0,
            "camera_fy": 400.0,
        })
        controller = PrecisionAlignController(fsm)
        controller.reset()
        controller.step(markers=[], cylinders=[{
            "confidence": 0.9,
            "center_u": 1000.0,
            "center_v": 480.0,
            "world_position_valid": False,
        }], drop_height=1.6, altitude_m=altitude,
            descent_speed=0.12, allow_release=False)
        forward, right, down = fsm.fc.body_commands[-1]
        assert math.isclose(math.hypot(forward, right), expected_cap, abs_tol=1e-9)
        assert down == 0.12


def test_blue_continuous_servo_keeps_original_speed_cap_at_low_height():
    fsm = _Fsm({
        "quick_drop_profile": "blue",
        "precision_align_continuous_enable": True,
        "servo_kp": 0.40,
        "servo_max_speed": 0.25,
        "bucket_servo_mid_max_speed": 0.14,
        "precision_align_fine_max_speed": 0.065,
        "camera_fx": 400.0,
        "camera_fy": 400.0,
    })
    controller = PrecisionAlignController(fsm)
    controller.reset()
    controller.step(markers=[], cylinders=[{
        "confidence": 0.9,
        "center_u": 1000.0,
        "center_v": 480.0,
        "world_position_valid": False,
    }], drop_height=1.6, altitude_m=1.65,
        descent_speed=0.12, allow_release=False)
    forward, right, _down = fsm.fc.body_commands[-1]
    assert math.isclose(math.hypot(forward, right), 0.25, abs_tol=1e-9)


def test_continuous_step_keeps_horizontal_servo_and_descent_in_one_command():
    fsm = _Fsm({
        "precision_align_continuous_enable": True,
        "servo_kp": 0.40,
        "camera_fx": 400.0,
        "camera_fy": 400.0,
        "servo_max_speed": 1.0,
        "precision_align_raw_pixel_tolerance": 6.0,
        "precision_align_smooth_pixel_tolerance": 5.0,
    })
    controller = PrecisionAlignController(fsm)
    controller.reset()

    controller.step(
        markers=[],
        cylinders=[{
            "confidence": 0.9,
            "center_u": 680.0,
            "center_v": 480.0,
            "world_position_valid": True,
            "world_x": 42.3,
            "world_y": 5.5,
        }],
        drop_height=2.4,
        altitude_m=2.4,
        descent_speed=0.28,
        allow_release=False,
    )

    forward, right, down = fsm.fc.body_commands[-1]
    assert forward == 0.0
    assert right == 0.096
    assert down == 0.28
    assert fsm.fc.stop_count == 0


def test_continuous_descent_keeps_correcting_inside_release_pixel_window():
    """下降阶段不能因投放容差而暂停横向伺服。"""
    fsm = _Fsm({
        "precision_align_continuous_enable": True,
        "servo_kp": 0.40,
        "camera_fx": 400.0,
        "camera_fy": 400.0,
        "servo_max_speed": 1.0,
        "precision_align_raw_pixel_tolerance": 80.0,
        "precision_align_smooth_pixel_tolerance": 80.0,
        "precision_align_threshold_norm_x": 0.10,
        "precision_align_threshold_norm_y": 0.10,
    })
    controller = PrecisionAlignController(fsm)
    controller.reset()

    controller.step(
        markers=[],
        cylinders=[{
            "confidence": 0.9,
            "center_u": 680.0,  # 40 px: 在投放容差内，但仍有 0.24 m 地面偏差
            "center_v": 480.0,
            "world_position_valid": True,
            "world_x": 42.3,
            "world_y": 5.5,
        }],
        drop_height=2.4,
        altitude_m=2.4,
        descent_speed=0.28,
        allow_release=False,
    )

    forward, right, down = fsm.fc.body_commands[-1]
    assert forward == 0.0
    assert right == 0.096
    assert down == 0.28


def _window_target(center_u=640.0, center_v=480.0):
    return [{
        "confidence": 0.9,
        "center_u": center_u,
        "center_v": center_v,
        "world_position_valid": True,
        "world_x": 42.3,
        "world_y": 5.5,
    }]


def test_two_stage_window_switches_by_drop_height():
    """两级窗口: 2.8m 精校段 35px 通过, 降到 1.5m 投放段同 33px 收紧被拒."""
    fsm = _Fsm({
        "precision_align_alpha": 1.0,
        "precision_align_raw_pixel_tolerance": 35.0,      # 精校段窗口
        "precision_align_final_raw_pixel_tolerance": 30.0,  # 投放段窗口
        "precision_align_tight_window_height": 2.0,       # 切换高度
        "precision_align_smooth_pixel_tolerance": 100.0,  # 只测 raw 窗口
        "precision_align_threshold_norm_x": 0.1,
        "precision_align_threshold_norm_y": 0.1,
    })
    controller = PrecisionAlignController(fsm)
    controller.reset(target_hint_xy=(42.3, 5.5))

    # 精校段 (2.8m): 33px ≤ 35 → aligned, 进入稳定计数
    controller.step([], _window_target(center_u=640.0 + 33.0), drop_height=2.8)
    assert controller.last_status.stable_count == 1

    # 投放段 (1.5m): 同 33px > 30 → 不 aligned, 稳定清零并走修正分支
    controller.step([], _window_target(center_u=640.0 + 33.0), drop_height=1.5)
    assert controller.last_status.stable_count == 0
    assert len(fsm.fc.body_commands) > 0

    # 投放段: 28px ≤ 30 → 重新 aligned (先跨过修正脉冲 + hover 两段等待)
    fsm.clock.advance(0.2)
    controller.step([], _window_target(center_u=640.0 + 28.0), drop_height=1.5)
    fsm.clock.advance(0.1)
    controller.step([], _window_target(center_u=640.0 + 28.0), drop_height=1.5)
    assert controller.last_status.stable_count == 1


def test_unconfigured_tight_window_height_keeps_single_window():
    """SITL 不配 tight_window_height (默认 0.0) → 永不切换, 行为不变."""
    fsm = _Fsm({
        "precision_align_alpha": 1.0,
        "precision_align_raw_pixel_tolerance": 35.0,
        "precision_align_final_raw_pixel_tolerance": 30.0,
        # 未配置 precision_align_tight_window_height
        "precision_align_smooth_pixel_tolerance": 100.0,
        "precision_align_threshold_norm_x": 0.1,
        "precision_align_threshold_norm_y": 0.1,
    })
    controller = PrecisionAlignController(fsm)
    controller.reset(target_hint_xy=(42.3, 5.5))

    # 即便 1.5m 低于任何切换高度, 仍用精校段窗口 35 → aligned
    controller.step([], _window_target(center_u=640.0 + 33.0), drop_height=1.5)
    assert controller.last_status.stable_count == 1


def _marker(center_u, center_v, world_x, world_y, confidence=0.9):
    return {
        "confidence": confidence,
        "center_u": center_u,
        "center_v": center_v,
        "world_position_valid": True,
        "world_x": world_x,
        "world_y": world_y,
    }


def _gate_candidates():
    """居中但远离记忆点 / 偏右但贴近记忆点 — 同一批候选, 只看门控是否生效."""
    return (
        _marker(640.0, 480.0, 50.0, 20.0),   # center_ratio=0, 距 hint 17m
        _marker(740.0, 480.0, 42.2, 5.1),    # center_ratio=0.0625, 距 hint 0.22m
    )


def test_world_gate_off_by_default_lets_off_hint_candidate_win():
    """门控默认关闭: 远离记忆点但更居中的候选获胜 (一切以视觉像素为准)."""
    fsm = _Fsm({"precision_align_target_source": "marker"})
    controller = PrecisionAlignController(fsm)
    controller.reset(target_hint_xy=(42.0, 5.0))

    centered_off_hint, off_center_near_hint = _gate_candidates()
    selected, _source, reason = controller._choose_target(
        [centered_off_hint, off_center_near_hint], [],
    )

    assert reason == "tracking"
    assert selected is centered_off_hint


def test_world_gate_on_rejects_the_off_hint_candidate():
    """门控开启: 同一批候选, 远处目标被拒, 改选贴近记忆点的那一个."""
    fsm = _Fsm({
        "precision_align_target_source": "marker",
        "precision_align_world_gate_enable": True,
        "precision_align_target_world_gate": 1.5,
    })
    controller = PrecisionAlignController(fsm)
    controller.reset(target_hint_xy=(42.0, 5.0))

    centered_off_hint, off_center_near_hint = _gate_candidates()
    selected, _source, reason = controller._choose_target(
        [centered_off_hint, off_center_near_hint], [],
    )

    assert reason == "tracking"
    assert selected is off_center_near_hint


def test_world_gate_on_rejects_every_candidate_when_all_are_off_hint():
    """门控开启且全部候选都超距 → 无候选可用 (这正是 quick 当年卡死的机制)."""
    fsm = _Fsm({
        "precision_align_target_source": "marker",
        "precision_align_world_gate_enable": True,
        "precision_align_target_world_gate": 1.5,
    })
    controller = PrecisionAlignController(fsm)
    controller.reset(target_hint_xy=(42.0, 5.0))

    centered_off_hint, _ = _gate_candidates()
    selected, _source, reason = controller._choose_target(
        [centered_off_hint], [],
    )

    assert selected is None
    assert reason == "all_candidates_rejected"


def test_bucket_ignores_world_gate_and_tracks_current_image():
    fsm = _Fsm({
        "quick_drop_profile": "bucket",
        "precision_align_world_gate_enable": True,
    })
    controller = PrecisionAlignController(fsm)
    controller.reset(target_hint_xy=(42.0, 5.0), target_hint_uv=(640.0, 480.0))
    target = {
        "confidence": 0.9, "center_u": 640.0, "center_v": 480.0,
        "world_position_valid": False, "world_x": math.nan, "world_y": math.nan,
    }
    selected, source, reason = controller._choose_target([], [target])
    assert (selected, source, reason) == (target, "cylinder", "tracking")
