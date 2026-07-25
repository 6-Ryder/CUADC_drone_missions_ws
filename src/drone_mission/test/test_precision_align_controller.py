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
            "precision_align_gain": 0.65,
            "precision_align_max_speed": 1.0,
            "precision_align_fine_pixel_error": 80.0,
            "precision_align_fine_max_speed": 0.022,
            "precision_align_min_correction_speed": 0.006,
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

    def get_param(self, key, default=None):
        return self.params.get(key, default)

    def get_clock(self):
        return self.clock

    def get_logger(self):
        return _Logger()


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
            "center_x": 41.7,
            "center_y": 5.5,
        }],
        drop_height=2.0,
    )

    forward, right, down = fsm.fc.body_commands[-1]
    assert forward > 0.0
    assert abs(right) < 1e-9
    assert down == 0.0


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
            "center_x": 41.7,
            "center_y": 5.5,
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
            "center_x": 43.03,
            "center_y": 2.15,
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
            "center_x": 44.4,
            "center_y": 4.1,
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
            "center_x": 44.4,
            "center_y": 4.1,
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
            "center_x": 43.03,
            "center_y": 2.15,
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
                "center_x": 42.3,
                "center_y": 5.5,
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
            "center_x": 42.3,
            "center_y": 5.5,
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
        "center_x": 42.3,
        "center_y": 5.5,
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
            "center_x": 42.3,
            "center_y": 5.5,
        }],
        drop_height=2.0,
    )

    forward, right, _down = fsm.fc.body_commands[-1]
    assert (forward ** 2 + right ** 2) ** 0.5 <= 0.0200001
