from drone_mission.states.state_recon_patrol import ReconPatrolState


class _Planner:
    def plan_coverage_ned(self, **_kwargs):
        return [
            {"north": 55.0, "east": -4.0, "down": -2.5},
            {"north": 60.0, "east": -4.0, "down": -2.5},
        ]


class _FC:
    def reached(self, *_args):
        return True

    def goto_ned(self, *_args, **_kwargs):
        pass

    def is_waypoint_timed_out(self):
        return False

    origin_down = 0.0

    def altitude_to_ned(self, altitude_m):
        return self.origin_down - altitude_m


class _Logger:
    def info(self, _message):
        pass

    def error(self, _message):
        pass


class _TaskLogger:
    def log_state(self, *_args):
        pass


class _FSM:
    def __init__(self):
        self.path_planner = _Planner()
        self.fc = _FC()
        self.watchdog = None
        self.logger = _TaskLogger()
        self.vision_modes = []

    def get_param(self, _name, default):
        return default

    def get_logger(self):
        return _Logger()

    def get_coverage(self):
        return None

    def set_vision_mode(self, mode):
        self.vision_modes.append(mode)


def test_recon_starts_coverage_only_after_reaching_the_first_strip_entry():
    fsm = _FSM()
    state = ReconPatrolState(fsm)
    state.enter()

    assert fsm.vision_modes == ["coverage_reset"]

    for _ in range(8):
        assert state.execute() is None

    assert fsm.vision_modes == ["coverage_reset", "coverage"]
