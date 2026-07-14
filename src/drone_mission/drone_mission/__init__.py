"""drone_mission — 任务编排层."""

from drone_mission.cylinder_selector import CylinderSelector
from drone_mission.path_planner import PathPlanner

__all__ = ["MissionFSM", "MissionState", "CylinderSelector", "PathPlanner"]


def __getattr__(name):
    if name in ("MissionFSM", "MissionState"):
        from drone_mission.mission_state_machine import MissionFSM, MissionState

        return {"MissionFSM": MissionFSM, "MissionState": MissionState}[name]
    raise AttributeError(name)
