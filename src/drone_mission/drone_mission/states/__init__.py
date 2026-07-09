"""
任务状态处理器 — 每个文件实现一个独立的状态逻辑.

状态列表:
  INIT → ARM → TAKEOFF → TRANSIT_TO_DROP_ZONE → CYLINDER_SEARCH
  → SELECT_TARGET → CYLINDER_APPROACH → DESCEND_TO_DROP
  → EXECUTE_DROP → ASCEND → (repeat drop or) TRANSIT_TO_RECON_ZONE
  → RECON_PATROL (全覆盖弓字形巡视) → RETURN_TO_HOME → PRECISION_LAND
  → COMPLETE

注意: 侦察区不再包含 RECON_CLASSIFY, 仅执行全覆盖路径飞行.
"""

from drone_mission.states.state_init import InitState
from drone_mission.states.state_takeoff import TakeoffState
from drone_mission.states.state_transit import TransitState
from drone_mission.states.state_cylinder_search import CylinderSearchState
from drone_mission.states.state_cylinder_approach import CylinderApproachState
from drone_mission.states.state_drop import DropState
from drone_mission.states.state_recon_patrol import ReconPatrolState
from drone_mission.states.state_return import ReturnState
from drone_mission.states.state_precision_land import PrecisionLandState

__all__ = [
    "InitState", "TakeoffState", "TransitState",
    "CylinderSearchState", "CylinderApproachState", "DropState",
    "ReconPatrolState",
    "ReturnState", "PrecisionLandState",
]
