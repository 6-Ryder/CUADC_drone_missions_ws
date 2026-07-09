"""
RECON_INSPECT 状态 — 对标记目标逐一悬停确认分类.

流程:
  1. 遍历侦察区中检测到的每个可疑圆筒
  2. 飞至该圆筒正上方, 悬停 2-3s
  3. 获取危险品分类结果 (flammable / health_hazard / danger / empty)
  4. 记录分类结果
  5. 所有目标已分类 → RETURN_TO_HOME

如果在巡视中已经完成了分类 (通过覆盖路径中的悬停),
此状态可以快速跳过.
"""

import time
from typing import Optional, List


class ReconInspectState:
    """侦察区逐个确认分类状态."""

    def __init__(self, fsm):
        self._fsm = fsm
        self._start_time: float = 0.0
        self._inspect_targets: List[dict] = []
        self._inspect_idx: int = 0
        self._classified: List[dict] = []
        self._hover_start: float = 0.0
        self._inspect_done: bool = False

    def enter(self):
        self._fsm.logger.log_state("RECON_PATROL", "RECON_CLASSIFY",
                                   "开始危险品分类确认")
        self._start_time = time.time()
        self._inspect_idx = 0
        self._classified = []
        self._inspect_done = False

        # 收集检测到的危险品
        hazards = self._fsm.get_hazards()

        if not hazards:
            self._fsm.get_logger().info("RECON_CLASSIFY: 无危险品需要确认, 直接返航")
            self._inspect_done = True
            return

        # 构建检测目标列表
        # hazards 格式: [{hazard_type, confidence, ...}, ...]
        self._inspect_targets = []
        for h in hazards:
            ht = h.hazard_type if hasattr(h, 'hazard_type') else h.get('hazard_type', 0)
            conf = h.confidence if hasattr(h, 'confidence') else h.get('confidence', 0)
            self._inspect_targets.append({
                "hazard_type": ht,
                "confidence": conf,
                "confirmed": conf > 0.7,  # 高置信度直接确认
            })

        self._fsm.get_logger().info(
            f"RECON_CLASSIFY: {len(self._inspect_targets)} 个目标待确认"
        )

        # 如果所有目标都已高置信度, 直接完成
        if all(t["confirmed"] for t in self._inspect_targets):
            self._fsm.get_logger().info("RECON_CLASSIFY: 所有目标已高置信度, 无需悬停确认")
            self._inspect_done = True
            return

    def execute(self) -> Optional[str]:
        if self._inspect_done:
            return "RETURN_TO_HOME"

        # 对低置信度目标进行二次确认
        # 重新获取最新检测结果
        hazards = self._fsm.get_hazards()

        # 检查是否所有目标都已高置信度
        all_confirmed = True
        for h in hazards:
            conf = h.confidence if hasattr(h, 'confidence') else h.get('confidence', 0)
            if conf < 0.7:
                all_confirmed = False
                break

        if all_confirmed:
            self._fsm.get_logger().info(
                f"RECON_CLASSIFY: 分类确认完成 "
                f"({len(hazards)} 个目标)"
            )
            self._fsm.logger.log("hazards_classified", {
                "count": len(hazards),
                "types": [
                    h.hazard_type if hasattr(h, 'hazard_type') else h.get('hazard_type', 0)
                    for h in hazards
                ],
            })
            return "RETURN_TO_HOME"

        # 超时
        elapsed = time.time() - self._start_time
        if elapsed > 30.0:
            self._fsm.get_logger().warn("RECON_CLASSIFY: 超时, 使用当前分类结果")
            return "RETURN_TO_HOME"

        # 悬停等待更好的检测
        if not self._fsm.fc._landing:
            self._fsm.loiter()

        return None

    def exit(self):
        hazards = self._fsm.get_hazards()
        self._fsm.logger.log_state("RECON_CLASSIFY", "RETURN_TO_HOME",
                                   f"分类完成, {len(hazards)} 个危险品已标记")
