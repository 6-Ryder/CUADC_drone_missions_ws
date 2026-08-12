"""
圆筒选择策略 — 从检测到的圆筒中选择投放目标.

策略:
  1. 按半径升序排序
  2. 第一轮选择半径最小者
  3. 第二轮选择半径次小者
  4. 不得重复选择同一圆筒 (基于 ID + 位置去重)
  5. 若半径估计置信度不足, 建议降高重新确认
"""

import math
from typing import List, Optional, Set


class CylinderSelector:
    """
    圆筒选择器.

    从 CylinderArray 中按策略选择投放目标.

    使用方式:
      selector = CylinderSelector()
      target = selector.select(cylinders)  # 第一轮
      target = selector.select(cylinders)  # 第二轮 (自动排除上次选择)
    """

    def __init__(self, min_radius_first: bool = True):
        """
        初始化目标选择器.

        Parameters
        ----------
        min_radius_first : bool
            True: 先选半径最小的 (默认)
            False: 先选半径最大的

        """
        self._min_radius_first = min_radius_first
        self._selected_ids: Set[int] = set()
        self._selected_positions: List[tuple] = []  # [(x, y), ...]
        self._selected_id_list: List[int] = []      # ordered IDs matching _selected_positions
        self._selection_count: int = 0
        self._position_tolerance: float = 0.5  # 位置去重容差 (m)

    def select(self, cylinders: list) -> Optional[int]:
        """
        选择下一个投放目标.

        Parameters
        ----------
        cylinders : list of CylinderDetection (或 dict with id/world_x/world_y/radius/confidence)
            当前可见的圆筒目标.

        Returns
        -------
        int or None: 选中的圆筒 ID, None 表示无可选

        """
        # 过滤掉已选择的
        available = []
        for c in cylinders:
            cid = c.id if hasattr(c, 'id') else c.get('id', -1)
            if cid in self._selected_ids:
                continue
            # 位置去重检查
            if isinstance(c, dict):
                world_valid = bool(c.get('world_position_valid', False))
                cx = c.get('world_x', math.nan)
                cy = c.get('world_y', math.nan)
            else:
                world_valid = bool(getattr(c, 'world_position_valid', False))
                cx = getattr(c, 'world_x', math.nan)
                cy = getattr(c, 'world_y', math.nan)
            if not world_valid or not (math.isfinite(cx) and math.isfinite(cy)):
                continue
            is_duplicate = any(
                math.hypot(cx - px, cy - py) < self._position_tolerance
                for px, py in self._selected_positions
            )
            if is_duplicate:
                continue

            radius = c.radius if hasattr(c, 'radius') else c.get('radius', 0)
            conf = c.confidence if hasattr(c, 'confidence') else c.get('confidence', 0)
            available.append((cid, cx, cy, radius, conf))

        if not available:
            return None

        # 按半径排序
        if self._min_radius_first:
            available.sort(key=lambda x: x[3])  # 升序
        else:
            available.sort(key=lambda x: -x[3])  # 降序

        # 选择第一个 (半径最小或最大)
        chosen = available[0]
        cid, cx, cy, radius, conf = chosen

        # 记录
        self._selected_ids.add(cid)
        self._selected_positions.append((cx, cy))
        self._selected_id_list.append(cid)
        self._selection_count += 1

        return cid

    def release_last_selection(self) -> None:
        """
        撤销最近一次选择, 用于逼近/投放失败后切换到下一个目标.

        保留选择历史 (_selected_positions / _selected_ids) 不变,
        仅递减计数, 确保下次 select() 排除已失败的目标.
        """
        self._selection_count = max(0, self._selection_count - 1)

    def select_with_confidence_check(self, cylinders: list,
                                     min_confidence: float = 0.5) -> tuple:
        """
        带置信度检查的选择.

        Returns
        -------
        (target_id, needs_reconfirm)
            target_id: 选中的圆筒 ID, -1 表示暂无可选
            needs_reconfirm: True 表示需要降高重新检测

        """
        if not cylinders:
            return (-1, False)

        # 检查置信度
        low_conf = any(
            (c.confidence if hasattr(c, 'confidence') else c.get('confidence', 0)) < min_confidence
            for c in cylinders
        )
        if low_conf:
            return (-1, True)  # 需要重新确认

        target = self.select(cylinders)
        if target is None:
            return (-1, False)

        return (target, False)

    def reset(self):
        """重置选择器 (新任务开始时调用)."""
        self._selected_ids.clear()
        self._selected_positions.clear()
        self._selected_id_list.clear()
        self._selection_count = 0

    @property
    def selection_count(self) -> int:
        return self._selection_count

    @property
    def selected_list(self) -> list:
        return list(zip(self._selected_id_list, self._selected_positions))
