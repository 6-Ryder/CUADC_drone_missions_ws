# 圆筒检测结果
int32 id             # 跟踪ID
float64 center_x     # 圆心 NED east
float64 center_y     # 圆心 NED north
float64 center_u     # 圆心像素 u
float64 center_v     # 圆心像素 v
float64 radius       # 估计真实半径 (m)
float64 confidence   # 0.0~1.0
# YOLO 边界框 (用于可视化调试)
int32 bbox_x1        # bbox 左上 x (像素)
int32 bbox_y1        # bbox 左上 y (像素)
int32 bbox_x2        # bbox 右下 x (像素)
int32 bbox_y2        # bbox 右下 y (像素)
