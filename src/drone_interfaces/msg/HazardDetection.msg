# 危险品检测结果
int32 id                 # 关联的圆筒ID
uint8 hazard_type        # 0=EMPTY, 1=FLAMMABLE, 2=HEALTH_HAZARD, 3=DANGER
float64 center_x         # 圆心 NED east
float64 center_y         # 圆心 NED north
float64 world_x          # World frame x
float64 world_y          # World frame y
float64 confidence       # 0.0~1.0
