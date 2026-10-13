# H降落目标定位
#
# header.stamp   = 该检测所依据的相机帧时刻 (采集时刻), 不是本消息的构造/发布
#                  时刻. 消费方的一切龄期判断与积分/微分步长都以它为准 —— 与
#                  MAVLink LANDING_TARGET.time_usec / PX4
#                  FiducialMarkerReport.timestamp_sample 的语义一致.
#                  注意: 视觉节点对"本帧没看到目标"也发布消息 (confidence=0),
#                  因此 stamp 前进只说明"相机还在出帧", 不说明"有新测量".
#                  判"还有没有可用测量"要用 confidence>0 另记时刻.
# header.frame_id = world (与 CylinderArray/HazardArray 同约定); 像素域字段
#                  (center_u/v) 是图像域, 不属于该帧.
std_msgs/Header header
float64 center_u           # 圆心像素 u
float64 center_v           # 圆心像素 v
float64 world_x            # World frame x (投影失败时保持 0.0, 与真实原点不可区分)
float64 world_y            # World frame y
float64 yaw_correction     # 偏航修正角 (rad)
float64 confidence         # 0.0~1.0; 恰为 0.0 表示"本帧无检测"
