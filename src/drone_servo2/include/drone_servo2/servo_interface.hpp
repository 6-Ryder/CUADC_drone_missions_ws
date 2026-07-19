/*
 * 舵机抽象接口 — 抹平 SITL / PX4-VehicleCommand 差异.
 */

#pragma once

#include <map>
#include <string>

namespace drone_servo2
{

class IServo
{
public:
  virtual ~IServo() = default;

  /// 设置 PWM 输出脉宽 (μs)
  virtual bool set_pwm(double pwm_us) = 0;

  /// 舵机是否就绪
  virtual bool is_ready() const = 0;

  /// 返回当前状态字典
  virtual std::map<std::string, double> status_dict() const = 0;

  /// 重置: 回到初始位置
  virtual void reset() = 0;
};

}  // namespace drone_servo2
