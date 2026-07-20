/*
 * 投放状态机 — 多位置步进投放管理.
 *
 * 投放逻辑:
 *   positions = [1150, 1700, 2150]  (μs)
 *   初始:    舵机 = 1150μs (闭合)
 *   投放 #1: 舵机 → 1700μs → 等待 drop_delay_ms
 *   投放 #2: 舵机 → 2150μs → 等待 drop_delay_ms
 *
 * 只需要一步到位, 不涉及 "open/close 循环".
 * 每个位置严格递增, 舵机不回退.
 */

#pragma once

#include "drone_servo2/servo_interface.hpp"

#include <functional>
#include <string>
#include <vector>

namespace drone_servo2
{

enum class DropState
{
  IDLE,
  MOVING,
  DONE,
  FAILED,
};

constexpr const char * drop_state_str(DropState s)
{
  switch (s) {
    case DropState::IDLE:   return "idle";
    case DropState::MOVING: return "moving";
    case DropState::DONE:   return "done";
    case DropState::FAILED: return "failed";
    default:                return "unknown";
  }
}

class DropManager
{
public:
  using DropDoneCb = std::function<void (bool success, int cylinder_id)>;

  /// @param servo       舵机实例
  /// @param positions   PWM 位置序列 (μs), 长度 = max_drops + 1
  /// @param delay_ms    投放后保持延时 (ms)
  DropManager(IServo * servo, std::vector<double> positions, double delay_ms);

  /// 执行一次投放: 舵机移动到 positions[drop_count + 1]
  bool execute_drop(int cylinder_id);

  /// 是否还可以投放
  bool can_drop() const;

  /// 重置状态, 舵机回到 positions[0]
  void reset();

  /// 注册投放完成回调
  void set_on_drop_done(DropDoneCb cb) {on_drop_done_ = std::move(cb);}

  // ---- 只读属性 ----
  DropState         state()          const {return state_;}
  int               drop_count()     const {return drop_count_;}
  int               drops_remaining() const {return max_drops_ - drop_count_;}
  int               max_drops()      const {return max_drops_;}
  bool              last_result()    const {return last_result_;}
  double            current_pwm()    const {return positions_[drop_count_];}
  const std::vector<double> & positions() const {return positions_;}

private:
  void notify_done(bool success, int cylinder_id);

  IServo * servo_;
  std::vector<double> positions_;
  int max_drops_;
  double delay_ms_;

  DropState state_{DropState::IDLE};
  int drop_count_{0};
  bool last_result_{false};

  DropDoneCb on_drop_done_;
};

}  // namespace drone_servo2
