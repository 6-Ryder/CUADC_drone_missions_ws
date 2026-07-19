/*
 * 舵机多位置步进投放控制器 — C++ ROS2 节点 (drone_servo2).
 *
 * 投放序列 (可通过 YAML 配置):
 *   初始:  1150μs → 闭合, 挂载两枚载荷
 *   投放1: 1700μs → 释放第一枚
 *   投放2: 2150μs → 释放第二枚
 *
 * 舵机只进不退, 每次投放移动到下一个 PWM 位置.
 *
 * 架构:
 *   Mission FSM → TriggerDrop service → ServoController
 *     → VehicleCommand (DO_SET_ACTUATOR)
 *       → /drone/cmd/vehicle_command_relay
 *         → px4_bridge → PX4 → MAIN5 PWM 50Hz → 舵机
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <drone_interfaces/srv/trigger_drop.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

#include "drone_servo2/servo_interface.hpp"
#include "drone_servo2/drop_manager.hpp"

using namespace std::chrono_literals;
using drone_interfaces::srv::TriggerDrop;
using px4_msgs::msg::VehicleCommand;

// ============================================================================
// ServoMock — SITL 模式
// ============================================================================

namespace drone_servo2
{

class ServoMock : public IServo
{
public:
  explicit ServoMock(double init_pwm_us = 1150.0)
    : current_pwm_(init_pwm_us), init_pwm_(init_pwm_us)
  {}

  bool set_pwm(double pwm_us) override
  {
    current_pwm_ = pwm_us;
    return true;
  }

  bool is_ready() const override { return true; }

  std::map<std::string, double> status_dict() const override
  {
    return {
      {"pwm_us", current_pwm_},
      {"ready",  1.0},
    };
  }

  void reset() override { current_pwm_ = init_pwm_; }

  double current_pwm() const { return current_pwm_; }

private:
  double current_pwm_;
  double init_pwm_;
};

// ============================================================================
// PX4CommandServo — 通过 DDS VehicleCommand 控制飞控 MAIN 输出
// ============================================================================

class PX4CommandServo : public IServo
{
public:
  /// @param node             ROS2 节点指针
  /// @param init_pwm_us      初始 PWM (μs)
  /// @param pwm_min          PWM 范围下限 (μs), 对应 normalized=-1
  /// @param pwm_max          PWM 范围上限 (μs), 对应 normalized=+1
  /// @param actuator_index   控制索引 (0-based)
  /// @param actuator_set     PX4 Actuator Set
  PX4CommandServo(rclcpp::Node * node,
                  double init_pwm_us,
                  double pwm_min, double pwm_max,
                  int actuator_index, int actuator_set)
    : node_(node),
      current_pwm_(init_pwm_us),
      pwm_min_(pwm_min), pwm_max_(pwm_max),
      pwm_center_((pwm_min + pwm_max) / 2.0),
      pwm_half_range_((pwm_max - pwm_min) / 2.0),
      actuator_index_(actuator_index),
      actuator_set_(actuator_set)
  {
    cmd_pub_ = node_->create_publisher<VehicleCommand>(
      "/drone/cmd/vehicle_command_relay", 10);

    // 初始化: 舵机置为初始位置
    set_pwm(init_pwm_us);
    ready_ = true;
  }

  bool set_pwm(double pwm_us) override
  {
    if (!ready_) return false;
    current_pwm_ = pwm_us;
    send_actuator_cmd(pwm_to_normalized(pwm_us));
    return true;
  }

  bool is_ready() const override { return ready_; }

  std::map<std::string, double> status_dict() const override
  {
    return {
      {"pwm_us", current_pwm_},
      {"ready",  ready_ ? 1.0 : 0.0},
    };
  }

  void reset() override { set_pwm(pwm_min_); }

private:
  /// PWM μs → normalized [-1.0, +1.0], 线性映射到 [pwm_min, pwm_max]
  float pwm_to_normalized(double pwm_us) const
  {
    double clamped = std::max(pwm_min_, std::min(pwm_max_, pwm_us));
    return static_cast<float>((clamped - pwm_center_) / pwm_half_range_);
  }

  void send_actuator_cmd(float value)
  {
    auto cmd = VehicleCommand();
    cmd.timestamp = static_cast<uint64_t>(
      node_->get_clock()->now().nanoseconds() / 1000);
    cmd.command = 187;  // VEHICLE_CMD_DO_SET_ACTUATOR

    float params[7];
    for (int i = 0; i < 7; i++) params[i] = NAN;
    params[actuator_index_] = value;

    cmd.param1 = params[0];
    cmd.param2 = params[1];
    cmd.param3 = params[2];
    cmd.param4 = params[3];
    cmd.param5 = static_cast<double>(params[4]);
    cmd.param6 = static_cast<double>(params[5]);
    cmd.param7 = static_cast<float>(actuator_set_);

    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 100;
    cmd.confirmation = 0;
    cmd.from_external = true;

    cmd_pub_->publish(cmd);
  }

  rclcpp::Node *                   node_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr cmd_pub_;

  double current_pwm_;
  double pwm_min_, pwm_max_;
  double pwm_center_, pwm_half_range_;
  int    actuator_index_;
  int    actuator_set_;
  bool   ready_{false};
};

}  // namespace drone_servo2

// ============================================================================
// DropManager 实现
// ============================================================================

namespace drone_servo2
{

DropManager::DropManager(IServo * servo, std::vector<double> positions,
                         double delay_ms, double timeout_s)
  : servo_(servo),
    positions_(std::move(positions)),
    max_drops_(static_cast<int>(positions_.size()) - 1),
    delay_ms_(delay_ms),
    timeout_s_(timeout_s)
{
  if (max_drops_ < 1) {
    throw std::invalid_argument("positions must have at least 2 entries");
  }
}

bool DropManager::execute_drop(int cylinder_id)
{
  if (drop_count_ >= max_drops_) {
    state_ = DropState::FAILED;
    notify_done(false, cylinder_id);
    return false;
  }

  if (!servo_->is_ready()) {
    state_ = DropState::FAILED;
    notify_done(false, cylinder_id);
    return false;
  }

  state_ = DropState::MOVING;

  // 移动到下一个位置: positions[drop_count + 1]
  double target_pwm = positions_[drop_count_ + 1];

  bool success = false;
  try {
    success = servo_->set_pwm(target_pwm);
    if (success) {
      // 等待舵机动作完成
      std::this_thread::sleep_for(
        std::chrono::duration<double>(delay_ms_ / 1000.0));
    }
  } catch (...) {
    success = false;
  }

  if (success) {
    drop_count_++;
    state_ = DropState::DONE;
  } else {
    state_ = DropState::FAILED;
  }

  last_result_ = success;
  notify_done(success, cylinder_id);
  return success;
}

bool DropManager::can_drop() const
{
  return drop_count_ < max_drops_
         && servo_->is_ready()
         && state_ != DropState::MOVING;
}

void DropManager::reset()
{
  state_ = DropState::IDLE;
  drop_count_ = 0;
  last_result_ = false;
  servo_->set_pwm(positions_[0]);
}

void DropManager::notify_done(bool success, int cylinder_id)
{
  if (on_drop_done_) {
    try { on_drop_done_(success, cylinder_id); }
    catch (...) {}
  }
}

}  // namespace drone_servo2

// ============================================================================
// ServoControllerNode — ROS2 节点
// ============================================================================

namespace drone_servo2
{

class ServoControllerNode : public rclcpp::Node
{
public:
  ServoControllerNode()
    : Node("servo_controller")
  {
    declare_params();

    std::string platform = get_parameter("platform").as_string();
    bool is_sitl = (platform == "sitl");

    auto positions = get_parameter("pwm_positions").as_double_array();
    if (positions.size() < 2) {
      RCLCPP_ERROR(get_logger(), "pwm_positions 至少需要 2 个值");
      rclcpp::shutdown();
      return;
    }

    double init_pwm = positions[0];
    double pwm_min  = positions.front();
    double pwm_max  = positions.back();

    if (is_sitl) {
      servo_ = std::make_unique<ServoMock>(init_pwm);
      RCLCPP_INFO(get_logger(), "舵机: SITL Mock 模式");
    } else {
      servo_ = std::make_unique<PX4CommandServo>(
        this, init_pwm, pwm_min, pwm_max,
        get_parameter("actuator_index").as_int(),
        get_parameter("actuator_set").as_int());
      RCLCPP_INFO(get_logger(),
        "舵机: PX4 VehicleCommand 模式 "
        "(set=%ld idx=%ld pwm=[%.0f..%.0f] init=%.0fμs)",
        static_cast<long>(get_parameter("actuator_set").as_int()),
        static_cast<long>(get_parameter("actuator_index").as_int()),
        pwm_min, pwm_max, init_pwm);
    }

    // 投放管理器
    drop_mgr_ = std::make_unique<DropManager>(
      servo_.get(), positions,
      get_parameter("drop_delay_ms").as_double(),
      get_parameter("drop_timeout_s").as_double());
    drop_mgr_->set_on_drop_done(
      [this](bool success, int cyl_id) { on_drop_done(success, cyl_id); });

    // TriggerDrop 服务
    drop_srv_ = create_service<TriggerDrop>(
      "/drone/servo/trigger_drop",
      [this](const TriggerDrop::Request::SharedPtr req,
             TriggerDrop::Response::SharedPtr res)
      { handle_drop_request(req, res); });

    // 状态发布 (供 watchdog / 监控使用)
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/drone/servo/status", 10);
    status_timer_ = create_wall_timer(1s, [this]{ publish_status(); });

    // 打印投放序列
    RCLCPP_INFO(get_logger(), "投放序列 (%d 次):", drop_mgr_->max_drops());
    for (size_t i = 0; i < positions.size(); i++) {
      RCLCPP_INFO(get_logger(), "  [%zu] %.0f μs%s",
                  i, positions[i],
                  i == 0 ? " ← 初始/闭合" : "");
    }
    RCLCPP_INFO(get_logger(), "舵机投放服务已就绪");
  }

private:
  void declare_params()
  {
    declare_parameter("platform",        "sitl");
    declare_parameter("pwm_positions",
      std::vector<double>{1150.0, 1700.0, 2150.0});
    declare_parameter("actuator_index",  0);
    declare_parameter("actuator_set",    1);
    declare_parameter("drop_delay_ms",   1000.0);
    declare_parameter("drop_timeout_s",  3.0);
  }

  void handle_drop_request(
    const TriggerDrop::Request::SharedPtr req,
    TriggerDrop::Response::SharedPtr res)
  {
    if (!drop_mgr_->can_drop()) {
      RCLCPP_WARN(get_logger(),
        "投放请求被拒绝: state=%s count=%d/%d pwm=%.0fμs",
        drop_state_str(drop_mgr_->state()),
        drop_mgr_->drop_count(), drop_mgr_->max_drops(),
        drop_mgr_->current_pwm());
      res->success = false;
      res->message = std::string("Cannot drop: ")
                   + drop_state_str(drop_mgr_->state());
      return;
    }

    double target = drop_mgr_->positions()[drop_mgr_->drop_count() + 1];
    RCLCPP_INFO(get_logger(),
      "投放 #%d: cyl=%d %.0f → %.0f μs",
      drop_mgr_->drop_count() + 1, req->cylinder_id,
      drop_mgr_->current_pwm(), target);

    bool ok = drop_mgr_->execute_drop(req->cylinder_id);
    res->success = ok;
    res->message = ok ? "Drop OK" : "Drop FAILED";
  }

  void on_drop_done(bool success, int cylinder_id)
  {
    RCLCPP_INFO(get_logger(),
      "投放 #%d 完成: %s (cyl=%d) pwm=%.0fμs",
      drop_mgr_->drop_count(),
      success ? "成功" : "失败",
      cylinder_id,
      drop_mgr_->current_pwm());
  }

  void publish_status()
  {
    auto st = servo_->status_dict();
    RCLCPP_DEBUG(get_logger(),
      "舵机状态: pwm=%.0fμs ready=%.0f",
      st["pwm_us"], st["ready"]);

    // 发布状态话题 (供 FSM watchdog 等消费)
    auto msg = std_msgs::msg::String();
    msg.data = "pwm=" + std::to_string(static_cast<int>(st["pwm_us"]))
             + " ready=" + std::to_string(static_cast<int>(st["ready"]))
             + " drops=" + std::to_string(drop_mgr_->drop_count())
             + "/" + std::to_string(drop_mgr_->max_drops());
    status_pub_->publish(msg);
  }

  std::unique_ptr<IServo>      servo_;
  std::unique_ptr<DropManager> drop_mgr_;

  rclcpp::Service<TriggerDrop>::SharedPtr    drop_srv_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr               status_timer_;
};

}  // namespace drone_servo2

// ============================================================================
// 入口
// ============================================================================

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<drone_servo2::ServoControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
