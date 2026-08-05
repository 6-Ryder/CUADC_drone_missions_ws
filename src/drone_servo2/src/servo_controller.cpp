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
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <drone_interfaces/srv/trigger_drop.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>

#include "drone_servo2/servo_interface.hpp"
#include "drone_servo2/drop_manager.hpp"

using namespace std::chrono_literals;
using drone_interfaces::srv::TriggerDrop;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleCommandAck;

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

  bool is_ready() const override {return true;}

  std::map<std::string, double> status_dict() const override
  {
    return {
      {"pwm_us", current_pwm_},
      {"ready", 1.0},
    };
  }

  void reset() override {current_pwm_ = init_pwm_;}

  double current_pwm() const {return current_pwm_;}

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
  PX4CommandServo(
    rclcpp::Node * node,
    double init_pwm_us,
    double pwm_min, double pwm_max,
    double command_timeout_s,
    int command_retries)
  : node_(node),
    init_pwm_(init_pwm_us),
    current_pwm_(init_pwm_us),
    pwm_min_(pwm_min), pwm_max_(pwm_max),
    pwm_center_((pwm_min + pwm_max) / 2.0),
    pwm_half_range_((pwm_max - pwm_min) / 2.0),
    command_timeout_s_(command_timeout_s),
    command_retries_(command_retries)
  {
    if (pwm_min_ >= pwm_max_) {
      throw std::invalid_argument("pwm_min must be less than pwm_max");
    }
    if (command_retries_ < 0) {
      throw std::invalid_argument("command_retries must be non-negative");
    }
    cmd_pub_ = node_->create_publisher<VehicleCommand>(
      "/drone/cmd/vehicle_command_relay", 10);

    ack_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions options;
    options.callback_group = ack_group_;
    ack_sub_ = node_->create_subscription<VehicleCommandAck>(
      "/fmu/out/vehicle_command_ack", rclcpp::SensorDataQoS(),
      [this](const VehicleCommandAck::SharedPtr msg) {handle_ack(*msg);},
      options);
  }

  bool initialize()
  {
    if (ready_.load()) {return true;}
    if (!send_pwm_command(init_pwm_)) {return false;}
    ready_.store(true);
    return true;
  }

  bool set_pwm(double pwm_us) override
  {
    if (!ready_.load()) {return false;}
    return send_pwm_command(pwm_us);
  }

  bool is_ready() const override {return ready_.load();}

  std::map<std::string, double> status_dict() const override
  {
    return {
      {"pwm_us", current_pwm_.load()},
      {"ready", ready_.load() ? 1.0 : 0.0},
    };
  }

  void reset() override {set_pwm(pwm_min_);}

private:
  bool send_pwm_command(double pwm_us)
  {
    for (int attempt = 0; attempt <= command_retries_; ++attempt) {
      {
        std::lock_guard<std::mutex> lock(ack_mutex_);
        awaiting_ack_ = true;
        ack_received_ = false;
        ack_accepted_ = false;
      }

      // Retrying must use this exact absolute position; DropManager has not
      // advanced its sequence until this method returns true.
      send_actuator_cmd(pwm_to_normalized(pwm_us));

      std::unique_lock<std::mutex> lock(ack_mutex_);
      bool received = ack_cv_.wait_for(
        lock, std::chrono::duration<double>(command_timeout_s_),
        [this] {return ack_received_;});
      awaiting_ack_ = false;

      if (received) {
        if (!ack_accepted_) {
          // A real rejection is definitive. Never repeat a rejected release.
          RCLCPP_ERROR(node_->get_logger(), "PX4 拒绝舵机命令");
          return false;
        }

        // The only mutation of the trusted actuator position follows an
        // accepted ACK, including after a timeout retry.
        current_pwm_.store(pwm_us);
        return true;
      }

      if (attempt < command_retries_) {
        RCLCPP_WARN(
          node_->get_logger(),
          "PX4 舵机命令 ACK 超时 (%.1fs), 重发相同 PWM %.0fμs (%d/%d)",
          command_timeout_s_, pwm_us, attempt + 1, command_retries_);
        continue;
      }

      RCLCPP_ERROR(
        node_->get_logger(),
        "PX4 舵机命令 ACK 超时 (%.1fs), 同一 PWM 重试耗尽",
        command_timeout_s_);
      return false;
    }

    return false;
  }

  void handle_ack(const VehicleCommandAck & ack)
  {
    if (ack.command != VehicleCommand::VEHICLE_CMD_DO_SET_ACTUATOR) {return;}
    if (ack.target_component != 100) {return;}

    std::lock_guard<std::mutex> lock(ack_mutex_);
    if (!awaiting_ack_) {return;}
    if (ack.result == VehicleCommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS) {return;}

    ack_accepted_ =
      ack.result == VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;
    ack_received_ = true;
    ack_cv_.notify_one();
  }

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
    cmd.command = VehicleCommand::VEHICLE_CMD_DO_SET_ACTUATOR;

    // MAIN5 is mapped in PX4 to Peripheral via Actuator Set 1.
    cmd.param1 = value;
    cmd.param2 = NAN;
    cmd.param3 = NAN;
    cmd.param4 = NAN;
    cmd.param5 = NAN;
    cmd.param6 = NAN;
    cmd.param7 = 0.0F;

    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 100;
    cmd.confirmation = 0;
    cmd.from_external = true;

    cmd_pub_->publish(cmd);
  }

  rclcpp::Node * node_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr cmd_pub_;
  rclcpp::Subscription<VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::CallbackGroup::SharedPtr ack_group_;

  double init_pwm_;
  std::atomic<double> current_pwm_;
  double pwm_min_, pwm_max_;
  double pwm_center_, pwm_half_range_;
  double command_timeout_s_;
  int command_retries_;
  std::atomic<bool> ready_{false};
  std::mutex ack_mutex_;
  std::condition_variable ack_cv_;
  bool awaiting_ack_{false};
  bool ack_received_{false};
  bool ack_accepted_{false};
};

}  // namespace drone_servo2

// ============================================================================
// DropManager 实现
// ============================================================================

namespace drone_servo2
{

DropManager::DropManager(
  IServo * servo, std::vector<double> positions,
  double delay_ms)
: servo_(servo),
  positions_(std::move(positions)),
  max_drops_(static_cast<int>(positions_.size()) - 1),
  delay_ms_(delay_ms)
{
  if (max_drops_ < 1) {
    throw std::invalid_argument("positions must have at least 2 entries");
  }
}

bool DropManager::execute_drop(int cylinder_id)
{
  bool success = false;
  {
    // Serialize the whole move so a second request cannot select the same
    // position.  Do not call user callbacks while this mutex is held.
    std::lock_guard<std::mutex> lock(mutex_);

    if (drop_count_ >= max_drops_ || !servo_->is_ready()) {
      state_ = DropState::FAILED;
      last_result_ = false;
    } else {
      state_ = DropState::MOVING;

      // 移动到下一个位置: positions[drop_count + 1]
      const double target_pwm = positions_[drop_count_ + 1];
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
    }
  }

  notify_done(success, cylinder_id);
  return success;
}

bool DropManager::can_drop() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return drop_count_ < max_drops_ &&
         servo_->is_ready() &&
         state_ != DropState::MOVING;
}

void DropManager::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = DropState::IDLE;
  drop_count_ = 0;
  last_result_ = false;
  servo_->set_pwm(positions_[0]);
}

void DropManager::notify_done(bool success, int cylinder_id)
{
  if (on_drop_done_) {
    try {
      on_drop_done_(success, cylinder_id);
    } catch (...) {
    }
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
    double pwm_min = positions.front();
    double pwm_max = positions.back();
    double delay_ms = get_parameter("drop_delay_ms").as_double();
    double timeout_s = get_parameter("drop_timeout_s").as_double();
    int command_retries = static_cast<int>(
      get_parameter("command_retries").as_int());
    double command_budget_s = timeout_s - delay_ms / 1000.0;
    if (command_retries < 0 || command_budget_s <= 0.0) {
      throw std::invalid_argument(
              "invalid command_retries or drop_timeout_s budget");
    }
    double command_timeout_s = command_budget_s / (command_retries + 1);

    if (is_sitl) {
      servo_ = std::make_unique<ServoMock>(init_pwm);
      RCLCPP_INFO(get_logger(), "舵机: SITL Mock 模式");
    } else {
      auto hardware_servo = std::make_unique<PX4CommandServo>(
        this, init_pwm, pwm_min, pwm_max,
        command_timeout_s, command_retries);
      hardware_servo_ = hardware_servo.get();
      servo_ = std::move(hardware_servo);
      RCLCPP_INFO(
        get_logger(),
        "舵机: PX4 VehicleCommand → MAIN5 模式 "
        "(Actuator Set 1, pwm=[%.0f..%.0f] init=%.0fμs)",
        pwm_min, pwm_max, init_pwm);
    }

    // 投放管理器
    drop_mgr_ = std::make_unique<DropManager>(
      servo_.get(), positions, delay_ms);
    drop_mgr_->set_on_drop_done(
      [this](bool success, int cyl_id) {on_drop_done(success, cyl_id);});

    // TriggerDrop 服务
    drop_srv_ = create_service<TriggerDrop>(
      "/drone/servo/trigger_drop",
      [this](const TriggerDrop::Request::SharedPtr req,
      TriggerDrop::Response::SharedPtr res)
      {handle_drop_request(req, res);});

    // 状态发布 (供 watchdog / 监控使用)
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/drone/servo/status", 10);
    status_timer_ = create_wall_timer(1s, [this] {publish_status();});

    // 任务租约: px4_bridge 在租约无效时静默丢弃 /drone/cmd/vehicle_command_relay,
    // 提前发 init 命令只会得到 ACK 超时 (真机 2026-08-05 日志实测: 前两次 init
    // 超时, 租约随 FSM 启动后才成功)。等 lease=true 再初始化, 消除启动竞态。
    lease_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/drone/cmd/autonomy_lease", rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        lease_valid_.store(msg->data);
      });

    if (hardware_servo_) {
      init_timer_ = create_wall_timer(
        1s, [this] {
          if (!lease_valid_.load()) {
            if (++lease_wait_ticks_ % 10 == 1) {
              RCLCPP_WARN(
                get_logger(),
                "等待任务租约 (autonomy_lease), 租约生效后才初始化 MAIN5");
            }
            return;
          }
          if (hardware_servo_->initialize()) {
            RCLCPP_INFO(get_logger(), "MAIN5 已确认处于初始闭合位置");
            init_timer_->cancel();
          } else {
            RCLCPP_WARN(get_logger(), "MAIN5 初始化未获 PX4 ACK, 将重试");
            init_timer_->reset();
          }
        });
    }

    // 打印投放序列
    RCLCPP_INFO(get_logger(), "投放序列 (%d 次):", drop_mgr_->max_drops());
    for (size_t i = 0; i < positions.size(); i++) {
      RCLCPP_INFO(
        get_logger(), "  [%zu] %.0f μs%s",
        i, positions[i],
        i == 0 ? " ← 初始/闭合" : "");
    }
    RCLCPP_INFO(get_logger(), "舵机投放服务已就绪");
  }

private:
  void declare_params()
  {
    declare_parameter("platform", "sitl");
    declare_parameter(
      "pwm_positions",
      std::vector<double>{1150.0, 1700.0, 2150.0});
    declare_parameter("drop_delay_ms", 1000.0);
    declare_parameter("drop_timeout_s", 3.0);
    declare_parameter("command_retries", 1);
  }

  void handle_drop_request(
    const TriggerDrop::Request::SharedPtr req,
    TriggerDrop::Response::SharedPtr res)
  {
    if (!drop_mgr_->can_drop()) {
      RCLCPP_WARN(
        get_logger(),
        "投放请求被拒绝: state=%s count=%d/%d pwm=%.0fμs",
        drop_state_str(drop_mgr_->state()),
        drop_mgr_->drop_count(), drop_mgr_->max_drops(),
        drop_mgr_->current_pwm());
      res->success = false;
      res->message = std::string("Cannot drop: ") +
        drop_state_str(drop_mgr_->state());
      return;
    }

    double target = drop_mgr_->positions()[drop_mgr_->drop_count() + 1];
    RCLCPP_INFO(
      get_logger(),
      "投放 #%d: cyl=%d %.0f → %.0f μs",
      drop_mgr_->drop_count() + 1, req->cylinder_id,
      drop_mgr_->current_pwm(), target);

    bool ok = drop_mgr_->execute_drop(req->cylinder_id);
    res->success = ok;
    res->message = ok ? "Drop OK" : "Drop FAILED";
  }

  void on_drop_done(bool success, int cylinder_id)
  {
    RCLCPP_INFO(
      get_logger(),
      "投放 #%d 完成: %s (cyl=%d) pwm=%.0fμs",
      drop_mgr_->drop_count(),
      success ? "成功" : "失败",
      cylinder_id,
      drop_mgr_->current_pwm());
  }

  void publish_status()
  {
    auto st = servo_->status_dict();
    RCLCPP_DEBUG(
      get_logger(),
      "舵机状态: pwm=%.0fμs ready=%.0f",
      st["pwm_us"], st["ready"]);

    // 发布状态话题 (供 FSM watchdog 等消费)
    auto msg = std_msgs::msg::String();
    msg.data = "pwm=" + std::to_string(static_cast<int>(st["pwm_us"])) +
      " ready=" + std::to_string(static_cast<int>(st["ready"])) +
      " drops=" + std::to_string(drop_mgr_->drop_count()) +
      "/" + std::to_string(drop_mgr_->max_drops());
    status_pub_->publish(msg);
  }

  std::unique_ptr<IServo> servo_;
  std::unique_ptr<DropManager> drop_mgr_;
  PX4CommandServo * hardware_servo_{nullptr};

  rclcpp::Service<TriggerDrop>::SharedPtr drop_srv_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lease_sub_;
  std::atomic<bool> lease_valid_{false};
  int lease_wait_ticks_{0};
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr init_timer_;
};

}  // namespace drone_servo2

// ============================================================================
// 入口
// ============================================================================

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<drone_servo2::ServoControllerNode>();
    rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("servo_controller_main"),
      "ServoControllerNode 构造失败: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
