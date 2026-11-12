#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/motors_order.hpp"
#include "interfaces/msg/ultrasonic.hpp"
#include "interfaces/msg/log_entry.hpp"
#include <algorithm>
#include <chrono>
#include <limits>
#include <string>

using std::placeholders::_1;
using namespace std::chrono_literals;

class SafetyStopNode : public rclcpp::Node {
public:
  SafetyStopNode() : rclcpp::Node("safety_stop_node"), steady_clock_(RCL_STEADY_TIME)
  {
    // Paramètres distance
    stop_dist_front_cm_ = declare_parameter<int>("stop_dist_front_cm", 20);
    stop_dist_rear_cm_  = declare_parameter<int>("stop_dist_rear_cm", 20);
    slow_dist_front_cm_ = declare_parameter<int>("slow_dist_front_cm", 45);
    slow_dist_rear_cm_  = declare_parameter<int>("slow_dist_rear_cm", 45);

    us_timeout_ms_      = declare_parameter<int>("us_timeout_ms", 200);
    cmd_timeout_ms_     = declare_parameter<int>("cmd_timeout_ms", 200);
    log_actions_        = declare_parameter<bool>("log_actions", true);

    // Rampe asymétrique
    max_delta_pwm_up_   = declare_parameter<int>("max_delta_pwm_up_per_msg", 1);
    max_delta_pwm_down_ = declare_parameter<int>("max_delta_pwm_down_per_msg", 1);
    max_delta_pwm_up_   = std::clamp(max_delta_pwm_up_,   0, 100);
    max_delta_pwm_down_ = std::clamp(max_delta_pwm_down_, 0, 100);

    // QoS
    auto qos_sensor = rclcpp::SensorDataQoS();  // /us_data

    rclcpp::QoS qos_cmd(rclcpp::KeepLast(10));  // = can_tx_node
    qos_cmd.reliable();

    // Publisher : commandes "filtrées"
    pub_safe_order_ = create_publisher<interfaces::msg::MotorsOrder>(
        "motors_order", qos_cmd);

    // Publisher : logging stops to web interface
    pub_log_ = create_publisher<interfaces::msg::LogEntry>("/logger", 10);

    // Souscriptions
    sub_us_ = create_subscription<interfaces::msg::Ultrasonic>(
        "us_data", qos_sensor,
        std::bind(&SafetyStopNode::onUltrasonic, this, _1));

    sub_raw_order_ = create_subscription<interfaces::msg::MotorsOrder>(
        "motors_order_raw", qos_cmd,
        std::bind(&SafetyStopNode::onMotorsOrderRaw, this, _1));

    // Timer santé + watchdog
    timer_health_ = create_wall_timer(50ms, std::bind(&SafetyStopNode::onTimer, this));

    last_us_time_   = now();
    last_cmd_time_  = steady_clock_.now();

    prev_cmd_valid_ = false;

    RCLCPP_INFO(get_logger(),
        "safety_stop_node READY (front_stop=%d cm, rear_stop=%d cm, "
        "front_slow=%d cm, rear_slow=%d cm, us_to=%d ms, cmd_to=%d ms, "
        "dPWM_up=%d/msg, dPWM_down=%d/msg)",
        stop_dist_front_cm_, stop_dist_rear_cm_,
        slow_dist_front_cm_, slow_dist_rear_cm_,
        us_timeout_ms_, cmd_timeout_ms_,
        max_delta_pwm_up_, max_delta_pwm_down_);
  }

private:
  // ---- Données ----
  rclcpp::Publisher<interfaces::msg::MotorsOrder>::SharedPtr pub_safe_order_;
  rclcpp::Publisher<interfaces::msg::LogEntry>::SharedPtr pub_log_;
  rclcpp::Subscription<interfaces::msg::Ultrasonic>::SharedPtr sub_us_;
  rclcpp::Subscription<interfaces::msg::MotorsOrder>::SharedPtr sub_raw_order_;
  rclcpp::TimerBase::SharedPtr timer_health_;

  interfaces::msg::Ultrasonic last_us_;
  rclcpp::Time last_us_time_;
  bool have_us_ = false;

  // Watchdog & sécurité
  rclcpp::Clock steady_clock_;
  rclcpp::Time last_cmd_time_;
  int stop_dist_front_cm_;
  int stop_dist_rear_cm_;
  int slow_dist_front_cm_;
  int slow_dist_rear_cm_;
  int us_timeout_ms_;
  int cmd_timeout_ms_;
  bool log_actions_;

  // Rampe
  int max_delta_pwm_up_;
  int max_delta_pwm_down_;
  interfaces::msg::MotorsOrder prev_cmd_;
  bool prev_cmd_valid_;

  // To avoid sending several log messages per stop:
  bool safety_stop_activated = false;

  // ---- Utils ----
  static inline int clamp_pwm(int v) { return std::max(0, std::min(100, v)); }

  const int int_max = std::numeric_limits<int>::max();

  enum us_direction_t {
    LEFT,
    MIDDLE,
    RIGHT,
  };

  struct us_readout_t {
    int us_value;
    us_direction_t direction;
  };

  static inline std::string dirToString(us_direction_t d) {
    switch (d) {
      case LEFT:   return "LEFT";
      case MIDDLE: return "MIDDLE";
      case RIGHT:  return "RIGHT";
      default:     return "UNKNOWN";
    }
  }

  // From a us readout message and forward/backward direction indication,
  // returns the value and direction (left/middle/right) of the smallest readout.
  us_readout_t smallest_us(const interfaces::msg::Ultrasonic &msg, bool forward) {
    int left, middle, right;

    if (forward) {
      left   = msg.front_left;
      middle = msg.front_center;
      right  = msg.front_right;
    } else {
      left   = msg.rear_left;
      middle = msg.rear_center;
      right  = msg.rear_right;
    }

    int smallest = int_max;
    us_readout_t result{};
    result.us_value = int_max;
    result.direction = MIDDLE;

    if (left < smallest) {
      smallest = left;
      result.us_value = left;
      result.direction = LEFT;
    }
    if (middle < smallest) {
      smallest = middle;
      result.us_value = middle;
      result.direction = MIDDLE;
    }
    if (right < smallest) {
      smallest = right;
      result.us_value = right;
      result.direction = RIGHT;
    }

    return result;
  }

  // Clamp forward : on limite l'amplitude vers 100 (PWM > 50)
  int computeClampedPwmForward(int pwm_req, int d, int d_stop, int d_slow, bool *stopped) {
    if (stopped) *stopped = false;
    pwm_req = clamp_pwm(pwm_req);

    // Sécurité: mesure invalide => STOP fail-safe
    if (d < 0) {
      if (stopped) *stopped = true;
      return 50;
    }

    if (d_slow <= d_stop) {
      if (d <= d_stop) {
        if (stopped) *stopped = true;
        return 50;
      }
      return pwm_req;
    }

    if (d <= d_stop) {
      if (stopped) *stopped = true;
      return 50;
    }

    if (d >= d_slow) {
      safety_stop_activated = false;
      return pwm_req;
    }

    float t = static_cast<float>(d - d_stop) /
              static_cast<float>(d_slow - d_stop);
    int pwm_max = 50 + static_cast<int>(t * (100 - 50) + 0.5f);
    pwm_max = clamp_pwm(pwm_max);
    return std::min(pwm_req, pwm_max);
  }

  // Clamp backward : on limite l'amplitude vers 0 (PWM < 50)
  int computeClampedPwmBackward(int pwm_req, int d, int d_stop, int d_slow, bool *stopped) {
    if (stopped) *stopped = false;
    pwm_req = clamp_pwm(pwm_req);

    // Sécurité: mesure invalide => STOP fail-safe
    if (d < 0) {
      if (stopped) *stopped = true;
      return 50;
    }

    if (d_slow <= d_stop) {
      if (d <= d_stop) {
        if (stopped) *stopped = true;
        return 50;
      }
      return pwm_req;
    }

    if (d <= d_stop) {
      if (stopped) *stopped = true;
      return 50;
    }

    if (d >= d_slow) {
      safety_stop_activated = false;
      return pwm_req;
    }

    float t = static_cast<float>(d - d_stop) /
              static_cast<float>(d_slow - d_stop);
    float dev_max = t * 50.0f;
    int pwm_min = 50 - static_cast<int>(dev_max + 0.5f);

    pwm_min = clamp_pwm(pwm_min);

    if (pwm_req < pwm_min) {
      return pwm_min;
    }
    return pwm_req;
  }

  // Applique la rampe aux PWM arrière
  void applyRamp(interfaces::msg::MotorsOrder &cmd) {
    if (!prev_cmd_valid_) {
      cmd.left_rear_pwm  = clamp_pwm(cmd.left_rear_pwm);
      cmd.right_rear_pwm = clamp_pwm(cmd.right_rear_pwm);
      prev_cmd_ = cmd;
      prev_cmd_valid_ = true;
      return;
    }

    auto rampOne = [this](int prev, int target) -> int {
      target = clamp_pwm(target);
      prev   = clamp_pwm(prev);
      int diff = target - prev;

      if (diff > 0) {
        if (diff > max_delta_pwm_up_) diff = max_delta_pwm_up_;
      } else if (diff < 0) {
        if (diff < -max_delta_pwm_down_) diff = -max_delta_pwm_down_;
      }

      return clamp_pwm(prev + diff);
    };

    cmd.left_rear_pwm  = rampOne(prev_cmd_.left_rear_pwm,  cmd.left_rear_pwm);
    cmd.right_rear_pwm = rampOne(prev_cmd_.right_rear_pwm, cmd.right_rear_pwm);

    prev_cmd_ = cmd;
    prev_cmd_valid_ = true;
  }

  // Publication centralisée
  void publishSafeOrder(const interfaces::msg::MotorsOrder &input,
                        bool bypass_ramp = false) {
    interfaces::msg::MotorsOrder out = input;

    if (!bypass_ramp) {
      applyRamp(out);
    } else {
      out.left_rear_pwm  = clamp_pwm(out.left_rear_pwm);
      out.right_rear_pwm = clamp_pwm(out.right_rear_pwm);
      prev_cmd_ = out;
      prev_cmd_valid_ = true;
    }

    pub_safe_order_->publish(out);
  }

  // Publish log to web interface
  void web_logger(int lvl, std::string sender, std::string message) {
    auto msg = interfaces::msg::LogEntry();
    msg.level = lvl;
    msg.sender = sender;
    msg.message = message;
    pub_log_->publish(msg);
  }

  // ---- Callbacks ----
  void onUltrasonic(const interfaces::msg::Ultrasonic &msg) {
    last_us_ = msg;
    last_us_time_ = now();
    have_us_ = true;
  }

  void onMotorsOrderRaw(const interfaces::msg::MotorsOrder &raw) {
    last_cmd_time_ = steady_clock_.now();

    interfaces::msg::MotorsOrder safe = raw;

    // Sens par roue (important pour les rotations sur place)
    const double dev_left  = static_cast<int>(raw.left_rear_pwm)  - 50.0;
    const double dev_right = static_cast<int>(raw.right_rear_pwm) - 50.0;

    const bool left_forward   = dev_left  >  0.5;
    const bool left_backward  = dev_left  < -0.5;
    const bool right_forward  = dev_right >  0.5;
    const bool right_backward = dev_right < -0.5;

    const bool us_fresh =
        have_us_ &&
        ((now() - last_us_time_).nanoseconds() / 1'000'000 <= us_timeout_ms_);

    bool sensor_fail_stop = false;
    bool hard_stop_due_to_obstacle = false;

    // Prépare mesures min front/rear (avec direction) si capteur OK
    us_readout_t front_min{};
    us_readout_t rear_min{};
    if (us_fresh) {
      front_min = smallest_us(last_us_, /*forward=*/true);
      rear_min  = smallest_us(last_us_, /*forward=*/false);
    }

    if (!us_fresh) {
      // Fail-safe : pas d'info fiable → STOP direct, sans rampe
      sensor_fail_stop = true;
      if (log_actions_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "No fresh US data (>%d ms). Hard STOP for safety.",
            us_timeout_ms_);
        if (!safety_stop_activated) {
          web_logger(2, "safety_stop_node",
                     "safety_stop_node stopping car due to: [ULTRASOUND SENSOR DATA TIMEOUT]");
        }
        safety_stop_activated = true;
      }
    } else {
      bool stopped_front = false;
      bool stopped_rear  = false;

      // Clamp par roue selon son sens
      if (left_forward) {
        bool s = false;
        safe.left_rear_pwm = computeClampedPwmForward(
            safe.left_rear_pwm,
            front_min.us_value,
            stop_dist_front_cm_,
            slow_dist_front_cm_,
            &s);
        stopped_front = stopped_front || s;
      } else if (left_backward) {
        bool s = false;
        safe.left_rear_pwm = computeClampedPwmBackward(
            safe.left_rear_pwm,
            rear_min.us_value,
            stop_dist_rear_cm_,
            slow_dist_rear_cm_,
            &s);
        stopped_rear = stopped_rear || s;
      }

      if (right_forward) {
        bool s = false;
        safe.right_rear_pwm = computeClampedPwmForward(
            safe.right_rear_pwm,
            front_min.us_value,
            stop_dist_front_cm_,
            slow_dist_front_cm_,
            &s);
        stopped_front = stopped_front || s;
      } else if (right_backward) {
        bool s = false;
        safe.right_rear_pwm = computeClampedPwmBackward(
            safe.right_rear_pwm,
            rear_min.us_value,
            stop_dist_rear_cm_,
            slow_dist_rear_cm_,
            &s);
        stopped_rear = stopped_rear || s;
      }

      // Si on est dans la zone STOP (avant ou arrière), on impose un STOP dur immédiat (bypass rampe)
      if (stopped_front || stopped_rear) {
        hard_stop_due_to_obstacle = true;
        safe.left_rear_pwm  = 50;
        safe.right_rear_pwm = 50;

        if (log_actions_ && !safety_stop_activated) {
          if (stopped_front) {
            web_logger(
              2, "safety_stop_node",
              "safety_stop_node stopping car due to: [ULTRASOUND FRONT " +
                dirToString(front_min.direction) +
                " DETECTED OBJECT TOO CLOSE WHILE MOVING FORWARD (limit=" +
                std::to_string(stop_dist_front_cm_) + "cm)]");
          } else {
            web_logger(
              2, "safety_stop_node",
              "safety_stop_node stopping car due to: [ULTRASOUND REAR " +
                dirToString(rear_min.direction) +
                " DETECTED OBJECT TOO CLOSE WHILE MOVING BACKWARD (limit=" +
                std::to_string(stop_dist_rear_cm_) + "cm)]");
          }
        }
        safety_stop_activated = true;
      }

      if (log_actions_) {
        RCLCPP_DEBUG(get_logger(),
                     "US(front=%dcm %s, rear=%dcm %s) req=(%d,%d) -> safe=(%d,%d) hard_stop=%s",
                     front_min.us_value, dirToString(front_min.direction).c_str(),
                     rear_min.us_value,  dirToString(rear_min.direction).c_str(),
                     raw.left_rear_pwm, raw.right_rear_pwm,
                     safe.left_rear_pwm, safe.right_rear_pwm,
                     hard_stop_due_to_obstacle ? "true" : "false");
      }
    }

    if (sensor_fail_stop) {
      // Hard stop (capteur mort) : 50 direct, pas de rampe
      safe.left_rear_pwm  = 50;
      safe.right_rear_pwm = 50;
      publishSafeOrder(safe, /*bypass_ramp=*/true);
    } else if (hard_stop_due_to_obstacle) {
      // STOP obstacle : 50 direct, pas de rampe (important)
      publishSafeOrder(safe, /*bypass_ramp=*/true);
    } else {
      // Cas normal : clamp distance + rampe
      publishSafeOrder(safe, /*bypass_ramp=*/false);
    }
  }

  void onTimer() {
    if (!have_us_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "No ultrasonic data received on /us_data yet.");
    }

    // Watchdog commandes
    const auto age_ms =
        (steady_clock_.now() - last_cmd_time_).nanoseconds() / 1'000'000;
    if (age_ms > cmd_timeout_ms_) {
      interfaces::msg::MotorsOrder stop;
      stop.left_rear_pwm  = 50;
      stop.right_rear_pwm = 50;
      publishSafeOrder(stop, /*bypass_ramp=*/true);

      if (log_actions_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Cmd timeout > %d ms (age=%ld ms): forcing hard STOP.",
            cmd_timeout_ms_, (long)age_ms);
        if (!safety_stop_activated) {
          web_logger(2, "safety_stop_node",
                     "safety_stop_node stopping car due to: [WATCHDOG TIMEOUT]");
        }
        safety_stop_activated = true;
      }
    }
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyStopNode>());
  rclcpp::shutdown();
  return 0;
}

