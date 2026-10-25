#include "ego_gazebo/frames.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <chrono>
#include <limits>
#include <optional>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using ego_gazebo::enu_to_ned;
using ego_gazebo::px4_topic;
using Vec = Eigen::Vector3d;
using Command = quadrotor_msgs::msg::PositionCommand;
using Goal = geometry_msgs::msg::PoseStamped;
using Trigger = std_srvs::srv::Trigger;
static double wall_now() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
template<class T> Vec vector(const T &p) { return {p.x, p.y, p.z}; }
static std::array<float, 3> floats(const Vec &p) {
  return {static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z())};
}

class Offboard : public rclcpp::Node {
 public:
  Offboard() : Node("offboard") {
    enabled_ = declare_parameter("auto_arm", false);
    height_ = declare_parameter("takeoff_height", 1.8);
    if (height_ < 0.6 || height_ > 4.2) throw std::invalid_argument("takeoff_height must be in [0.6,4.2]");
    mode_pub_ = create_publisher<OffboardControlMode>(px4_topic<OffboardControlMode>("in", "offboard_control_mode"), 10);
    sp_pub_ = create_publisher<TrajectorySetpoint>(px4_topic<TrajectorySetpoint>("in", "trajectory_setpoint"), 10);
    vehicle_pub_ = create_publisher<VehicleCommand>(px4_topic<VehicleCommand>("in", "vehicle_command"), 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("/sim/offboard_state", rclcpp::QoS(1).transient_local());
    goal_pub_ = create_publisher<Goal>("/move_base_simple/goal", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/sim/odom", 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        if (vector(m->pose.pose.position).allFinite()) { odom_ = *m; odom_wall_ = wall_now(); }
      });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>("/camera/depth/image_rect_raw", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr m) { if (m->width && m->height && m->encoding == "32FC1") depth_wall_ = wall_now(); });
    local_sub_ = create_subscription<VehicleLocalPosition>(px4_topic<VehicleLocalPosition>("out", "vehicle_local_position"), rclcpp::SensorDataQoS(),
      [this](VehicleLocalPosition::ConstSharedPtr m) {
        local_ = *m; local_wall_ = wall_now();
      });
    status_sub_ = create_subscription<VehicleStatus>(px4_topic<VehicleStatus>("out", "vehicle_status"), rclcpp::SensorDataQoS(),
      [this](VehicleStatus::ConstSharedPtr m) { status_ = *m; status_wall_ = wall_now(); });
    ack_sub_ = create_subscription<VehicleCommandAck>(px4_topic<VehicleCommandAck>("out", "vehicle_command_ack"), rclcpp::SensorDataQoS(),
      [this](VehicleCommandAck::ConstSharedPtr m) { RCLCPP_INFO(get_logger(), "PX4 command %u result %u", m->command, m->result); });
    command_sub_ = create_subscription<Command>("/position_cmd", 10,
      [this](Command::ConstSharedPtr m) {
        const double age = (now() - rclcpp::Time(m->header.stamp)).seconds();
        if (m->trajectory_flag == Command::TRAJECTORY_STATUS_READY &&
            ego_gazebo::within_flight_limits(vector(m->position)) && vector(m->velocity).allFinite() &&
            vector(m->acceleration).allFinite() && std::isfinite(m->yaw) && std::isfinite(m->yaw_dot) &&
            age >= -0.1 && age < 0.5 && (m->header.frame_id.empty() || m->header.frame_id == "world")) {
          cmd_ = *m; cmd_wall_ = wall_now();
        }
      });
    goal_sub_ = create_subscription<Goal>("/goal_pose", 10, [this](Goal::ConstSharedPtr msg) {
      auto m = *msg;
      if (m.pose.position.z == 0.0) m.pose.position.z = height_;
      if ((!m.header.frame_id.empty() && m.header.frame_id != "world") ||
          !ego_gazebo::within_flight_limits(vector(m.pose.position))) {
        RCLCPP_ERROR(get_logger(), "Goal must be in world frame and within RMUC flight limits"); return;
      }
      pending_goal_ = m;
    });
    start_srv_ = create_service<Trigger>("/sim/start", [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
      r->success = !terminal_; r->message = terminal_ ? "Restart after landing or mode loss" : "Waiting for SITL sensors";
      if (!terminal_) enabled_ = true;
    });
    land_srv_ = create_service<Trigger>("/sim/land", [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
      vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
      terminal_ = true; state("LANDING"); r->success = true; r->message = "Land requested; check PX4 ACK";
    });
    timer_ = create_wall_timer(20ms, [this] { tick(); });
    state_timer_ = create_wall_timer(500ms, [this] { publish_state(); });
  }
 private:
  void publish_state() { std_msgs::msg::String m; m.data = state_; state_pub_->publish(m); }
  void state(const std::string &s) {
    if (s != state_) { state_ = s; RCLCPP_INFO(get_logger(), "%s", s.c_str()); publish_state(); }
  }
  void vehicle_command(uint32_t command, float p1 = 0, float p2 = 0) {
    VehicleCommand m;
    m.timestamp = now().nanoseconds() / 1000; m.command = command; m.param1 = p1; m.param2 = p2;
    m.target_system = 1; m.target_component = 1; m.source_system = 1; m.source_component = 191;
    m.from_external = true; vehicle_pub_->publish(m);
  }
  void tick() {
    if (terminal_) return;
    const double wall = wall_now(), sim = now().seconds();
    if (last_sim_ > sim) { terminal_ = true; state("STOPPED_CLOCK_RESET"); return; }
    last_sim_ = sim;
    if (!enabled_) { state(wall - depth_wall_ < 0.8 ? "READY_TO_START" : "WAIT_SENSORS"); return; }
    const bool sensors = odom_ && local_ && status_ && local_->xy_valid && local_->z_valid &&
      std::isfinite(local_->x) && std::isfinite(local_->y) && std::isfinite(local_->z) && std::isfinite(local_->heading) &&
      wall - odom_wall_ < 0.5 && wall - local_wall_ < 0.5 && wall - status_wall_ < 2.0;
    if (!sensors) { stream_start_ = 0; state("WAIT_SENSORS"); return; }
    if (std::abs(static_cast<double>(local_->timestamp) * 1e-6 - sim) > 1.0) {
      stream_start_ = 0; state("WAIT_PX4_SIM_CLOCK"); return;
    }
    const Vec world = vector(odom_->pose.pose.position);
    const bool armed = status_->arming_state == VehicleStatus::ARMING_STATE_ARMED;
    const bool offboard = status_->nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    if (was_offboard_ && (!armed || !offboard)) { terminal_ = true; state("STOPPED_MODE_LOSS"); return; }
    if (!armed) {
      // External vision publishes absolute Gazebo world NED. Estimator resets
      // correct its estimate in that fixed frame; they must not move the goal.
      const auto &q = odom_->pose.pose.orientation;
      hold_yaw_ = ego_gazebo::yaw(Eigen::Quaterniond(q.w, q.x, q.y, q.z));
      if ((enu_to_ned(world) - Vec(local_->x, local_->y, local_->z)).norm() > 0.3 ||
          std::abs(ego_gazebo::wrap(ego_gazebo::yaw_enu_to_ned(hold_yaw_) - local_->heading)) > 0.2) {
        stream_start_ = 0; state("WAIT_VISION_ALIGNMENT"); return;
      }
      hold_ = Vec(world.x(), world.y(), height_); origin_ = true;
    }
    if (!origin_) { state("WAIT_DISARMED_ORIGIN"); return; }
    const bool depth_ok = wall - depth_wall_ < 0.8;
    if (!was_offboard_ && !depth_ok) { stream_start_ = 0; state("WAIT_DEPTH"); return; }
    if (armed && offboard) {
      was_offboard_ = true;
      if (std::abs(world.z() - height_) < 0.2 && vector(odom_->twist.twist.linear).norm() < 0.3) {
        if (!settled_) settled_ = sim;
      } else if (!ready_) settled_.reset();
      if (settled_ && sim - *settled_ >= 1.0) ready_ = true;
    }
    Vec position = hold_, velocity = Vec::Zero(), acceleration = Vec::Zero();
    double yaw = hold_yaw_, yaw_rate = 0;
    if (ready_ && depth_ok) {
      if (pending_goal_) {
        pending_goal_->header.stamp = now(); pending_goal_->header.frame_id = "world";
        goal_pub_->publish(*pending_goal_); pending_goal_.reset(); cmd_.reset();
        hold_ = world; position = hold_;
      }
      if (cmd_ && wall - cmd_wall_ < 0.5 && (now() - rclcpp::Time(cmd_->header.stamp)).seconds() < 0.5) {
        position = vector(cmd_->position); velocity = vector(cmd_->velocity); acceleration = vector(cmd_->acceleration);
        yaw = cmd_->yaw; yaw_rate = cmd_->yaw_dot;
        hold_ = world; hold_yaw_ = yaw; state("TRACKING");
      } else { state("HOLD"); }
    } else if (ready_) {
      if (state_ != "DEPTH_LOST_HOLD") hold_ = world;
      position = hold_; cmd_.reset(); state("DEPTH_LOST_HOLD");
    } else { state(armed && offboard ? "TAKEOFF" : "PRESTREAM"); }
    TrajectorySetpoint sp;
    sp.timestamp = now().nanoseconds() / 1000;
    sp.position = floats(enu_to_ned(position));
    sp.velocity = floats(enu_to_ned(velocity)); sp.acceleration = floats(enu_to_ned(acceleration));
    sp.jerk.fill(std::numeric_limits<float>::quiet_NaN());
    sp.yaw = ego_gazebo::yaw_enu_to_ned(yaw); sp.yawspeed = -yaw_rate;
    OffboardControlMode mode; mode.timestamp = sp.timestamp; mode.position = true;
    mode_pub_->publish(mode); sp_pub_->publish(sp);
    if (!stream_start_) stream_start_ = wall;
    if (!was_offboard_ && wall - stream_start_ > 2 && wall - last_request_ > 2) {
      last_request_ = wall;
      if (!offboard) vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
      else if (!armed) vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1);
    }
  }
  bool enabled_{false}, terminal_{false}, origin_{false}, was_offboard_{false}, ready_{false};
  double height_, odom_wall_{0}, local_wall_{0}, status_wall_{0}, depth_wall_{0}, cmd_wall_{0};
  double stream_start_{0}, last_request_{0}, last_sim_{0}, hold_yaw_{0};
  Vec hold_{Vec::Zero()};
  std::string state_{"WAIT_SENSORS"};
  std::optional<double> settled_;
  std::optional<nav_msgs::msg::Odometry> odom_;
  std::optional<VehicleLocalPosition> local_;
  std::optional<VehicleStatus> status_;
  std::optional<Command> cmd_;
  std::optional<Goal> pending_goal_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr mode_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr sp_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<Goal>::SharedPtr goal_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::Subscription<Command>::SharedPtr command_sub_;
  rclcpp::Subscription<Goal>::SharedPtr goal_sub_;
  rclcpp::Service<Trigger>::SharedPtr start_srv_, land_srv_;
  rclcpp::TimerBase::SharedPtr timer_, state_timer_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<Offboard>()); rclcpp::shutdown();
}
