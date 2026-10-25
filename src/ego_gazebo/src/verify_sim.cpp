#include "ego_gazebo/frames.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <chrono>
#include <cstring>
#include <iostream>
#include <set>

using namespace std::chrono_literals;
class VerifySim : public rclcpp::Node {
 public:
  VerifySim() : Node("verify_sim"), start_(std::chrono::steady_clock::now()) {
    fly_ = declare_parameter("run_flight", false);
    require_detour_ = declare_parameter("require_detour", false);
    timeout_ = declare_parameter("timeout", 120.0);
    target_ = declare_parameter("target", std::vector<double>{5.0, 2.0, 1.8});
    if (target_.size() != 3 || !ego_gazebo::within_flight_limits({target_[0], target_[1], target_[2]}))
      throw std::invalid_argument("target must contain three coordinates within flight limits");
    depth_ = create_subscription<sensor_msgs::msg::Image>("/camera/depth/image_rect_raw", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr m) {
        if (m->encoding != "32FC1" || m->width != 640 || m->height != 480 || m->is_bigendian || m->step < 2560 || m->data.size() < m->step * m->height) return;
        if (!depth_stamps_.insert(rclcpp::Time(m->header.stamp).nanoseconds()).second) return;
        bool valid = false;
        for (unsigned y = 0; y < m->height; y += 8) for (unsigned x = 0; x < m->width; x += 8) {
          float d; std::memcpy(&d, &m->data[y * m->step + x * 4], 4);
          valid = valid || (std::isfinite(d) && d > 0.1f && d < 10.0f);
        }
        if (valid) ++depth_count_;
      });
    pose_ = create_subscription<geometry_msgs::msg::PoseStamped>("/sim/camera_pose", 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) { pose_stamps_.insert(rclcpp::Time(m->header.stamp).nanoseconds()); });
    odom_ = create_subscription<nav_msgs::msg::Odometry>("/sim/odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
      ++odom_count_; const auto &p = m->pose.pose.position;
      position_ = {p.x, p.y, p.z};
      if (!have_origin_) { origin_ = {p.x, p.y, p.z}; have_origin_ = true; }
      max_distance_ = std::max(max_distance_, (Eigen::Vector3d(p.x, p.y, p.z) - origin_).norm());
      if (goal_sent_) {
        const Eigen::Vector3d line = Eigen::Vector3d(target_[0], target_[1], target_[2]) - route_origin_;
        if (line.norm() > 0.1) {
          const Eigen::Vector3d delta = position_ - route_origin_;
          max_detour_ = std::max(max_detour_, (delta - line * delta.dot(line) / line.squaredNorm()).norm());
        }
        const auto error = (Eigen::Vector3d(p.x, p.y, p.z) - Eigen::Vector3d(target_[0], target_[1], target_[2])).norm();
        if (error < 0.35) ++at_goal_; else at_goal_ = 0;
      }
    });
    info_ = create_subscription<sensor_msgs::msg::CameraInfo>("/camera/depth/camera_info", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
        info_ok_ = m->width == 640 && m->height == 480 && std::abs(m->k[0] - 320.0 / std::tan(1.518436449 / 2)) < 1.0;
      });
    map_ = create_subscription<sensor_msgs::msg::PointCloud2>("/grid_map/occupancy_inflate", 10,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { if (m->width * m->height > 0) ++map_count_; });
    command_ = create_subscription<quadrotor_msgs::msg::PositionCommand>("/position_cmd", 10,
      [this](quadrotor_msgs::msg::PositionCommand::ConstSharedPtr) { if (goal_sent_) ++cmd_count_; });
    using Status = px4_msgs::msg::VehicleStatus;
    status_ = create_subscription<Status>(ego_gazebo::px4_topic<Status>("out", "vehicle_status"), rclcpp::SensorDataQoS(),
      [this](Status::ConstSharedPtr m) { px4_seen_ = true; offboard_ = m->nav_state == Status::NAVIGATION_STATE_OFFBOARD && m->arming_state == Status::ARMING_STATE_ARMED; });
    state_ = create_subscription<std_msgs::msg::String>("/sim/offboard_state", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr m) { controller_state_ = m->data; });
    start_client_ = create_client<std_srvs::srv::Trigger>("/sim/start");
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    timer_ = create_wall_timer(250ms, [this] { tick(); });
  }
  bool done{false};
  int result{1};
 private:
  void tick() {
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();
    std::size_t synced = 0;
    for (auto t : depth_stamps_) if (pose_stamps_.count(t)) ++synced;
    const bool perception = depth_count_ >= 20 && odom_count_ >= 50 && synced >= 20 && info_ok_ && map_count_ > 0 && px4_seen_;
    if (fly_ && perception && !start_sent_ && start_client_->service_is_ready()) {
      start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>()); start_sent_ = true;
    }
    if (fly_ && start_sent_ && !goal_sent_ && offboard_ && controller_state_ == "HOLD") {
      geometry_msgs::msg::PoseStamped goal; goal.header.stamp = now(); goal.header.frame_id = "world";
      goal.pose.position.x = target_[0]; goal.pose.position.y = target_[1]; goal.pose.position.z = target_[2]; goal.pose.orientation.w = 1;
      route_origin_ = position_; goal_pub_->publish(goal); goal_sent_ = true;
    }
    const bool success = perception && (!fly_ || (offboard_ && cmd_count_ >= 20 && at_goal_ >= 50 && max_distance_ > 0.7 &&
      (!require_detour_ || max_detour_ > 0.35)));
    if (success || elapsed > timeout_) {
      result = success ? 0 : 1; done = true;
      std::cout << "{\"success\":" << (success ? "true" : "false") << ",\"depth_frames\":" << depth_count_
        << ",\"synced_frames\":" << synced << ",\"odom\":" << odom_count_ << ",\"camera_info_ok\":" << info_ok_
        << ",\"occupied_maps\":" << map_count_ << ",\"px4_seen\":" << px4_seen_ << ",\"offboard\":" << offboard_
        << ",\"commands\":" << cmd_count_ << ",\"goal_samples\":" << at_goal_ << ",\"max_displacement_m\":" << max_distance_
        << ",\"max_detour_m\":" << max_detour_ << ",\"position\":[" << position_.x() << ',' << position_.y() << ',' << position_.z()
        << "],\"state\":\"" << controller_state_ << "\"}" << std::endl;
    }
  }
  bool fly_, require_detour_, info_ok_{false}, px4_seen_{false}, offboard_{false}, start_sent_{false}, goal_sent_{false}, have_origin_{false};
  double timeout_, max_distance_{0}, max_detour_{0};
  unsigned depth_count_{0}, odom_count_{0}, map_count_{0}, cmd_count_{0}, at_goal_{0};
  Eigen::Vector3d origin_;
  Eigen::Vector3d position_{Eigen::Vector3d::Zero()}, route_origin_{Eigen::Vector3d::Zero()};
  std::vector<double> target_;
  std::string controller_state_;
  std::set<int64_t> depth_stamps_, pose_stamps_;
  std::chrono::steady_clock::time_point start_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr command_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv); auto node = std::make_shared<VerifySim>();
  while (rclcpp::ok() && !node->done) { rclcpp::spin_some(node); std::this_thread::sleep_for(10ms); }
  const int result = node->result; rclcpp::shutdown(); return result;
}
