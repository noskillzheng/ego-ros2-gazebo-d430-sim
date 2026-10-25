#include "ego_gazebo/frames.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <deque>

using ego_gazebo::Pose;
class SimAdapter : public rclcpp::Node {
 public:
  SimAdapter() : Node("sim_adapter"), tf_(*this), static_tf_(*this) {
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/sim/odom", 10);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/sim/camera_pose", 10);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>("/camera/depth/image_rect_raw", 10);
    vision_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
      ego_gazebo::px4_topic<px4_msgs::msg::VehicleOdometry>("in", "vehicle_visual_odometry"), 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/gz/odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { odometry(*msg); });
    image_sub_ = create_subscription<sensor_msgs::msg::Image>("/gz/depth", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        if (msg->encoding != "32FC1" || !msg->width || !msg->height ||
            msg->step < msg->width * sizeof(float) || msg->data.size() < msg->step * msg->height) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Invalid metric depth image");
          return;
        }
        images_.push_back(msg);
        if (images_.size() > 8) images_.pop_front();
        flush();
      });
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = "base_link";
    t.child_frame_id = "camera_depth_optical_frame";
    const auto p = ego_gazebo::camera_offset();
    t.transform.translation.x = p.x(); t.transform.translation.y = p.y(); t.transform.translation.z = p.z();
    t.transform.rotation = quaternion(ego_gazebo::body_from_optical());
    static_tf_.sendTransform(t);
  }
 private:
  static double seconds(const builtin_interfaces::msg::Time &s) { return s.sec + 1e-9 * s.nanosec; }
  static geometry_msgs::msg::Quaternion quaternion(const Eigen::Quaterniond &q) {
    geometry_msgs::msg::Quaternion out;
    out.x = q.x(); out.y = q.y(); out.z = q.z(); out.w = q.w(); return out;
  }
  void odometry(nav_msgs::msg::Odometry msg) {
    const auto &p = msg.pose.pose.position; const auto &q = msg.pose.pose.orientation;
    Pose pose{seconds(msg.header.stamp), {p.x, p.y, p.z}, {q.w, q.x, q.y, q.z}};
    if (!pose.position.allFinite() || !pose.rotation.coeffs().allFinite() || pose.rotation.norm() < 1e-6) return;
    pose.rotation.normalize();
    if (!history_.empty() && pose.time < history_.back().time) { history_.clear(); images_.clear(); }
    if (!history_.empty() && pose.time == history_.back().time) return;
    history_.push_back(pose);
    if (history_.size() > 200) history_.pop_front();
    msg.header.frame_id = "world"; msg.child_frame_id = "base_link";
    msg.pose.pose.orientation = quaternion(pose.rotation);
    // Gazebo reports body-frame twist. EGO's odometryCallback expects world velocity.
    // /sim/odom is a planner interface; TF still describes the actual body frame.
    auto &v = msg.twist.twist.linear;
    const Eigen::Vector3d velocity = pose.rotation * Eigen::Vector3d(v.x, v.y, v.z);
    v.x = velocity.x(); v.y = velocity.y(); v.z = velocity.z();
    px4_msgs::msg::VehicleOdometry vision;
    vision.timestamp = now().nanoseconds() / 1000;
    vision.timestamp_sample = rclcpp::Time(msg.header.stamp).nanoseconds() / 1000;
    vision.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    vision.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
    const auto ned = ego_gazebo::enu_to_ned(pose.position);
    const auto ned_velocity = ego_gazebo::enu_to_ned(velocity);
    Eigen::Matrix3d ned_from_enu;
    ned_from_enu << 0, 1, 0, 1, 0, 0, 0, 0, -1;
    const Eigen::Quaterniond attitude = Eigen::Quaterniond(ned_from_enu) * pose.rotation *
      Eigen::Quaterniond(Eigen::AngleAxisd(ego_gazebo::pi, Eigen::Vector3d::UnitX()));
    vision.position = {float(ned.x()), float(ned.y()), float(ned.z())};
    vision.velocity = {float(ned_velocity.x()), float(ned_velocity.y()), float(ned_velocity.z())};
    vision.q = {float(attitude.w()), float(attitude.x()), float(attitude.y()), float(attitude.z())};
    vision.angular_velocity.fill(std::numeric_limits<float>::quiet_NaN());
    vision.position_variance.fill(0.0001f);
    vision.velocity_variance.fill(0.0001f);
    vision.orientation_variance.fill(0.0001f);
    vision.quality = 100;
    vision_pub_->publish(vision);
    odom_pub_->publish(msg);
    geometry_msgs::msg::TransformStamped t;
    t.header = msg.header; t.child_frame_id = "base_link";
    t.transform.translation.x = p.x; t.transform.translation.y = p.y; t.transform.translation.z = p.z;
    t.transform.rotation = msg.pose.pose.orientation;
    tf_.sendTransform(t);
    flush();
  }
  void flush() {
    while (!images_.empty() && history_.size() >= 2) {
      const auto image = images_.front(); const double t = seconds(image->header.stamp);
      if (t > history_.back().time) return;
      images_.pop_front();
      for (std::size_t i = 1; i < history_.size(); ++i) {
        const auto &a = history_[i-1]; const auto &b = history_[i];
        if (a.time <= t && t <= b.time && b.time - a.time < 0.1) {
          const auto pose = ego_gazebo::optical_pose(ego_gazebo::interpolate(a, b, t));
          geometry_msgs::msg::PoseStamped out;
          out.header = image->header; out.header.frame_id = "world";
          out.pose.position.x = pose.position.x(); out.pose.position.y = pose.position.y(); out.pose.position.z = pose.position.z();
          out.pose.orientation = quaternion(pose.rotation);
          auto depth = *image; depth.header.frame_id = "camera_depth_optical_frame";
          // Both messages retain the sensor acquisition timestamp, including while the body rotates.
          pose_pub_->publish(out); depth_pub_->publish(depth);
          break;
        }
      }
    }
  }
  std::deque<Pose> history_;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> images_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr vision_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  tf2_ros::TransformBroadcaster tf_;
  tf2_ros::StaticTransformBroadcaster static_tf_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<SimAdapter>()); rclcpp::shutdown();
}
