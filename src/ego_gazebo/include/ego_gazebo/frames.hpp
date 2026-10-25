#pragma once
#include <Eigen/Geometry>
#include <cmath>
#include <string>
#include <type_traits>

namespace ego_gazebo {
constexpr double pi = 3.14159265358979323846;
inline Eigen::Vector3d enu_to_ned(const Eigen::Vector3d &p) { return {p.y(), p.x(), -p.z()}; }
inline double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
inline double yaw_enu_to_ned(double yaw) { return wrap(pi / 2.0 - yaw); }
inline double yaw(const Eigen::Quaterniond &q) {
  const auto r = q.toRotationMatrix();
  return std::atan2(r(1, 0), r(0, 0));
}
inline Eigen::Quaterniond body_from_optical() {
  Eigen::Matrix3d r;
  r << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  return Eigen::Quaterniond(r);
}
inline Eigen::Vector3d camera_offset() { return {0.12, 0.0, 0.02}; }
struct Pose {
  double time;
  Eigen::Vector3d position;
  Eigen::Quaterniond rotation;
};
inline Pose interpolate(const Pose &a, const Pose &b, double t) {
  const double f = (t - a.time) / (b.time - a.time);
  return {t, a.position + f * (b.position - a.position), a.rotation.slerp(f, b.rotation)};
}
inline Pose optical_pose(const Pose &p) {
  return {p.time, p.position + p.rotation * camera_offset(), p.rotation * body_from_optical()};
}
inline bool within_flight_limits(const Eigen::Vector3d &p) {
  return p.allFinite() && p.x() >= 0.5 && p.x() <= 27.65 &&
         p.y() >= 0.5 && p.y() <= 15.57 && p.z() >= 0.6 && p.z() <= 4.2;
}
template<class T, class = void> struct MessageVersion { static constexpr unsigned value = 0; };
template<class T> struct MessageVersion<T, std::void_t<decltype(T::MESSAGE_VERSION)>> {
  static constexpr unsigned value = T::MESSAGE_VERSION;
};
template<class T> std::string px4_topic(const std::string &direction, const std::string &name) {
  const auto version = MessageVersion<T>::value;
  return "/fmu/" + direction + "/" + name + (version ? "_v" + std::to_string(version) : "");
}
}  // namespace ego_gazebo
