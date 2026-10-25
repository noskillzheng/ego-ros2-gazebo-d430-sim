#include "ego_gazebo/frames.hpp"
#include <iostream>
#include <stdexcept>
using namespace ego_gazebo;
void check(bool ok, const char *message) { if (!ok) throw std::runtime_error(message); }
struct Unversioned {};
struct Versioned { static constexpr unsigned MESSAGE_VERSION = 4; };
int main() {
  check(enu_to_ned({1, 2, 3}).isApprox(Eigen::Vector3d(2, 1, -3)), "ENU/NED position");
  check(enu_to_ned(enu_to_ned({1, 2, 3})).isApprox(Eigen::Vector3d(1, 2, 3)), "round trip");
  check(std::abs(yaw_enu_to_ned(0) - pi/2) < 1e-10, "east yaw");
  check(std::abs(yaw_enu_to_ned(pi/2)) < 1e-10, "north yaw");
  check((body_from_optical() * Eigen::Vector3d::UnitZ()).isApprox(Eigen::Vector3d::UnitX()), "optical forward");
  check((body_from_optical() * Eigen::Vector3d::UnitX()).isApprox(-Eigen::Vector3d::UnitY()), "optical right");
  Pose a{1, {2, 3, 1}, Eigen::Quaterniond::Identity()};
  Pose b{2, {4, 3, 1}, Eigen::Quaterniond(Eigen::AngleAxisd(pi/2, Eigen::Vector3d::UnitZ()))};
  const auto mid = interpolate(a, b, 1.5);
  check(mid.position.isApprox(Eigen::Vector3d(3, 3, 1)), "position interpolation");
  check(std::abs(yaw(mid.rotation) - pi/4) < 1e-10, "rotation interpolation");
  check(optical_pose(b).position.isApprox(Eigen::Vector3d(4, 3.12, 1.02)), "rotated camera offset");
  check(px4_topic<Unversioned>("in", "test") == "/fmu/in/test", "unversioned PX4 topic");
  check(px4_topic<Versioned>("out", "vehicle_status") == "/fmu/out/vehicle_status_v4", "versioned PX4 topic");
  check(within_flight_limits({27, 15, 4}) && !within_flight_limits({29, 15, 4}), "map limits");
  std::cout << "All frame, optical pose and PX4 topic checks passed\n";
}
