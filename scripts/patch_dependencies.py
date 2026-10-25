#!/usr/bin/python3
"""Apply the small, reviewable Jazzy/simulation compatibility patch once."""
from pathlib import Path

root = Path(__file__).resolve().parents[1]
ego = root / 'src/ego-planner-swarm/src/planner'

def replace(path, old, new):
    text = path.read_text()
    if new in text:
        return
    if old in text:
        path.write_text(text.replace(old, new))
    else:
        raise RuntimeError(f'Upstream changed: {path}: {old!r}')

for path in (root / 'src/ego-planner-swarm/src').rglob('*'):
    if path.suffix in ('.h', '.hpp', '.cpp'):
        text = path.read_text()
        if '#include <cv_bridge/cv_bridge.h>' in text:
            path.write_text(text.replace('cv_bridge/cv_bridge.h', 'cv_bridge/cv_bridge.hpp'))

replace(ego / 'plan_manage/include/ego_planner/planner_manager.h',
        '    /* main planning algorithms & modules */',
        '    rclcpp::Node::SharedPtr node_;\n    /* main planning algorithms & modules */')
replace(ego / 'plan_manage/src/planner_manager.cpp',
        '    node->declare_parameter("manager/max_vel", -1.0);',
        '    node_ = node;\n    node->declare_parameter("manager/max_vel", -1.0);')
for relative in ['plan_manage/src/planner_manager.cpp', 'plan_manage/src/ego_replan_fsm.cpp',
                 'traj_utils/src/planning_visualization.cpp']:
    replace(ego / relative, 'rclcpp::Clock().now()', 'node_->now()')
replace(ego / 'plan_manage/src/ego_replan_fsm.cpp',
        'rclcpp::Clock clock(RCL_SYSTEM_TIME);', 'auto &clock = *node_->get_clock();')
replace(ego / 'plan_env/src/grid_map.cpp', 'rclcpp::Time(0, 0, RCL_SYSTEM_TIME)',
        'rclcpp::Time(0, 0, node_->get_clock()->get_clock_type())')
replace(ego / 'plan_manage/src/traj_server.cpp',
        'rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub;',
        'rclcpp::Clock::SharedPtr trajectory_clock;\n'
        'rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub;')
replace(ego / 'plan_manage/src/traj_server.cpp',
        'rclcpp::Clock clock(RCL_ROS_TIME);', 'auto &clock = *trajectory_clock;')
replace(ego / 'plan_manage/src/traj_server.cpp',
        'auto node = rclcpp::Node::make_shared("traj_server");',
        'auto node = rclcpp::Node::make_shared("traj_server");\n  trajectory_clock = node->get_clock();')
# Before a future-dated spline starts, do not publish an all-zero position command.
replace(ego / 'plan_manage/src/traj_server.cpp',
        'cout << "[Traj server]: invalid time." << endl;', 'return; // Wait for the spline start in simulation time.')
# Paused simulation may produce identical timestamps on wall-timer callbacks.
replace(ego / 'plan_manage/src/traj_server.cpp',
        'static rclcpp::Time time_last = clock.now();',
        'static rclcpp::Time time_last = clock.now();\n  if (time_now <= time_last) return;')
# Keep the patch independent of input image resolution (upstream assumes 640x480).
replace(ego / 'plan_env/src/grid_map.cpp',
        '  md_.proj_points_cnt = 0;',
        '  md_.proj_points_cnt = 0;\n  md_.proj_points_.resize(md_.depth_image_.total());')

patch_dir = root / 'patches'
# Clean duplicate guards produced by the previous non-idempotent patcher.
server = ego / 'plan_manage/src/traj_server.cpp'
text = server.read_text()
guard = '  if (time_now <= time_last) return;\n'
while guard + guard in text:
    text = text.replace(guard + guard, guard)
if text != server.read_text():
    server.write_text(text)
grid = ego / 'plan_env/src/grid_map.cpp'
text = grid.read_text()
resize = '  md_.proj_points_.resize(md_.depth_image_.total());\n'
while resize + resize in text:
    text = text.replace(resize + resize, resize)
if text != grid.read_text():
    grid.write_text(text)
for relative, line in [
    ('plan_manage/include/ego_planner/planner_manager.h', '    rclcpp::Node::SharedPtr node_;\n'),
    ('plan_manage/src/planner_manager.cpp', '    node_ = node;\n'),
    ('plan_manage/src/traj_server.cpp', 'rclcpp::Clock::SharedPtr trajectory_clock;\n'),
    ('plan_manage/src/traj_server.cpp', '  trajectory_clock = node->get_clock();\n'),
]:
    path = ego / relative
    original = text = path.read_text()
    while line + line in text:
        text = text.replace(line + line, line)
    if text != original:
        path.write_text(text)
replace(ego / 'plan_manage/src/ego_replan_fsm.cpp',
        'Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, 1.0);',
        'Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);')
replace(ego / 'plan_env/src/grid_map.cpp',
        'mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);',
        'mp_.map_origin_ = Eigen::Vector3d(\n'
        '      node_->declare_parameter("grid_map/origin_x", -x_size / 2.0),\n'
        '      node_->declare_parameter("grid_map/origin_y", -y_size / 2.0), mp_.ground_height_);')
patch_dir.mkdir(exist_ok=True)
import subprocess
with (patch_dir / 'ego-jazzy-sim-time.patch').open('w') as out:
    subprocess.run(['git', '-C', str(ego.parents[1]), 'diff', '--', 'src'], stdout=out, check=True)
