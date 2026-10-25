"""RMUC + PX4 SITL + native Gazebo D430 depth + EGO (all data in sim time)."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    share = Path(get_package_share_directory('ego_gazebo'))
    workspace = Path(LaunchConfiguration('workspace').perform(context)).expanduser().resolve()
    px4 = Path(LaunchConfiguration('px4_dir').perform(context)).expanduser().resolve()
    if not (px4 / 'build/px4_sitl_default/bin/px4').is_file():
        raise RuntimeError(f'PX4 SITL executable missing under {px4}')
    if not (workspace / '.runtime/px4/etc/init.d-posix/rcS').is_file():
        raise RuntimeError('Run scripts/prepare_px4.sh first')
    assets = share / 'assets'
    resources = ':'.join([str(assets / 'px4_models'), str(assets / 'rmuc_2027_gazebo/models'), os.environ.get('GZ_SIM_RESOURCE_PATH', '')])
    env = {
        'GZ_SIM_RESOURCE_PATH': resources,
        'GZ_PARTITION': LaunchConfiguration('gz_partition').perform(context),
        'PX4_DIR': str(px4), 'EGO_SIM_ROOT': str(workspace),
    }
    gz_cmd = ['gz', 'sim', '-r', '--physics-engine', 'gz-physics-dartsim-plugin',
              str(assets / 'rmuc_2027_gazebo/worlds/rmuc_2027_x500.sdf')]
    if LaunchConfiguration('headless').perform(context).lower() == 'true':
        gz_cmd += ['-s', '--headless-rendering']
    gazebo = ExecuteProcess(cmd=gz_cmd, additional_env=env, output='screen')
    px4_process = Node(package='ego_gazebo', executable='run_px4.sh', additional_env=env, output='screen')
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
                  additional_env=env, parameters=[{'config_file': str(share / 'config/bridge.yaml'), 'use_sim_time': True}], output='screen')
    sim_params = {'use_sim_time': True}
    planner = Node(package='ego_planner', executable='ego_planner_node', name='ego_planner',
        parameters=[str(share / 'config/planner.yaml')], output='screen',
        remappings=[('odom_world', '/sim/odom'), ('grid_map/odom', '/sim/odom'),
                    ('grid_map/pose', '/sim/camera_pose'), ('grid_map/depth', '/camera/depth/image_rect_raw'),
                    ('planning/broadcast_bspline_from_planner', '/broadcast_bspline'),
                    ('planning/broadcast_bspline_to_planner', '/broadcast_bspline')])
    nodes = [gazebo]
    if LaunchConfiguration('start_agent').perform(context).lower() == 'true':
        nodes.append(ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'], output='screen'))
    nodes += [px4_process, bridge,
        Node(package='ego_gazebo', executable='sim_adapter', parameters=[sim_params], output='screen'),
        planner,
        Node(package='ego_planner', executable='traj_server', parameters=[sim_params, {'traj_server/time_forward': 1.0}], output='screen'),
        Node(package='ego_gazebo', executable='offboard', parameters=[sim_params, {
            'auto_arm': LaunchConfiguration('auto_arm').perform(context).lower() == 'true',
            'takeoff_height': float(LaunchConfiguration('takeoff_height').perform(context))}], output='screen')]
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        nodes.append(Node(package='rviz2', executable='rviz2', arguments=['-d', str(share / 'config/sim.rviz')], parameters=[sim_params]))
    # A dead flight/perception process terminates the launch and its children.
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='Simulation component exited'))])) for node in nodes if node is not None]
    return handlers + nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('workspace', default_value=os.environ.get('EGO_SIM_ROOT', str(Path.cwd()))),
        DeclareLaunchArgument('px4_dir', default_value=os.environ.get('PX4_DIR', str(Path.home() / 'Projects/PX4-Autopilot'))),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('auto_arm', default_value='false'),
        DeclareLaunchArgument('takeoff_height', default_value='1.8'),
        DeclareLaunchArgument('start_agent', default_value='true'),
        DeclareLaunchArgument('gz_partition', default_value='ego_rmuc_2027'),
        OpaqueFunction(function=launch_setup),
    ])
