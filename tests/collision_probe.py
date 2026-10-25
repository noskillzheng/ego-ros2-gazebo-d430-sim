"""Validate real DART contact with RMUC mesh, not just its ground plane."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
out = root / 'log/validation'
out.mkdir(parents=True, exist_ok=True)
tree = ET.parse(root / 'assets/rmuc_2027_gazebo/worlds/rmuc_2027.sdf')
world = tree.getroot().find('world')
world.append(ET.fromstring('''<model name="collision_probe">
<pose>14 8 5 0 0 0</pose><link name="body">
<inertial><mass>1</mass><inertia><ixx>0.006667</ixx><iyy>0.006667</iyy><izz>0.006667</izz></inertia></inertial>
<collision name="cube"><geometry><box><size>0.2 0.2 0.2</size></box></geometry></collision>
</link><plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
<odom_publish_frequency>20</odom_publish_frequency><dimensions>3</dimensions><odom_topic>/collision_probe/odom</odom_topic>
</plugin></model>'''))
# This check needs physics only, so no rendering or GUI.
for plugin in list(world.findall('plugin')):
    if plugin.get('name') in ('gz::sim::systems::Sensors', 'gz::sim::systems::SceneBroadcaster'):
        world.remove(plugin)
path = out / 'collision_probe.sdf'
tree.write(path, encoding='unicode')
env = dict(os.environ, GZ_PARTITION=f'ego_collision_{os.getpid()}',
           GZ_SIM_RESOURCE_PATH=str(root / 'assets/rmuc_2027_gazebo/models'))
with (out / 'collision_probe.log').open('w') as log:
    process = subprocess.Popen(['gz', 'sim', '-s', '-r', '--physics-engine',
        'gz-physics-dartsim-plugin', str(path)], env=env, stdout=log,
        stderr=subprocess.STDOUT, start_new_session=True)
    try:
        time.sleep(4)
        sample = subprocess.run(['gz', 'topic', '-e', '-t', '/collision_probe/odom',
            '-n', '1', '--json-output'], env=env, check=True, capture_output=True,
            text=True, timeout=10)
        pose = json.loads(sample.stdout)['pose']['position']
        # STL at (14,8) has its highest surface at z=3.116 m. Cube half-height=0.1.
        assert abs(pose['x'] - 14) < 0.25 and abs(pose['y'] - 8) < 0.25, pose
        assert 3.17 < pose['z'] < 3.27, pose
        print(json.dumps({'collision_passed': True, 'cube_position': pose}))
    finally:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
