"""Check cross-file contracts that otherwise silently corrupt depth fusion."""
import ast
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

root = Path(__file__).resolve().parents[1]
package = root / 'src/ego_gazebo'
model = ET.parse(root / 'assets/px4_models/x500_d430/model.sdf')
camera = model.find('.//sensor/camera')
params = yaml.safe_load((package / 'config/planner.yaml').read_text())['ego_planner']['ros__parameters']
width = int(camera.findtext('image/width'))
height = int(camera.findtext('image/height'))
focal = width / (2 * math.tan(float(camera.findtext('horizontal_fov')) / 2))
assert abs(params['grid_map/fx'] - focal) < 0.02
assert params['grid_map/fx'] == params['grid_map/fy']
assert params['grid_map/cx'] == width / 2 and params['grid_map/cy'] == height / 2
assert model.findtext('.//link[@name="d430_link"]/pose') == '0.12 0 0.02 0 0 0'
assert params['grid_map/origin_x'] + params['grid_map/map_size_x'] > 28.15
assert params['grid_map/origin_y'] + params['grid_map/map_size_y'] > 16.07
assert params['grid_map/obstacles_inflation'] >= 0.35
bridge = yaml.safe_load((package / 'config/bridge.yaml').read_text())
assert all(entry['direction'] == 'GZ_TO_ROS' for entry in bridge)
assert next(entry for entry in bridge if entry['ros_topic_name'] == '/gz/depth')['gz_topic_name'] == model.findtext('.//sensor/topic')
for source in (package / 'launch').glob('*.py'):
    ast.parse(source.read_text(), filename=str(source))
for source in (root / 'assets').rglob('*.sdf'):
    ET.parse(source)
print('Camera intrinsics, map extents, bridge directions, XML and launch syntax passed')
