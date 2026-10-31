#!/usr/bin/env python3
"""Build the "venue lighting" variant of the RMUC 2027 world.

Lighting and surface appearance follow the real venue, measured from auto-aim recordings
of the RMUC 2026 finals (Shenzhen, 2026-08-05) taken with a Hikrobot MV-CS016-10UC at a
fixed 4000 us / 8 dB exposure and no gamma. The camera is only a measuring instrument: its
exposure, clipping and colour balance are not carried into the scene. What is used are
quantities a camera measures independently of its exposure: where the light comes from,
how much of it is ambient, reflectance ratios between surfaces, and LED hues. The absolute
light level is set for a normally exposed camera (see E_WHITE). See docs/venue_lighting.md.

    python3 tools/build_venue_lighting.py

Only needs numpy. Reads the existing visual GLBs and writes (relative to ..):

  models/rmuc_2027_lit/                     field: same geometry; measured floor, wall and
                                            paint reflectance; light bars emit
  models/rmuc_2027_outpost_rotor_{red,blue}_lit/
  worlds/rmuc_2027_venue.sdf                overhead downlight grid, no sun

The lit GLBs differ from the originals only in material factors (glTF JSON), so they are
regenerated in about a second and are git-ignored.
"""
import json
import math
import re
import struct
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
MODELS = ROOT / 'models'

# --- Measurements ------------------------------------------------------------------------
# All measured values come from video/measurements.json, written by video/measure_video.py
# from the auto-aim recordings (8-bit linear DN at 4000 us / 8 dB; only ratios are used).
MEASURED = json.loads((ROOT / 'video/measurements.json').read_text())
_M, _D = MEASURED['measurements'], MEASURED['derived']
WHITE_DN_G = _M['white_marking_dn'][1]          # white floor marking, horizontal, green
BANNER_DN = tuple(_M['banner_dn'])              # LED ring banner around the stands, median
# Shadowed / lit floor is 0.05-0.14 in the recordings. gz has no indirect light; the
# ambient term stands in for the bounce off the lit floor that fills the shadows.
AMBIENT_FRACTION = 0.08

# --- Surface reflectance -----------------------------------------------------------------
# Same light, same frame -> ratios of DN are ratios of albedo. One absolute anchor is
# assumed: white floor paint reflects 80%.
WHITE_PAINT_ALBEDO = 0.8
FLOOR_ALBEDO = round(_D['floor_over_white'] * WHITE_PAINT_ALBEDO, 3)
BLUE_PAINT_ALBEDO = [round(v * FLOOR_ALBEDO, 3) for v in _D['blue_paint_over_floor_median']]
# Red paint is not in view in any clip. Same paint family assumed: blue's channels mirrored.
# The CAD value (1, 0.002, 0.002) would reflect more red than the white paint does.
RED_PAINT_ALBEDO = BLUE_PAINT_ALBEDO[::-1]
# Walls, highland sides and the central structure are near black: wall/floor in the same
# frame is _D['wall_over_floor_range']. This albedo reproduces that ratio in rendered
# ground-level views (it is higher than the plain ratio because gz walls get no light
# bounced off the floor). CAD hue kept.
WALL_ALBEDO = 0.12
# LED hue from unclipped pixels at the edge of clipped light bars (max channel = 1). Cores
# clip, so only the hue is measured; brightness is at least _D['led_over_white_lower_bound']
# times the white marking.
LED_COLOUR = {k: tuple(v['rgb']) for k, v in _M['led_edge_chromaticity_median'].items()}

# CAD colours (linear) these rules apply to.
CAD_FLOOR_GREYS = (0.033, 0.053)
CAD_WALL_BEIGE = (0.578, 0.578, 0.456)
# Team blue that build_from_glb.py swaps in for CAD red 1.0/0.0271 (sRGB 0.066/0.004/0.593).
CAD_TEAM_DARK_BLUE = (0.006, 0.0, 0.31)

# --- Absolute level ----------------------------------------------------------------------
# A gz camera has no exposure control: pixel = albedo x light. The recordings fix ratios,
# not the absolute level, so the level is chosen like a camera auto-exposure would: a white
# (albedo 1) horizontal surface under the mean floor light renders at linear 0.8 (sRGB 231),
# and stays below clipping under the brightest light (max/mean 1.24). Scale this to make the
# whole scene brighter or darker; all ratios stay the same.
E_WHITE = 0.8
LIGHT_COLOUR = (1.0, 1.0, 1.0)        # neutral: the auto-aim camera's colour balance is unknown

# --- Lighting model ------------------------------------------------------------------------
FIELD_X, FIELD_Y = 28.15, 16.07
LIGHT_HEIGHT = 16.0              # m; venue truss height is not measurable from the clips
# Downlights along x, y. 4 x 3 gives the same floor level and uniformity as 6 x 4; with
# 24 shadow-casting spots ogre2 shows blocky shadow-map patches on the floor.
LIGHT_GRID = (4, 3)
# gz-sim 8 / ogre2, measured with a calibration world: a spot light of intensity I at
# distance d (quadratic attenuation 1) gives linear radiance 0.665 * albedo * I / d^2
# for a surface facing it. Ambient a gives 0.32 * a * albedo.
GZ_SPOT_GAIN = 0.665
GZ_AMBIENT_GAIN = 0.32


def light_positions():
    nx, ny = LIGHT_GRID
    xs = np.linspace(-1.0, FIELD_X + 1.0, nx + 2)[1:-1]
    ys = np.linspace(-1.0, FIELD_Y + 1.0, ny + 2)[1:-1]
    return np.array([(x, y, LIGHT_HEIGHT) for x in xs for y in ys])


def light_intensity(lights):
    """Per-light intensity so the mean floor irradiance equals E_WHITE."""
    pts = np.array([(x, y, 0.3) for x in np.linspace(1, FIELD_X - 1, 27)
                    for y in np.linspace(1, FIELD_Y - 1, 15)])
    e = []
    for p in pts:
        d = lights - p
        r = np.linalg.norm(d, axis=1)
        e.append((d[:, 2] / r / r**2).sum())
    e = np.array(e)
    return E_WHITE / (GZ_SPOT_GAIN * e.mean()), e.min() / e.mean(), e.max() / e.mean()


# --- GLB editing -------------------------------------------------------------------------
def read_glb(path):
    data = path.read_bytes()
    n = struct.unpack('<I', data[12:16])[0]
    return json.loads(data[20:20 + n]), data[20 + n:]


def write_glb(path, gltf, rest):
    js = json.dumps(gltf, separators=(',', ':')).encode()
    js += b' ' * (-len(js) % 4)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(struct.pack('<III', 0x46546C67, 2, 20 + len(js) + len(rest))
                     + struct.pack('<II', len(js), 0x4E4F534A) + js + rest)


def accessor(gltf, binary, index):
    a = gltf['accessors'][index]
    bv = gltf['bufferViews'][a['bufferView']]
    dtype = {5126: np.float32, 5125: np.uint32, 5123: np.uint16, 5121: np.uint8}[a['componentType']]
    comps = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4}[a['type']]
    offset = 8 + bv.get('byteOffset', 0) + a.get('byteOffset', 0)   # skip BIN chunk header
    x = np.frombuffer(binary, dtype, a['count'] * comps, offset)
    return x.reshape(-1, comps) if comps > 1 else x


def _close(c, ref, tol=2e-3):
    return all(abs(a - b) < tol for a, b in zip(c, ref))


def _hue_class(c):
    r, g, b = c
    if g > 0.6 and b > 0.6 and r < 0.4:
        return 'cyan'
    if b >= max(r, g):
        return 'blue'
    if r >= max(g, b):
        return 'red'
    return 'green'


def apply_measured_appearance(gltf, binary):
    """Set the surfaces that the recordings measure; everything else keeps its CAD colour.

    Per primitive (each has its own material):
      CAD floor greys, mostly up-facing, top below 1.1 m -> FLOOR_ALBEDO
      CAD floor greys elsewhere (central structure, tall walls)
      and CAD beige (low walls, highland sides)          -> WALL_ALBEDO, CAD hue
      white (paint)                                      -> WHITE_PAINT_ALBEDO
      saturated, small (< 1 m^2) and mostly upright      -> light bar: emits LED_COLOUR
      saturated blue, flat or large                      -> BLUE_PAINT_ALBEDO
      pure red, flat or large                            -> RED_PAINT_ALBEDO (by analogy)
    Light bars emit at full strength: they clip in the recordings while the white marking
    reads 50/255, so they are at least 5x brighter than white paint under the venue light,
    more than an 8-bit gz camera can show. Cyan LEDs are not visible in the recordings and
    keep the CAD hue; other colours not listed keep their CAD colour.
    """
    edits = []
    for mesh in gltf['meshes']:
        for prim in mesh['primitives']:
            mat = gltf['materials'][prim['material']]
            pbr = mat.setdefault('pbrMetallicRoughness', {})
            if 'baseColorTexture' in pbr:
                continue
            c = pbr.get('baseColorFactor', [1, 1, 1, 1])[:3]
            pos = accessor(gltf, binary, prim['attributes']['POSITION']).astype(float)
            tri = accessor(gltf, binary, prim['indices']).reshape(-1, 3)
            v0, v1, v2 = pos[tri[:, 0]], pos[tri[:, 1]], pos[tri[:, 2]]
            n = np.cross(v1 - v0, v2 - v0)
            area = np.linalg.norm(n, axis=1) / 2
            nz = n[:, 2] / (2 * area + 1e-12)
            total = max(area.sum(), 1e-12)
            up = area[nz > 0.9].sum() / total
            flat = area[np.abs(nz) > 0.9].sum() / total
            grey = max(c) - min(c) < 1e-3
            new = None
            if grey and any(abs(c[0] - v) < 2e-3 for v in CAD_FLOOR_GREYS):
                if up >= 0.4 and pos[:, 2].max() < 1.1:
                    new, kind = [FLOOR_ALBEDO] * 3, 'floor'
                else:
                    new, kind = [WALL_ALBEDO] * 3, 'structure'
            elif _close(c, CAD_WALL_BEIGE):
                new, kind = [round(WALL_ALBEDO * v / max(c), 4) for v in c], 'wall'
            elif grey and c[0] > 0.999:
                new, kind = [WHITE_PAINT_ALBEDO] * 3, 'white paint'
            elif max(c) - min(c) >= 0.5 or _close(c, CAD_TEAM_DARK_BLUE):
                cls = _hue_class(c)
                if flat <= 0.5 and total <= 1.0:
                    mat['emissiveFactor'] = list(LED_COLOUR.get(cls, [round(v / max(c), 4) for v in c]))
                    edits.append((mat['name'], f'LED {cls}'))
                    continue
                if cls == 'blue':
                    new, kind = BLUE_PAINT_ALBEDO, 'blue paint'
                elif cls == 'red' and c[1] < 0.05 and c[2] < 0.05:
                    new, kind = RED_PAINT_ALBEDO, 'red paint (by analogy)'
            if new is not None:
                pbr['baseColorFactor'] = list(new) + [pbr.get('baseColorFactor', [1, 1, 1, 1])[3]]
                edits.append((mat['name'], kind))
    return edits


# --- SDF ---------------------------------------------------------------------------------
def fmt(v):
    return ' '.join(f'{x:.4g}' for x in v)


def model_config(name, description):
    return f'''<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.10">model.sdf</sdf>
  <author><name>RMUC 2027 field conversion</name><email>none</email></author>
  <description>{description}</description>
</model>
'''


def write_field():
    src = MODELS / 'rmuc_2027'
    gltf, rest = read_glb(src / 'meshes/rmuc_2027_visual.glb')
    edits = apply_measured_appearance(gltf, rest)
    dst = MODELS / 'rmuc_2027_lit'
    write_glb(dst / 'meshes/rmuc_2027_visual_lit.glb', gltf, rest)
    sdf = (src / 'model.sdf').read_text()
    sdf = sdf.replace('<model name="rmuc_2027">', '<model name="rmuc_2027_lit">')
    sdf = sdf.replace('model://rmuc_2027/meshes/rmuc_2027_visual.glb',
                      'model://rmuc_2027_lit/meshes/rmuc_2027_visual_lit.glb')
    sdf = sdf.replace('<!-- rm-map-tools', '<!-- Venue-lighting copy of rmuc_2027 (tools/build_venue_lighting.py):\n'
                      '           measured floor/wall/paint reflectance, emissive light bars.\n'
                      '           rm-map-tools')
    (dst / 'model.sdf').write_text(sdf)
    (dst / 'model.config').write_text(model_config(
        'rmuc_2027_lit', 'rmuc_2027 with reflectance and LED colours measured at the RMUC 2026 venue. '
        'Collision mesh is shared with rmuc_2027.'))
    return edits


def write_rotor(team):
    src = MODELS / f'rmuc_2027_outpost_rotor_{team}'
    gltf, rest = read_glb(src / 'meshes/rotor_visual.glb')
    edits = apply_measured_appearance(gltf, rest)
    name = f'rmuc_2027_outpost_rotor_{team}_lit'
    dst = MODELS / name
    write_glb(dst / 'meshes/rotor_visual.glb', gltf, rest)
    sdf = (src / 'model.sdf').read_text().replace(f'rmuc_2027_outpost_rotor_{team}', name)
    (dst / 'model.sdf').write_text(sdf)
    (dst / 'model.config').write_text(model_config(name, f'{team} outpost rotor with emissive light bars.'))
    return edits


def banner_visuals():
    """LED ring banner around the stands: a dim emissive band, approximate geometry.

    Its brightness relative to the lit white floor marking is taken from the recordings.
    """
    r, g, b = (E_WHITE * v / WHITE_DN_G for v in BANNER_DN)
    margin, z, height = 6.0, 7.0, 0.8
    cx, cy = FIELD_X / 2, FIELD_Y / 2
    hx, hy = FIELD_X / 2 + margin, FIELD_Y / 2 + margin
    sides = [((cx, cy - hy), 2 * hx, 0.0), ((cx, cy + hy), 2 * hx, 0.0),
             ((cx - hx, cy), 2 * hy, math.pi / 2), ((cx + hx, cy), 2 * hy, math.pi / 2)]
    out = []
    for i, ((x, y), length, yaw) in enumerate(sides):
        out.append(f'''        <visual name="banner_{i}">
          <pose>{x:.3f} {y:.3f} {z} 0 0 {yaw:.6f}</pose>
          <geometry><box><size>{length:.3f} 0.05 {height}</size></box></geometry>
          <cast_shadows>false</cast_shadows>
          <material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse><specular>0 0 0 1</specular>
            <emissive>{r:.3f} {g:.3f} {b:.3f} 1</emissive></material>
        </visual>''')
    return '\n'.join(out)


def write_world(lights, intensity, umin, umax):
    rmuc = (ROOT / 'worlds/rmuc_2027.sdf').read_text()
    rotors = re.findall(r'(<include>\s*<uri>model://rmuc_2027_outpost_rotor_(red|blue)</uri>.*?</include>)', rmuc, re.S)
    rotor_xml = '\n'.join('    ' + x.replace(f'rmuc_2027_outpost_rotor_{t}<', f'rmuc_2027_outpost_rotor_{t}_lit<')
                          for x, t in rotors)
    ground = re.search(r'    <model name="ground_plane">.*?</model>\n', rmuc, re.S).group(0)
    ambient = AMBIENT_FRACTION * E_WHITE / GZ_AMBIENT_GAIN
    lc = fmt(LIGHT_COLOUR)
    light_xml, disc_xml = [], []
    for i, (x, y, z) in enumerate(lights):
        light_xml.append(f'''    <light type="spot" name="downlight_{i:02d}">
      <pose>{x:.3f} {y:.3f} {z:.2f} 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <visualize>false</visualize>
      <intensity>{intensity:.3f}</intensity>
      <diffuse>{lc} 1</diffuse>
      <specular>{fmt(np.array(LIGHT_COLOUR) * 0.3)} 1</specular>
      <direction>0 0 -1</direction>
      <attenuation><range>40</range><constant>0</constant><linear>0</linear><quadratic>1</quadratic></attenuation>
      <spot><inner_angle>1.1</inner_angle><outer_angle>1.4</outer_angle><falloff>1</falloff></spot>
    </light>''')
        # What a camera sees when it looks up: a small saturated disc.
        disc_xml.append(f'''        <visual name="downlight_{i:02d}">
          <pose>{x:.3f} {y:.3f} {z + 0.05:.2f} 0 0 0</pose>
          <geometry><cylinder><radius>0.25</radius><length>0.05</length></cylinder></geometry>
          <cast_shadows>false</cast_shadows>
          <material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse><specular>0 0 0 1</specular>
            <emissive>1 1 1 1</emissive></material>
        </visual>''')
    world = f'''<?xml version="1.0"?>
<!-- Generated by tools/build_venue_lighting.py. Do not edit by hand; change the script.
     RMUC 2027 field under lighting fitted to RMUC 2026 finals venue recordings: overhead
     downlights, little ambient light, dark arena; floor, wall and paint reflectance and
     LED colours measured from the same recordings.
     A white horizontal surface renders at linear {E_WHITE} on average (floor uniformity
     min/mean {umin:.2f}, max/mean {umax:.2f}). See docs/venue_lighting.md. -->
<sdf version="1.10">
  <world name="rmuc_2027_venue">
    <gravity>0 0 -9.81</gravity>

    <physics name="1ms" type="bullet">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <!-- Ambient {AMBIENT_FRACTION:.0%} of the direct light; stands and ceiling are black. -->
    <scene>
      <ambient>{ambient:.4f} {ambient:.4f} {ambient:.4f} 1</ambient>
      <background>0 0 0 1</background>
      <shadows>true</shadows>
      <grid>false</grid>
    </scene>

    <!-- {len(lights)} downlights at {LIGHT_HEIGHT} m, {LIGHT_GRID[0]} x {LIGHT_GRID[1]} grid over the field. -->
{chr(10).join(light_xml)}

    <model name="venue_fixtures">
      <static>true</static>
      <link name="link">
{chr(10).join(disc_xml)}
{banner_visuals()}
      </link>
    </model>

{ground}
    <include>
      <uri>model://rmuc_2027_lit</uri>
      <name>rmuc_2027</name>
      <pose>0 0 0 0 0 0</pose>
    </include>
{rotor_xml}
  </world>
</sdf>
'''
    (ROOT / 'worlds/rmuc_2027_venue.sdf').write_text(world)
    return ambient


def main():
    lights = light_positions()
    intensity, umin, umax = light_intensity(lights)
    edits = write_field()
    rotor_edits = {t: write_rotor(t) for t in ('red', 'blue')}
    ambient = write_world(lights, intensity, umin, umax)
    print(f'{len(lights)} lights x intensity {intensity:.3f}, ambient {ambient:.4f}, '
          f'white level {E_WHITE}, floor uniformity min/mean {umin:.2f} max/mean {umax:.2f}')
    from collections import Counter
    print('field materials:', dict(Counter(k for _, k in edits)))
    for t, e in rotor_edits.items():
        print(f'rotor {t}:', dict(Counter(k for _, k in e)))


if __name__ == '__main__':
    main()
