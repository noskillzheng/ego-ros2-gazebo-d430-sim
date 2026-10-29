#!/usr/bin/env python3
"""Build the Gazebo field visual and spinning outpost rotors from rm-map-tools' GLB.

Input: field.glb from the rm-map-tools release rmuc2027-simplified-glb-20261018
(SHA-256 ef40412b...). Run from anywhere:

    python build_from_glb.py /path/to/field.glb [--visual-only]

Requires numpy, scipy, trimesh, pygltflib, rtree, networkx, shapely, mapbox_earcut,
pillow and fast_simplification.

Outputs (relative to ../models):
  rmuc_2027/meshes/rmuc_2027_visual.glb            static field visual, outpost rotors removed,
                                                   team colours fixed, base/outpost stickers
  rmuc_2027/meshes/rmuc_2027_collision.stl        static field collision: the visual geometry
                                                   (rotors removed, no stickers), decimated per part
  rmuc_2027_outpost_rotor_{red,blue}/              rotor visual (armor modules + carrier + stickers),
                                                   revolute joint + velocity JointController
  ../tools/build_report.json                       placements, axis fit, colour and sticker checks

--visual-only skips the collision mesh (it does not depend on colours or stickers).
The GLB is the only geometry input.
"""
import argparse
import hashlib
import json
import io
import struct
from pathlib import Path

import numpy as np
import trimesh
from pygltflib import GLTF2
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from trimesh.graph import connected_component_labels
from PIL import Image
from scipy import ndimage
import shapely.geometry

GLB_SHA256 = 'ef40412bae1324475aab47e908e150b7b21d66f233740561232df5adf7919882'
# GLB node "rmuc-2027" holds CAD coordinates (Z-up, metres, centred). This translation
# puts the field corner at the origin. It was fitted against the former STL release of the
# field (median point-to-surface error 0.0 mm, p90 below 0.5 mm) and is kept so that world
# poses stay valid.
FIELD_OFFSET = np.array([14.149088636040688, 6.441009044647217, 1.9607571363449097])
# Rulebook 5.6.1 (rm-map-tools rules/source-motion-rmuc2027.json): 0.8*pi rad/s.
ROTOR_SPEED = 0.8 * np.pi
# Static field collision, built per GLB mesh node. Each part is decimated as far as it can go
# while 98% of points sampled on the original surface stay within COLLISION_TOLERANCE (and all
# within 4x that). Decimating the whole field at once does not work: a single coarse part such
# as a road slab loses its top face before any detail elsewhere is removed.
COLLISION_TOLERANCE = 0.03       # metres
COLLISION_RATIOS = (0.01, 0.02, 0.04, 0.08, 0.16, 0.32)  # face fractions tried, smallest first
COLLISION_MIN_FACES = 12         # parts this small are kept as they are
COLLISION_MIN_SIZE = 0.05        # bounding-box diagonal (m) below which a part is dropped
COLLISION_HULL_SIZE = 0.3        # parts up to this size that cannot be decimated use their hull
SOURCE_AXIS_ORIGIN_MM = np.array([-9.437883451525, 1137.4399976, -1.0455166794])

# Field centre in the field frame; the arena is point-symmetric about it and red is +x.
CENTRE_XY = np.array([14.149088636040688, 8.065])

# Team colours, by GLB material name. Shared CAD parts (both outposts, bases, resource
# zones, fortresses, hangar-platform pads, dart stations) use one mesh with one colour set,
# so the blue copies show red lights and vice versa. Pairs below follow the terrain, which
# is already coloured per team (e.g. red-road 1,0.0271,0.0271 <-> blue-road 0.066,0.004,0.593).
RED_TO_BLUE = {'colour_1.0000,0.0271,0.0271': 'colour_0.0660,0.0040,0.5930',
               'colour_1.0000,0.0000,0.0000': 'colour_0.0000,0.0000,1.0000',
               'colour_0.4400,0.1400,0.1400': 'colour_0.1400,0.1400,0.4400'}
BLUE_TO_RED = {'colour_0.0660,0.0040,0.5930': 'colour_1.0000,0.0271,0.0271',
               'colour_0.0000,0.0000,1.0000': 'colour_1.0000,0.0000,0.0000',
               'colour_0.1333,0.2902,0.7451': 'colour_1.0000,0.0271,0.0271'}
# Rule 5.4.3 / table 5-13: each team has its own technology core, and light groups 2 and 3
# (the side panels) show the team colour. The core at x > centre faces the red assembly area.
# The outpost's lights are mostly 1,0.0271,0.0271. Mapped to the terrain's dark navy they
# are hard to see; the CAD itself uses 0,0,1 for the outpost's blue lights, so use that.
LIGHT_RED_TO_BLUE = {'colour_1.0000,0.0271,0.0271': 'colour_0.0000,0.0000,1.0000'}
LIGHT_OWNERS = ('red-outpost-with-footing', 'blue-outpost-with-footing')
TEAM_OWNED_BY_POSITION = ('fortress-structure', 'drone-landing-pad', 'technology-core')

# Dart detection modules: the CAD gives the light strips on both sides of the dart armor
# plate an unlit colour (grey on the base, white diffuser on the outpost). Rulebook figure
# 4-13 shows them lit in the team colour. Strip box in the plate frame (metres): lateral
# |x| 0.054-0.072, vertical |y| < 0.04, 0-20 mm below the plate face.
DART_STRIP_MATERIALS = ('colour_0.6275,0.6275,0.6275', 'colour_1.0000,1.0000,0.9490')
DART_STRIP_BOX = (0.054, 0.072, 0.04, -0.020, 0.0)
TEAM_LIGHT = {'red': 'colour_1.0000,0.0000,0.0000', 'blue': 'colour_0.0000,0.0000,1.0000'}

# Official stickers, RoboMaster 2027 robot specification manual V1.0.0, appendix 2
# (printed pp. 88-89). Replace the PNGs to change the artwork.
STICKER_DIR = Path(__file__).resolve().parent / 'stickers'
STICKER_OFFSET = 0.0008  # metres in front of the plate, against z-fighting

HERE = Path(__file__).resolve().parent
MODELS = HERE.parent / 'models'
CT = {5121: np.uint8, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}
NC = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4}


class Source:
    def __init__(self, path):
        self.gltf = GLTF2().load(str(path))
        self.blob = self.gltf.binary_blob()
        self.nodes = self.gltf.nodes

    def accessor(self, index):
        a = self.gltf.accessors[index]
        view = self.gltf.bufferViews[a.bufferView]
        n, dt = NC[a.type], np.dtype(CT[a.componentType])
        assert not view.byteStride or view.byteStride == n * dt.itemsize, 'interleaved data'
        offset = (view.byteOffset or 0) + (a.byteOffset or 0)
        return np.frombuffer(self.blob, dt, a.count * n, offset).reshape(a.count, n)

    def local(self, i):
        node = self.nodes[i]
        if node.matrix:
            return np.array(node.matrix, float).reshape(4, 4).T
        m = np.eye(4)
        if node.scale:
            m = np.diag(list(node.scale) + [1.0])
        if node.rotation:
            r = np.eye(4)
            r[:3, :3] = Rotation.from_quat(node.rotation).as_matrix()
            m = r @ m
        if node.translation:
            m[:3, 3] += node.translation
        return m

    def walk(self, i, parent=np.eye(4)):
        world = parent @ self.local(i)
        yield i, world
        for child in self.nodes[i].children:
            yield from self.walk(child, world)

    def find(self, parent, name):
        hits = [c for c in self.nodes[parent].children if self.nodes[c].name == name]
        assert len(hits) == 1, (parent, name, hits)
        return hits[0]

    def primitives(self, i, world):
        """Yield (node, prim_index, material, positions, normals, uv, triangles) in world frame."""
        for j, w in self.walk(i, world):
            mesh = self.nodes[j].mesh
            if mesh is None:
                continue
            for k, p in enumerate(self.gltf.meshes[mesh].primitives):
                assert p.mode in (None, 4), 'only triangle lists are supported'
                pos = self.accessor(p.attributes.POSITION).astype(float) @ w[:3, :3].T + w[:3, 3]
                nrm = self.accessor(p.attributes.NORMAL).astype(float) @ w[:3, :3].T
                nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12)
                uv = (self.accessor(p.attributes.TEXCOORD_0).astype(float)
                      if p.attributes.TEXCOORD_0 is not None else None)
                tri = self.accessor(p.indices).reshape(-1, 3).astype(np.int64)
                yield j, k, p.material, pos, nrm, uv, tri


class Batch:
    """Accumulates indexed triangle sets per material."""
    def __init__(self):
        self.parts = {}

    def add(self, material, pos, nrm, uv, tri):
        if len(tri) == 0:
            return
        used, remap = np.unique(tri, return_inverse=True)
        self.parts.setdefault(material, []).append(
            (pos[used], nrm[used], None if uv is None else uv[used], remap.reshape(-1, 3)))

    def triangles(self):
        out = [p[v] for parts in self.parts.values() for p, _, _, v in parts]
        return np.concatenate(out) if out else np.zeros((0, 3, 3))

    def tri_mats(self):
        tris, mats = [], []
        for mat, parts in self.parts.items():
            for p, _, _, v in parts:
                tris.append(p[v])
                mats.append(np.full(len(v), -1 if mat is None else mat))
        return np.concatenate(tris), np.concatenate(mats)

    def repaint(self, materials, select, new_material):
        """Move triangles of `materials` whose centroid passes `select` to `new_material`."""
        moved = 0
        for mat in [m for m in list(self.parts) if m in materials]:
            keep = []
            for p, n, uv, t in self.parts[mat]:
                hit = select(p[t].mean(1))
                if hit.any():
                    self.add(new_material, p, n, uv, t[hit])
                    moved += int(hit.sum())
                if (~hit).any():
                    keep.append((p, n, uv, t[~hit]))
            self.parts[mat] = keep
            if not keep:
                del self.parts[mat]
        return moved

    def transformed(self, matrix):
        b = Batch()
        for mat, parts in self.parts.items():
            for p, n, uv, t in parts:
                b.parts.setdefault(mat, []).append(
                    (p @ matrix[:3, :3].T + matrix[:3, 3], n @ matrix[:3, :3].T, uv, t))
        return b


def srgb_to_linear(c):
    c = np.asarray(c, float)
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


class Materials:
    """Source materials plus generated ones (team colours, stickers)."""
    def __init__(self, source):
        g = source.gltf
        self.materials = [json.loads(m.to_json()) for m in g.materials]
        self.textures = [json.loads(t.to_json()) for t in g.textures]
        self.samplers = [json.loads(t.to_json()) for t in g.samplers]
        self.images = []
        for image in g.images:
            view = g.bufferViews[image.bufferView]
            off = view.byteOffset or 0
            self.images.append({'data': source.blob[off:off + view.byteLength],
                                'mimeType': image.mimeType, 'name': image.name})

    def by_name(self, name):
        hits = [i for i, m in enumerate(self.materials) if m.get('name') == name]
        return hits[0] if hits else None

    def colour(self, name):
        i = self.by_name(name)
        if i is None:
            rgb = [float(x) for x in name.split('_', 1)[1].split(',')]
            template = dict(self.materials[self.by_name('colour_1.0000,0.0271,0.0271')])
            template['name'] = name
            template['pbrMetallicRoughness'] = dict(template['pbrMetallicRoughness'],
                                                    baseColorFactor=[*srgb_to_linear(rgb).tolist(), 1.0])
            self.materials.append(template)
            i = len(self.materials) - 1
        return i

    def sticker(self, name, png):
        self.images.append({'data': png, 'mimeType': 'image/png', 'name': name})
        self.samplers.append({'magFilter': 9729, 'minFilter': 9987, 'wrapS': 33071, 'wrapT': 33071})
        self.textures.append({'source': len(self.images) - 1, 'sampler': len(self.samplers) - 1})
        self.materials.append({'name': name, 'doubleSided': False, 'pbrMetallicRoughness': {
            'baseColorTexture': {'index': len(self.textures) - 1},
            'metallicFactor': 0.0, 'roughnessFactor': 0.7}})
        return len(self.materials) - 1

    def team_map(self, team, lights=False):
        table = RED_TO_BLUE if team == 'blue' else BLUE_TO_RED
        if lights and team == 'blue':
            table = {**table, **LIGHT_RED_TO_BLUE}
        out = {}
        for i, m in enumerate(self.materials):
            if m.get('name') in table:
                out[i] = self.colour(table[m['name']])
        return out


def write_glb(batch, mats, path):
    blob = bytearray()
    views, accessors, prims = [], [], []

    def add_view(data, target=None):
        while len(blob) % 4:
            blob.append(0)
        views.append({'buffer': 0, 'byteOffset': len(blob), 'byteLength': len(data),
                      **({'target': target} if target else {})})
        blob.extend(data)
        return len(views) - 1

    def add_accessor(array, ctype, kind, target, minmax=False):
        a = {'bufferView': add_view(array.tobytes(), target), 'componentType': ctype,
             'count': len(array), 'type': kind}
        if minmax:
            a['min'] = array.min(0).tolist()
            a['max'] = array.max(0).tolist()
        accessors.append(a)
        return len(accessors) - 1

    # Keep only the materials, textures, samplers and images this file uses.
    used = sorted(m for m in batch.parts if m is not None)
    mat_new = {m: i for i, m in enumerate(used)}
    materials = [json.loads(json.dumps(mats.materials[m])) for m in used]
    tex_new, textures, img_new, images, smp_new, samplers = {}, [], {}, [], {}, []
    for m in materials:
        info = m.get('pbrMetallicRoughness', {}).get('baseColorTexture')
        if not info:
            continue
        t = info['index']
        if t not in tex_new:
            tex = dict(mats.textures[t])
            if tex['source'] not in img_new:
                img_new[tex['source']] = len(images)
                images.append(mats.images[tex['source']])
            tex['source'] = img_new[tex['source']]
            if 'sampler' in tex:
                if tex['sampler'] not in smp_new:
                    smp_new[tex['sampler']] = len(samplers)
                    samplers.append(mats.samplers[tex['sampler']])
                tex['sampler'] = smp_new[tex['sampler']]
            tex_new[t] = len(textures)
            textures.append(tex)
        info['index'] = tex_new[t]

    for material in sorted(batch.parts, key=lambda m: -1 if m is None else m):
        parts = batch.parts[material]
        has_uv = any(uv is not None for _, _, uv, _ in parts)
        pos, nrm, uvs, idx, base = [], [], [], [], 0
        for p, n, uv, t in parts:
            pos.append(p); nrm.append(n); idx.append(t + base); base += len(p)
            uvs.append(uv if uv is not None else np.zeros((len(p), 2)))
        attrs = {'POSITION': add_accessor(np.concatenate(pos).astype(np.float32), 5126, 'VEC3', 34962, True),
                 'NORMAL': add_accessor(np.concatenate(nrm).astype(np.float32), 5126, 'VEC3', 34962)}
        if has_uv:
            attrs['TEXCOORD_0'] = add_accessor(np.concatenate(uvs).astype(np.float32), 5126, 'VEC2', 34962)
        prim = {'attributes': attrs, 'mode': 4,
                'indices': add_accessor(np.concatenate(idx).astype(np.uint32).ravel(), 5125, 'SCALAR', 34963)}
        if material is not None:
            prim['material'] = mat_new[material]
        prims.append(prim)

    images = [{'bufferView': add_view(im['data']), 'mimeType': im['mimeType'], 'name': im['name']}
              for im in images]
    while len(blob) % 4:
        blob.append(0)

    doc = {'asset': {'version': '2.0', 'generator': 'ego-ros2-gazebo-d430-sim build_from_glb.py',
                     'copyright': 'CAD geometry, appearance and sticker artwork: DJI / RoboMaster'},
           'scene': 0, 'scenes': [{'nodes': [0]}],
           'nodes': [{'name': path.stem, 'mesh': 0}],
           'meshes': [{'name': path.stem, 'primitives': prims}],
           'materials': materials, 'textures': textures,
           'samplers': samplers, 'images': images,
           'buffers': [{'byteLength': len(blob)}], 'bufferViews': views, 'accessors': accessors}
    doc = json.loads(json.dumps(doc), object_hook=lambda d: {
        k: v for k, v in d.items() if v is not None and not (isinstance(v, (dict, list)) and not v)})
    text = json.dumps(doc, separators=(',', ':')).encode()
    text += b' ' * (-len(text) % 4)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('wb') as f:
        f.write(struct.pack('<III', 0x46546C67, 2, 28 + len(text) + len(blob)))
        f.write(struct.pack('<II', len(text), 0x4E4F534A) + text)
        f.write(struct.pack('<II', len(blob), 0x004E4942) + bytes(blob))


def gather(src, node, world, exclude=()):
    """Triangles and per-triangle source material for a subtree, in the frame `world` maps to."""
    tris, mats = [], []
    for j, k, mat, pos, nrm, uv, tri in src.primitives(node, world):
        if j in exclude:
            continue
        tris.append(pos[tri])
        mats.append(np.full(len(tri), -1 if mat is None else mat))
    return np.concatenate(tris), np.concatenate(mats)


def planar_patches(tris, mats):
    """Connected sets of coplanar, same-material triangles."""
    m = trimesh.Trimesh(tris.reshape(-1, 3), np.arange(len(tris) * 3).reshape(-1, 3), process=False)
    m.merge_vertices(digits_vertex=5)
    n = m.face_normals
    adj = m.face_adjacency
    ok = (np.einsum('ij,ij->i', n[adj[:, 0]], n[adj[:, 1]]) > 0.9999) & (mats[adj[:, 0]] == mats[adj[:, 1]])
    labels = connected_component_labels(adj[ok], len(m.faces))
    area = np.bincount(labels, m.area_faces)
    out = []
    for lab in np.where(area > 1e-3)[0]:
        f = np.where(labels == lab)[0]
        verts = m.vertices[m.faces[f]].reshape(-1, 3)
        centre = (m.area_faces[f, None] * m.triangles_center[f]).sum(0) / area[lab]
        centred = verts - verts.mean(0)
        ext = np.ptp(centred @ np.linalg.svd(centred)[2][:2].T, axis=0)
        out.append({'c': centre, 'n': n[f[0]], 'verts': verts, 'area': float(area[lab]),
                    'mat': int(mats[f[0]]), 'ext': np.sort(ext)[::-1]})
    return out


def outermost(patch, patches, outward_hint):
    """The plate may be covered by thin parallel faces (e.g. a black armor skin 2 mm above
    the grey base armor plate). Return the outermost face of that stack, within 2 cm."""
    n0 = patch['n'] * (1 if patch['n'] @ outward_hint > 0 else -1)
    best, top = patch, (patch['verts'] @ n0).max()
    for p in patches:
        d = p['c'] - patch['c']
        h = (p['verts'] @ n0).max()
        if (abs(p['n'] @ n0) > 0.999 and np.linalg.norm(d - (d @ n0) * n0) < 0.02
                and 0 < h - top < 0.02 and p['ext'][1] > 0.8 * patch['ext'][1]):
            best, top = p, h
    return best


def sticker_frame(patch, outward_hint):
    """Outward normal, in-plane right/up axes (up = world +Z projected), centre and size."""
    n = patch['n'] * (1 if patch['n'] @ outward_hint > 0 else -1)
    up = np.array([0.0, 0.0, 1.0]) - n[2] * n
    up /= np.linalg.norm(up)
    right = np.cross(up, n)
    r, u = patch['verts'] @ right, patch['verts'] @ up
    centre = (patch['verts'] @ n).max() * n + (r.max() + r.min()) / 2 * right + (u.max() + u.min()) / 2 * up
    return n, right, up, centre, float(np.ptp(r)), float(np.ptp(u))


def sticker_shape(path):
    """Crop the artwork to its outline and triangulate the outline (the PDF bitmaps have a
    white background outside the notched outline). Returns png bytes, aspect, 2-D triangles
    in normalised image coordinates (u right, v down)."""
    rgb = np.asarray(Image.open(path).convert('RGB'))
    light = rgb.min(2) > 200
    labels, _ = ndimage.label(light)
    border = np.unique(np.r_[labels[0], labels[-1], labels[:, 0], labels[:, -1]])
    background = np.isin(labels, border[border > 0])
    rows, cols = np.where(~background)
    r0, r1, c0, c1 = rows.min(), rows.max() + 1, cols.min(), cols.max() + 1
    inside = ~background[r0:r1, c0:c1]
    crop = rgb[r0:r1, c0:c1].copy()
    crop[~inside] = 0  # bleed the outline colour so filtering never shows white at the edge
    h, w = inside.shape
    import contourpy
    lines = contourpy.contour_generator(z=np.pad(inside, 1).astype(float)).lines(0.5)
    ring = max(lines, key=len) - 1.0  # undo padding; columns are (x, y) in pixels
    poly = shapely.geometry.Polygon(ring).buffer(0).simplify(1.0)
    if poly.geom_type != 'Polygon':
        poly = max(poly.geoms, key=lambda g: g.area)
    verts, faces = trimesh.creation.triangulate_polygon(poly, engine='earcut')
    uv = np.clip(verts / [w, h], 0, 1)
    buf = io.BytesIO()
    Image.fromarray(crop).save(buf, 'PNG', optimize=True)
    return buf.getvalue(), w / h, uv[faces]


def add_sticker(batch, material, shape, frame, scale=1.0):
    """Place sticker `shape` on `frame`, as large as fits the plate, keeping its aspect."""
    _, aspect, uv_tris = shape
    n, right, up, centre, w, h = frame
    width = min(w, h * aspect) * scale
    height = width / aspect
    uv = uv_tris.reshape(-1, 2)
    pos = (centre + STICKER_OFFSET * n + np.outer(uv[:, 0] - 0.5, right * width)
           + np.outer(0.5 - uv[:, 1], up * height))
    tri = np.arange(len(uv)).reshape(-1, 3)
    # Keep the front faces counter-clockwise when seen from outside.
    if np.cross(pos[tri[0, 1]] - pos[tri[0, 0]], pos[tri[0, 2]] - pos[tri[0, 0]]) @ n < 0:
        tri = tri[:, ::-1]
    batch.add(material, pos, np.tile(n, (len(pos), 1)), uv, tri)
    return {'centre': centre.tolist(), 'normal': n.tolist(), 'size_m': [width, height],
            'plate_m': [w, h]}


def soup(triangles):
    return trimesh.Trimesh(triangles.reshape(-1, 3), np.arange(len(triangles) * 3).reshape(-1, 3),
                           process=False)


def fmt(v):
    return ' '.join(f'{x:.6f}'.rstrip('0').rstrip('.') if abs(x) > 5e-7 else '0' for x in v)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('glb', type=Path)
    ap.add_argument('--skip-hash', action='store_true')
    ap.add_argument('--visual-only', action='store_true', help='keep the existing collision mesh')
    args = ap.parse_args()
    digest = hashlib.sha256(args.glb.read_bytes()).hexdigest()
    if digest != GLB_SHA256 and not args.skip_hash:
        raise SystemExit(f'unexpected GLB SHA-256 {digest}; selectors were reviewed for {GLB_SHA256}')

    src = Source(args.glb)
    field_root = src.find(0, 'rmuc-2027')
    root_world = np.eye(4)
    root_world[:3, 3] = FIELD_OFFSET  # node 0 (Y-up presentation frame) is deliberately skipped
    equipment = src.find(field_root, 'equipment')

    # Outpost selections, by name.
    teams = {}
    for team in ('red', 'blue'):
        top = src.find(equipment, f'{team}-outpost-with-footing')
        outpost = src.find(top, 'outpost')
        fixed = src.find(outpost, 'fixed')
        upper = src.find(fixed, 'outpost/fixed/upper-cluster')
        mesh_nodes = [(sum(src.accessor(p.indices).size for p in src.gltf.meshes[src.nodes[j].mesh].primitives), j)
                      for j, _ in src.walk(upper) if src.nodes[j].mesh is not None]
        teams[team] = {'rotor': src.find(outpost, 'rotor'), 'carrier_host': max(mesh_nodes)[1],
                       'outpost': outpost}
    rotor_nodes = {j: team for team, t in teams.items() for j, _ in src.walk(t['rotor'])}
    carrier_hosts = {t['carrier_host']: team for team, t in teams.items()}
    world_of = dict(src.walk(field_root, root_world))

    # Team ownership of shared equipment, for the colour fix.
    mats = Materials(src)
    team_map = {team: mats.team_map(team) for team in teams}
    light_map = {team: mats.team_map(team, lights=True) for team in teams}
    lit = set()
    owner, owned_report = {}, {}
    containers = [equipment] + [src.find(field_root, n) for n in ('terrain', 'fixtures')]
    for parent in containers:
        for c in src.nodes[parent].children:
            name = src.nodes[c].name
            if name.startswith(('red-', 'blue-')) and parent == equipment:
                team = name.split('-')[0]
            elif name in TEAM_OWNED_BY_POSITION:
                centre = gather(src, c, world_of[parent])[0].reshape(-1, 3).mean(0)
                team = 'red' if centre[0] > CENTRE_XY[0] else 'blue'
            else:
                continue
            for j, _ in src.walk(c):
                owner[j] = team
                if name in LIGHT_OWNERS:
                    lit.add(j)
            owned_report[f'{name}#{c}'] = team

    recoloured = {}

    def recolour(j, mat, count=0):
        team = owner.get(j)
        table = light_map if j in lit else team_map
        new = table[team].get(mat, mat) if team else mat
        if new != mat and count:
            key = f'{team}: {mats.materials[mat]["name"]} -> {mats.materials[new]["name"]}'
            recoloured[key] = recoloured.get(key, 0) + count
        return new

    field = Batch()
    rotor = {team: Batch() for team in teams}
    for j, k, mat, pos, nrm, uv, tri in src.primitives(field_root, root_world):
        if j in rotor_nodes:
            rotor[rotor_nodes[j]].add(recolour(j, mat, len(tri)), pos, nrm, uv, tri)
    module_top = {team: rotor[team].triangles()[..., 2].max() for team in teams}

    carrier_tris = {team: 0 for team in teams}
    static_parts = {}  # node -> triangles of the static field, the collision source
    for j, k, mat, pos, nrm, uv, tri in src.primitives(field_root, root_world):
        if j in rotor_nodes:
            continue
        if j in carrier_hosts:
            # The carrier is fused into the fixed upper body in this GLB. Its disconnected
            # pieces lie entirely below the top of the armor modules; the fixed dart target
            # and its hardware extend above. Split by connected component, not by clipping.
            team = carrier_hosts[j]
            merged = trimesh.Trimesh(pos, tri, process=True, validate=False)
            assert len(merged.faces) == len(tri), 'face count changed while merging vertices'
            labels = connected_component_labels(merged.face_adjacency, len(merged.faces))
            zmax = np.zeros(labels.max() + 1)
            np.maximum.at(zmax, labels, pos[tri][..., 2].max(1))
            moving = zmax[labels] <= module_top[team] + 0.01
            rotor[team].add(recolour(j, mat, len(tri)), pos, nrm, uv, tri[moving])
            field.add(recolour(j, mat), pos, nrm, uv, tri[~moving])
            static_parts.setdefault(j, []).append(pos[tri[~moving]])
            carrier_tris[team] += int(moving.sum())
        else:
            field.add(recolour(j, mat, len(tri)), pos, nrm, uv, tri)
            static_parts.setdefault(j, []).append(pos[tri])

    # Axis: rm-map-tools rules/source-motion-rmuc2027.json, joint outpost.rotor.spin:
    # axis +Y, origin (-9.4379, 1137.44, -1.0455) mm in the outpost product frame, which is
    # the GLB "outpost" node frame. The CAD places the outpost tilted 1.5 deg, so the axis
    # is not exactly vertical. check_axis() confirms it against the armor-module geometry.
    report = {'source_glb_sha256': digest, 'field_offset': FIELD_OFFSET.tolist(), 'teams': {},
              'team_owner': owned_report, 'recoloured_triangles': recoloured}
    frames = {}
    for team, t in teams.items():
        node = world_of[t['outpost']]
        frame = np.eye(4)
        frame[:3, :3] = node[:3, :3] @ Rotation.from_euler('x', -np.pi / 2).as_matrix()
        frame[:3, 3] = (node @ np.r_[SOURCE_AXIS_ORIGIN_MM / 1000.0, 1.0])[:3]
        frames[team] = frame
        rpy = Rotation.from_matrix(frame[:3, :3]).as_euler('xyz')
        modules = []
        for module in src.nodes[t['rotor']].children:
            b = Batch()
            for j, k, mat, pos, nrm, uv, tri in src.primitives(module, world_of[t['rotor']]):
                b.add(mat, pos, nrm, uv, tri)
            modules.append(b.transformed(np.linalg.inv(frame)).triangles())
        report['teams'][team] = {
            'model_pose_xyz_rpy': [*frame[:3, 3].tolist(), *rpy.tolist()],
            'axis_world': frame[:3, 2].tolist(),
            'axis_tilt_deg': float(np.degrees(np.arccos(frame[2, 2]))),
            'module_120deg_fit_median_m': check_axis(modules),
            'carrier_triangles': carrier_tris[team],
            'rotor_triangles': int(len(rotor[team].triangles()))}
        assert max(report['teams'][team]['module_120deg_fit_median_m']) < 1e-3

    local = {team: rotor[team].transformed(np.linalg.inv(frames[team])) for team in teams}
    red_t, blue_t = local['red'].triangles(), local['blue'].triangles()
    assert red_t.shape == blue_t.shape
    gap = cKDTree(red_t.reshape(-1, 3)).query(blue_t.reshape(-1, 3))[0].max()
    report['red_blue_rotor_max_vertex_gap_m'] = float(gap)
    assert gap < 1e-4, gap

    # Stickers.
    shapes = {k: sticker_shape(STICKER_DIR / f'{k}.png')
              for k in ('outpost_armor', 'outpost_dart', 'base_armor', 'base_dart')}
    sticker_mat = {k: mats.sticker(f'sticker-{k}', v[0]) for k, v in shapes.items()}
    name_of = lambda m: mats.materials[m]['name'] if m >= 0 else None
    green = [i for i, m in enumerate(mats.materials)  # guide-light green, linear base colour
             if (lambda c: c[1] > 0.5 and c[0] < 0.1 and c[2] < 0.1)(
                 m['pbrMetallicRoughness'].get('baseColorFactor') or [1, 1, 1])]
    stickers = report['stickers'] = {}

    strip_src = [i for i, m in enumerate(mats.materials) if m.get('name') in DART_STRIP_MATERIALS]

    def light_strips(frame, team):
        n, right, up, centre = frame[:4]
        x0, x1, y1, h0, h1 = DART_STRIP_BOX

        def select(c):
            d = c - centre
            x, y, h = np.abs(d @ right), np.abs(d @ up), d @ n
            return (x > x0) & (x < x1) & (y < y1) & (h > h0) & (h < h1)
        moved = field.repaint(strip_src, select, mats.colour(TEAM_LIGHT[team]))
        assert moved > 100, (team, moved)
        return moved

    def dart_plate(tris, mt, label):
        # Dart detection module: the outermost square plate (~0.14 m) next to the green guide light.
        light = tris[np.isin(mt, green)].reshape(-1, 3).mean(0)
        cand = [p for p in planar_patches(tris, mt)
                if name_of(p['mat']) == 'colour_0.2000,0.2000,0.2000' and 0.13 < p['ext'][1]
                and p['ext'][0] < 0.16 and np.linalg.norm(p['c'] - light) < 0.2]
        outward = np.array([0, 0, 1.0])
        frames = [sticker_frame(p, outward) for p in cand]
        assert frames, label
        # Outermost face of the plate stack, measured along one shared normal.
        n0 = frames[0][0]
        best = max(range(len(frames)), key=lambda i: (cand[i]['verts'] @ n0).max())
        return frames[best]

    for team, t in teams.items():
        # Outpost armor: the light-grey face (0.117 x 0.102 m) on each rotor module.
        # The plates sit in the carrier mesh split off the fixed body, so search the whole
        # assembled rotor (rotor frame) rather than the source "rotor" node.
        tris, mt = local[team].tri_mats()
        patches = planar_patches(tris, mt)
        plates = [p for p in patches if name_of(p['mat']) == 'colour_default'
                  and abs(p['ext'][0] - 0.117) < 0.005 and abs(p['ext'][1] - 0.102) < 0.005]
        best = {}
        for p in plates:  # one per module, the radially outermost
            radial = np.r_[p['c'][:2], 0.0] / np.linalg.norm(p['c'][:2])
            key = round(float(np.degrees(np.arctan2(p['c'][1], p['c'][0]))) / 10)
            fr = sticker_frame(outermost(p, patches, radial), radial)
            if key not in best or fr[3] @ radial > best[key][3] @ radial:
                best[key] = fr
        assert len(best) == 3, (team, len(best))
        stickers[f'{team}/outpost_armor'] = [add_sticker(local[team], sticker_mat['outpost_armor'],
                                                         shapes['outpost_armor'], fr) for fr in best.values()]
        # Outpost dart detection module (fixed part, world frame).
        top = world_of[t['outpost']] @ np.linalg.inv(src.local(t['outpost']))
        tris, mt = gather(src, t['outpost'], top, exclude=set(rotor_nodes))
        fr = dart_plate(tris, mt, f'{team} outpost dart')
        stickers[f'{team}/outpost_dart'] = [add_sticker(field, sticker_mat['outpost_dart'], shapes['outpost_dart'], fr)]
        stickers[f'{team}/outpost_dart'][0]['light_strip_triangles'] = light_strips(fr, team)
        # Base: three exposed armor plates on top, and the dart detection module.
        base = src.find(equipment, f'{team}-base-with-footing')
        tris, mt = gather(src, base, world_of[equipment])
        centroid = tris.reshape(-1, 3).mean(0)
        patches = planar_patches(tris, mt)
        plates = [p for p in patches if name_of(p['mat']) == 'colour_0.6667,0.7059,0.7647'
                  and p['area'] > 0.01]
        assert len(plates) == 3, (team, len(plates))
        stickers[f'{team}/base_armor'] = [add_sticker(field, sticker_mat['base_armor'], shapes['base_armor'],
                                                      sticker_frame(outermost(p, patches, p['c'] - centroid),
                                                                    p['c'] - centroid)) for p in plates]
        fr = dart_plate(tris, mt, f'{team} base dart')
        stickers[f'{team}/base_dart'] = [add_sticker(field, sticker_mat['base_dart'], shapes['base_dart'], fr)]
        stickers[f'{team}/base_dart'][0]['light_strip_triangles'] = light_strips(fr, team)

    for team in teams:
        out_rotor = MODELS / f'rmuc_2027_outpost_rotor_{team}'
        write_glb(local[team], mats, out_rotor / 'meshes/rotor_visual.glb')
    write_glb(field, mats, MODELS / 'rmuc_2027/meshes/rmuc_2027_visual.glb')
    report['field_visual_triangles'] = int(len(field.triangles()))
    report['field_visual_primitives'] = len(field.parts)

    # Armor-module collision boxes in the rotor frame.
    boxes = []
    for idx, module in enumerate(src.nodes[teams['red']['rotor']].children):
        b = Batch()
        for j, k, mat, pos, nrm, uv, tri in src.primitives(module, world_of[teams['red']['rotor']]):
            b.add(mat, pos, nrm, uv, tri)
        pts = b.transformed(np.linalg.inv(frames['red'])).triangles().reshape(-1, 3)
        to_box, extents = trimesh.bounds.oriented_bounds(pts)
        pose = np.linalg.inv(to_box)
        boxes.append((idx, pose[:3, 3], Rotation.from_matrix(pose[:3, :3]).as_euler('xyz'), extents))

    for team in teams:
        write_rotor_sdf(MODELS / f'rmuc_2027_outpost_rotor_{team}', team, boxes)
    report['armor_boxes'] = [{'module': i, 'xyz': p.tolist(), 'rpy': r.tolist(), 'size': e.tolist()}
                             for i, p, r, e in boxes]
    report_path = HERE / 'build_report.json'
    if args.visual_only:
        old = json.loads(report_path.read_text()) if report_path.exists() else {}
        report.update({k: v for k, v in old.items() if k.startswith('collision_')})
        report_path.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps({k: v for k, v in report.items() if k not in ('armor_boxes', 'stickers')}, indent=2))
        return

    # Collision: the static field (outpost rotors and their carriers already split off,
    # stickers not included), simplified part by part.
    parts, kinds = [], {}
    for j in sorted(static_parts):
        part, kind = collision_part(np.concatenate(static_parts[j]))
        kinds[kind] = kinds.get(kind, 0) + 1
        if part is not None:
            parts.append(part)
    col = trimesh.util.concatenate(parts)
    col.export(MODELS / 'rmuc_2027/meshes/rmuc_2027_collision.stl')
    report['collision_source_triangles'] = int(sum(len(np.concatenate(v)) for v in static_parts.values()))
    report['collision_triangles'] = int(len(col.faces))
    report['collision_parts'] = kinds

    report_path.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k not in ('armor_boxes', 'stickers')}, indent=2))


def collision_part(tris):
    """Simplified collision mesh for one GLB part, and how it was made."""
    import fast_simplification
    m = soup(tris)
    m.merge_vertices(digits_vertex=4)
    m.update_faces(m.nondegenerate_faces())
    m.remove_unreferenced_vertices()
    if len(m.faces) == 0:
        return None, 'degenerate'
    size = float(np.linalg.norm(np.ptp(m.vertices, 0)))
    if size < COLLISION_MIN_SIZE:
        return None, 'dropped'
    if len(m.faces) <= COLLISION_MIN_FACES:
        return m, 'kept'
    pts = trimesh.sample.sample_surface(m, 400, seed=0)[0]
    for ratio in COLLISION_RATIOS:
        n = int(len(m.faces) * ratio)
        if n < COLLISION_MIN_FACES:
            continue
        v, f = fast_simplification.simplify(m.vertices.astype(np.float32), m.faces.astype(np.int64),
                                            target_count=n)
        if len(f) == 0:
            continue
        d = trimesh.Trimesh(v, f, process=False)
        d.update_faces(d.nondegenerate_faces())
        if len(d.faces) == 0:
            continue
        dist = trimesh.proximity.closest_point(d, pts)[1]
        if np.percentile(dist, 98) < COLLISION_TOLERANCE and dist.max() < 4 * COLLISION_TOLERANCE:
            return d, 'decimated'
    if size < COLLISION_HULL_SIZE:
        hull = m.convex_hull
        if len(hull.faces) > 64:
            v, f = fast_simplification.simplify(hull.vertices.astype(np.float32),
                                                hull.faces.astype(np.int64), target_count=64)
            hull = trimesh.Trimesh(v, f, process=False)
        return hull, 'hull'
    return m, 'kept'


def check_axis(modules):
    """Rotate each armor module by 120 deg about the rotor axis, shift it along the axis by
    the centroid height difference, and return the median distance to the next module."""
    out = []
    for a, b in ((0, 1), (1, 2), (2, 0)):
        ca, cb = modules[a].reshape(-1, 3).mean(0), modules[b].reshape(-1, 3).mean(0)
        ang = np.arctan2(cb[1], cb[0]) - np.arctan2(ca[1], ca[0])
        m = np.eye(4)
        m[:3, :3] = Rotation.from_euler('z', ang).as_matrix()
        m[2, 3] = cb[2] - ca[2]
        pts = trimesh.transform_points(soup(modules[a]).sample(4000), m)
        out.append(float(np.median(trimesh.proximity.closest_point(soup(modules[b]), pts)[1])))
    return out


def write_rotor_sdf(out, team, boxes):
    collisions = '\n'.join(f'''      <collision name="armor_module_{i}">
        <pose>{fmt(p)} {fmt(r)}</pose>
        <geometry><box><size>{fmt(e)}</size></box></geometry>
        <surface><contact><collide_bitmask>0x02</collide_bitmask></contact></surface>
      </collision>''' for i, p, r, e in boxes)
    (out / 'model.sdf').write_text(f'''<?xml version="1.0"?>
<!-- Generated by tools/build_from_glb.py. Place the model frame on the rotor axis
     (see tools/build_report.json); the rotor spins about +Z. -->
<sdf version="1.10">
  <model name="rmuc_2027_outpost_rotor_{team}">
    <link name="rotor">
      <gravity>false</gravity>
      <!-- Placeholder inertia (no measured mass in the CAD): 2 kg ring, r = 0.27 m. -->
      <inertial>
        <mass>2.0</mass>
        <inertia><ixx>0.073</ixx><iyy>0.073</iyy><izz>0.146</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <visual name="visual">
        <geometry><mesh><uri>model://rmuc_2027_outpost_rotor_{team}/meshes/rotor_visual.glb</uri></mesh></geometry>
      </visual>
{collisions}
    </link>
    <joint name="rotor_joint" type="revolute">
      <parent>world</parent>
      <child>rotor</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-1e16</lower><upper>1e16</upper></limit>
        <dynamics><damping>0</damping><friction>0</friction></dynamics>
      </axis>
    </joint>
    <!-- Velocity mode. Change speed at runtime, e.g. stop:
         gz topic -t /model/<name>/joint/rotor_joint/cmd_vel -m gz.msgs.Double -p 'data: 0' -->
    <plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController">
      <joint_name>rotor_joint</joint_name>
      <initial_velocity>{ROTOR_SPEED!r}</initial_velocity>
    </plugin>
  </model>
</sdf>
''')
    (out / 'model.config').write_text(f'''<?xml version="1.0"?>
<model>
  <name>rmuc_2027_outpost_rotor_{team}</name>
  <version>1.0</version>
  <sdf version="1.10">model.sdf</sdf>
  <author><name>RMUC 2027 field conversion</name><email>none</email></author>
  <description>
    RMUC 2027 {team} outpost rotor: three armor modules (with the official outpost armor
    stickers) and carrier from the rm-map-tools simplified GLB, spinning at 0.8*pi rad/s about a vertical axis. Collision is one
    box per armor module (bitmask 0x02, so it ignores the static field).
  </description>
</model>
''')


if __name__ == '__main__':
    main()
