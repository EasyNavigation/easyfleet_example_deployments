#!/usr/bin/env python3
# Copyright 2026 Intelligent Robotics Lab
#
# This file is part of the project EasyFleet
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Build a 2D occupancy map (PGM + map_server YAML) from a Gazebo SDF world.

Every collision (or visual) geometry (box, cylinder, sphere, DAE/OBJ/GLB mesh) of the world
and of the models it includes is turned into triangles in the world frame,
clipped to the height band a ground robot can hit, and its footprint is
filled in the map. A wall leaves a line, a low shelf leaves an area.
"""

import argparse
import math
import os
import xml.etree.ElementTree as ET

import cv2
import numpy as np


# -- poses -------------------------------------------------------------------

def pose_matrix(text):
    """4x4 transform from an SDF '<pose>' text 'x y z roll pitch yaw'."""
    v = [float(x) for x in (text or '').split()] + [0.0] * 6
    x, y, z, roll, pitch, yaw = v[:6]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    m = np.eye(4)
    m[:3, :3] = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])
    m[:3, 3] = [x, y, z]
    return m


def child_pose(elem):
    p = elem.find('pose')
    return pose_matrix(p.text if p is not None else None)


# -- primitive and mesh triangles (local frame) -------------------------------

def box_triangles(size):
    sx, sy, sz = [s / 2.0 for s in size]
    c = np.array([[x, y, z] for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)])
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    tris = []
    for a, b, cc, d in faces:
        tris.append(c[[a, b, cc]])
        tris.append(c[[a, cc, d]])
    return np.array(tris)


def cylinder_triangles(radius, length, n=24):
    ang = np.linspace(0, 2 * math.pi, n, endpoint=False)
    ring = np.stack([radius * np.cos(ang), radius * np.sin(ang)], axis=1)
    lo, hi = -length / 2.0, length / 2.0
    tris = []
    for i in range(n):
        p, q = ring[i], ring[(i + 1) % n]
        a, b = [p[0], p[1], lo], [q[0], q[1], lo]
        c, d = [q[0], q[1], hi], [p[0], p[1], hi]
        tris += [[a, b, c], [a, c, d], [[0, 0, lo], b, a], [[0, 0, hi], d, c]]
    return np.array(tris, dtype=float)


def sphere_triangles(radius):
    # A box is a close enough footprint for the few spheres a world has.
    return box_triangles([2 * radius] * 3)


def load_obj(path):
    verts, tris = [], []
    with open(path, errors='ignore') as f:
        for line in f:
            if line.startswith('v '):
                verts.append([float(x) for x in line.split()[1:4]])
            elif line.startswith('f '):
                idx = [int(tok.split('/')[0]) for tok in line.split()[1:]]
                idx = [i - 1 if i > 0 else len(verts) + i for i in idx]
                for k in range(1, len(idx) - 1):
                    tris.append((idx[0], idx[k], idx[k + 1]))
    if not tris:
        return np.zeros((0, 3, 3))
    return np.array(verts)[np.array(tris)]


def _strip_ns(root):
    for el in root.iter():
        if '}' in el.tag:
            el.tag = el.tag.split('}', 1)[1]
    return root


def _node_matrix(node):
    m = np.eye(4)
    for t in node:
        if t.tag == 'matrix':
            m = m @ np.array([float(x) for x in t.text.split()]).reshape(4, 4)
        elif t.tag == 'translate':
            tr = np.eye(4)
            tr[:3, 3] = [float(x) for x in t.text.split()]
            m = m @ tr
        elif t.tag == 'rotate':
            ax, ay, az, deg = [float(x) for x in t.text.split()]
            a = math.radians(deg)
            k = np.array([ax, ay, az]) / (np.linalg.norm([ax, ay, az]) or 1)
            kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
            r = np.eye(4)
            r[:3, :3] = np.eye(3) + math.sin(a) * kx + (1 - math.cos(a)) * kx @ kx
            m = m @ r
        elif t.tag == 'scale':
            s = np.eye(4)
            s[0, 0], s[1, 1], s[2, 2] = [float(x) for x in t.text.split()]
            m = m @ s
    return m


def load_dae(path):
    root = _strip_ns(ET.parse(path).getroot())
    unit = root.find('asset/unit')
    scale = float(unit.get('meter', '1')) if unit is not None else 1.0
    up = root.find('asset/up_axis')
    fix = np.eye(4)
    if up is not None and up.text.strip() == 'Y_UP':
        fix[:3, :3] = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]

    geometries = {}
    for geom in root.iter('geometry'):
        mesh = geom.find('mesh')
        if mesh is None:
            continue
        sources = {s.get('id'): s for s in mesh.findall('source')}
        verts_el = mesh.find('vertices')
        pos_src = None
        if verts_el is not None:
            for inp in verts_el.findall('input'):
                if inp.get('semantic') == 'POSITION':
                    pos_src = sources.get(inp.get('source').lstrip('#'))
        if pos_src is None:
            continue
        pos = np.array([float(x) for x in pos_src.find('float_array').text.split()]).reshape(-1, 3)
        tris = []
        for prim in list(mesh.findall('triangles')) + list(mesh.findall('polylist')) + \
                list(mesh.findall('polygons')):
            inputs = prim.findall('input')
            stride = max(int(i.get('offset', 0)) for i in inputs) + 1
            voff = next(int(i.get('offset', 0)) for i in inputs if i.get('semantic') == 'VERTEX')
            ps = [p for p in prim.findall('p') if p.text and p.text.strip()]
            if not ps:
                continue
            if prim.tag == 'polylist':
                counts = [int(x) for x in prim.find('vcount').text.split()]
                idx = np.array([int(x) for x in ps[0].text.split()])[voff::stride]
                k = 0
                for c in counts:
                    poly = idx[k:k + c]
                    k += c
                    for j in range(1, c - 1):
                        tris.append((poly[0], poly[j], poly[j + 1]))
            else:
                for p in ps:
                    idx = np.array([int(x) for x in p.text.split()])[voff::stride]
                    if prim.tag == 'triangles':
                        tris += [tuple(idx[i:i + 3]) for i in range(0, len(idx) - 2, 3)]
                    else:
                        for j in range(1, len(idx) - 1):
                            tris.append((idx[0], idx[j], idx[j + 1]))
        if tris:
            geometries[geom.get('id')] = pos[np.array(tris)]

    out = []

    def walk(node, m):
        m = m @ _node_matrix(node)
        for ig in node.findall('instance_geometry'):
            g = geometries.get(ig.get('url', '').lstrip('#'))
            if g is not None:
                out.append(transform(g, m))
        for child in node.findall('node'):
            walk(child, m)

    scene = root.find('scene/instance_visual_scene')
    vs = None
    if scene is not None:
        vs_id = scene.get('url', '').lstrip('#')
        vs = next((v for v in root.iter('visual_scene') if v.get('id') == vs_id), None)
    if vs is None:
        out = list(geometries.values())
    else:
        for node in vs.findall('node'):
            walk(node, np.eye(4))
    if not out:
        return np.zeros((0, 3, 3))
    return transform(np.concatenate(out), fix @ np.diag([scale, scale, scale, 1]))


def load_glb(path):
    import json
    import struct
    data = open(path, 'rb').read()
    if data[:4] != b'glTF':
        raise ValueError('not a GLB file')
    offset, gltf, binary = 12, None, b''
    while offset < len(data):
        length, kind = struct.unpack_from('<II', data, offset)
        chunk = data[offset + 8:offset + 8 + length]
        if kind == 0x4E4F534A:
            gltf = json.loads(chunk)
        elif kind == 0x004E4942:
            binary = chunk
        offset += 8 + length
    comp = {5120: 'i1', 5121: 'u1', 5122: 'i2', 5123: 'u2', 5125: 'u4', 5126: 'f4'}
    ncomp = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4, 'MAT4': 16}

    def accessor(i):
        acc = gltf['accessors'][i]
        view = gltf['bufferViews'][acc['bufferView']]
        n, k = acc['count'], ncomp[acc['type']]
        dt = np.dtype('<' + comp[acc['componentType']])
        start = view.get('byteOffset', 0) + acc.get('byteOffset', 0)
        stride = view.get('byteStride', dt.itemsize * k)
        raw = np.frombuffer(binary, dtype=np.uint8, count=stride * (n - 1) + dt.itemsize * k,
                            offset=start)
        rows = np.lib.stride_tricks.as_strided(raw, (n, dt.itemsize * k), (stride, 1))
        return np.ascontiguousarray(rows).view(dt).reshape(n, k)

    def node_matrix(node):
        if 'matrix' in node:
            return np.array(node['matrix']).reshape(4, 4).T
        m = np.eye(4)
        x, y, z, w = node.get('rotation', [0, 0, 0, 1])
        m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
        m[:3, :3] = m[:3, :3] * np.array(node.get('scale', [1, 1, 1]))
        m[:3, 3] = node.get('translation', [0, 0, 0])
        return m

    out = []

    def walk(i, m):
        node = gltf['nodes'][i]
        m = m @ node_matrix(node)
        if 'mesh' in node:
            for prim in gltf['meshes'][node['mesh']]['primitives']:
                if prim.get('mode', 4) != 4 or 'POSITION' not in prim['attributes']:
                    continue
                pos = accessor(prim['attributes']['POSITION']).astype(float)
                idx = accessor(prim['indices']).ravel() if 'indices' in prim else \
                    np.arange(len(pos))
                out.append(transform(pos[idx[:len(idx) // 3 * 3].reshape(-1, 3)], m))
        for child in node.get('children', []):
            walk(child, m)

    scene = gltf['scenes'][gltf.get('scene', 0)]
    # glTF is Y-up; SDF/Gazebo is Z-up.
    y_up = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]], dtype=float)
    for i in scene['nodes']:
        walk(i, y_up)
    return np.concatenate(out) if out else np.zeros((0, 3, 3))


def transform(tris, m):
    flat = tris.reshape(-1, 3)
    flat = flat @ m[:3, :3].T + m[:3, 3]
    return flat.reshape(-1, 3, 3)


# -- SDF walking ---------------------------------------------------------------

class World:

    def __init__(self, model_paths, element='collision'):
        self.model_paths = model_paths
        self.element = element
        self.mesh_cache = {}
        self.missing = set()

    def resolve(self, uri, base_dir):
        if '://' in uri and not uri.startswith(('model://', 'file://')):
            return None  # e.g. a Gazebo Fuel URL: not available offline
        if uri.startswith('model://'):
            rest = uri[len('model://'):]
            for p in self.model_paths:
                cand = os.path.join(p, rest)
                if os.path.exists(cand):
                    return cand
            return None
        if uri.startswith('file://'):
            return uri[len('file://'):]
        return os.path.join(base_dir, uri)

    def mesh(self, path):
        if path not in self.mesh_cache:
            ext = path.rsplit('.', 1)[-1].lower()
            try:
                if ext == 'obj':
                    self.mesh_cache[path] = load_obj(path)
                elif ext == 'dae':
                    self.mesh_cache[path] = load_dae(path)
                elif ext == 'glb':
                    self.mesh_cache[path] = load_glb(path)
                else:
                    self.missing.add(path)
                    self.mesh_cache[path] = np.zeros((0, 3, 3))
            except (ET.ParseError, ValueError, StopIteration, OSError, AttributeError, IndexError,
                    KeyError):
                self.missing.add(path)
                self.mesh_cache[path] = np.zeros((0, 3, 3))
        return self.mesh_cache[path]

    def geometry(self, geom, base_dir):
        """(triangles, convex) of a collision geometry, or (None, False)."""
        if geom is None:
            return None, False
        box = geom.find('box')
        if box is not None:
            return box_triangles([float(x) for x in box.find('size').text.split()]), True
        cyl = geom.find('cylinder')
        if cyl is not None:
            return cylinder_triangles(
                float(cyl.find('radius').text), float(cyl.find('length').text)), True
        sph = geom.find('sphere')
        if sph is not None:
            return sphere_triangles(float(sph.find('radius').text)), True
        mesh = geom.find('mesh')
        if mesh is not None:
            path = self.resolve(mesh.find('uri').text.strip(), base_dir)
            if path is None or not os.path.exists(path):
                self.missing.add(mesh.find('uri').text.strip())
                return None, False
            tris = self.mesh(path)
            sc = mesh.find('scale')
            if sc is not None:
                tris = tris * np.array([float(x) for x in sc.text.split()])
            return tris, False
        return None, False  # plane, heightmap...: not an obstacle for a 2D map

    def model(self, model, m, base_dir, out):
        m = m @ child_pose(model)
        for link in model.findall('link'):
            lm = m @ child_pose(link)
            for col in link.findall(self.element):
                tris, convex = self.geometry(col.find('geometry'), base_dir)
                if tris is not None and len(tris):
                    out.append((transform(tris, lm @ child_pose(col)), convex))
        for nested in model.findall('model'):
            self.model(nested, m, base_dir, out)
        for inc in model.findall('include'):
            self.include(inc, m, base_dir, out)

    def include(self, inc, m, base_dir, out):
        path = self.resolve(inc.find('uri').text.strip(), base_dir)
        if path is None:
            self.missing.add(inc.find('uri').text.strip())
            return
        sdf = os.path.join(path, 'model.sdf') if os.path.isdir(path) else path
        model = _strip_ns(ET.parse(sdf).getroot()).find('model')
        # An <include> pose replaces the included model's own pose.
        pose = inc.find('pose')
        if pose is not None:
            mp = m @ pose_matrix(pose.text)
            own = model.find('pose')
            if own is not None:
                model.remove(own)
        else:
            mp = m
        self.model(model, mp, os.path.dirname(sdf), out)


# -- rasterizing ---------------------------------------------------------------

def clip_band(poly, zmin, zmax):
    """Sutherland-Hodgman clip of a 3D polygon to zmin <= z <= zmax."""
    for keep_above, z0 in ((True, zmin), (False, zmax)):
        def inside(p):
            return p[2] >= z0 if keep_above else p[2] <= z0
        res = []
        for i in range(len(poly)):
            a, b = poly[i], poly[(i + 1) % len(poly)]
            if inside(a):
                res.append(a)
            if inside(a) != inside(b):
                t = (z0 - a[2]) / (b[2] - a[2])
                res.append(a + t * (b - a))
        poly = res
        if not poly:
            return poly
    return poly


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument('world', help='SDF world file')
    ap.add_argument('--model-path', action='append', default=[],
                    help='directory with models, for model:// URIs (repeatable)')
    ap.add_argument('--center', type=float, nargs=2, required=True,
                    help='world x y that becomes the map frame origin')
    ap.add_argument('--size', type=float, nargs=2, required=True,
                    help='map width and height in meters, centered on --center')
    ap.add_argument('--resolution', type=float, default=0.2)
    ap.add_argument('--zmin', type=float, default=0.05, help='lowest height that blocks (m)')
    ap.add_argument('--zmax', type=float, default=0.5, help='highest height that blocks (m)')
    ap.add_argument('--source', choices=['collision', 'visual'], default='collision',
                    help='geometry to use: collision (what stops the robot) or visual '
                    '(what a GPU lidar sees)')
    ap.add_argument('--inside', type=float, nargs=4, metavar=('XMIN', 'YMIN', 'XMAX', 'YMAX'),
                    help='world rectangle the robots must stay in: everything outside it is '
                    'marked occupied (closes doors to other zones)')
    ap.add_argument('--output', required=True, help='output path without extension')
    args = ap.parse_args()

    world = World(args.model_path, args.source)
    root = _strip_ns(ET.parse(args.world).getroot())
    w = root.find('world')
    tris = []
    base = os.path.dirname(os.path.abspath(args.world))
    for model in w.findall('model'):
        world.model(model, np.eye(4), base, tris)
    for inc in w.findall('include'):
        world.include(inc, np.eye(4), base, tris)
    objects = tris

    cx, cy = args.center
    width, height = args.size
    res = args.resolution
    nx, ny = int(round(width / res)), int(round(height / res))
    x0, y0 = cx - width / 2.0, cy - height / 2.0

    def to_pixels(poly):
        pts = np.array([[(p[0] - x0) / res, (y0 + height - p[1]) / res] for p in poly])
        return np.round(pts * 16).astype(np.int32)  # 4 bits of subpixel precision

    def draw(pts):
        if len(pts) > 2 and cv2.contourArea(pts.astype(np.float32)) >= 16 * 16 * 0.25:
            cv2.fillPoly(occupied, [pts], 255, cv2.LINE_8, shift=4)
        cv2.polylines(occupied, [pts], True, 255, 1, cv2.LINE_8, shift=4)

    occupied = np.zeros((ny, nx), dtype=np.uint8)
    used = total = 0
    for tris, convex in objects:
        total += len(tris)
        # Only triangles that can touch the map area and the height band.
        lo, hi = tris.min(axis=1), tris.max(axis=1)
        keep = (hi[:, 2] >= args.zmin) & (lo[:, 2] <= args.zmax) & \
            (hi[:, 0] >= x0) & (lo[:, 0] <= x0 + width) & \
            (hi[:, 1] >= y0) & (lo[:, 1] <= y0 + height)
        if not keep.any():
            continue
        used += int(keep.sum())
        polys = [clip_band(list(tri), args.zmin, args.zmax) for tri in tris[keep]]
        polys = [p for p in polys if len(p) >= 2]
        if convex:
            # A solid convex shape (box, cylinder) blocks its whole slice, not
            # just the faces that cross the band.
            pts = to_pixels([p for poly in polys for p in poly])
            draw(cv2.convexHull(pts).reshape(-1, 2))
        else:
            for poly in polys:
                draw(to_pixels(poly))

    if args.inside:
        xmin, ymin, xmax, ymax = args.inside
        c0, c1 = int(math.floor((xmin - x0) / res)), int(math.ceil((xmax - x0) / res))
        r0 = int(math.floor((y0 + height - ymax) / res))
        r1 = int(math.ceil((y0 + height - ymin) / res))
        outside = np.ones_like(occupied, dtype=bool)
        outside[max(r0, 0):min(r1, ny), max(c0, 0):min(c1, nx)] = False
        occupied[outside] = 255

    image = np.where(occupied > 0, 0, 254).astype(np.uint8)
    cv2.imwrite(args.output + '.pgm', image)
    name = os.path.basename(args.output)
    with open(args.output + '.yaml', 'w') as f:
        f.write(
            f'# Generated by sdf_to_costmap.py from {os.path.basename(args.world)}:\n'
            f'# {args.source} geometry between z = {args.zmin} and {args.zmax} m, '
            f'world ({cx}, {cy}) at\n'
            f'# the origin of this map frame.\n'
            f'image: {name}.pgm\nmode: trinary\nresolution: {res}\n'
            f'origin: [{-width / 2.0}, {-height / 2.0}, 0.0]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    print(f'{nx}x{ny} cells, {int((occupied > 0).sum())} occupied, '
          f'{used} of {total} triangles in the area and band')
    if world.missing:
        print('not found or not loaded:', *sorted(world.missing)[:20], sep='\n  ')


if __name__ == '__main__':
    main()
