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
Build an initial routes file (easynav_routes_maps_manager format) for a rack warehouse map.

Racks are the tall, narrow occupied blocks of the map. One route runs along
each aisle (the middle of the gap between two neighbouring racks of a block),
two parallel ones along the widest (central) aisle, one along each edge of the
rack area, and the cross corridors above, between and below the rack blocks
join them all. Spurs leave the bottom corridor towards each group of objects
below it (e.g. packages at the docks). Every segment is checked to be free
with a clearance margin.
"""

import argparse
import math

import cv2
import numpy as np
import yaml


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument('map_yaml', help='map_server YAML of the map')
    ap.add_argument('--output', required=True, help='routes YAML to write')
    ap.add_argument('--also-free-in', action='append', default=[],
                    help='another map YAML (same size and origin) every route must '
                    'also be free in')
    ap.add_argument('--min-rack-length', type=float, default=20.0, help='m')
    ap.add_argument('--min-aisle-width', type=float, default=1.2, help='m')
    ap.add_argument('--edge-offset', type=float, default=1.5,
                    help='distance (m) from the outermost racks to the edge routes')
    ap.add_argument('--spur-group-gap', type=float, default=3.0,
                    help='objects closer than this (m) form one group for a dock spur')
    ap.add_argument('--min-spur-group', type=float, default=3.0,
                    help='minimum length (m) of a group of objects to get a spur')
    ap.add_argument('--clearance', type=float, default=0.3,
                    help='free space required around a route (m)')
    args = ap.parse_args()

    with open(args.map_yaml) as f:
        meta = yaml.safe_load(f)
    image = cv2.imread(
        str(__import__('pathlib').Path(args.map_yaml).parent / meta['image']),
        cv2.IMREAD_GRAYSCALE)
    res = float(meta['resolution'])
    ox, oy = meta['origin'][:2]
    rows = image.shape[0]
    occupied = (image < 128).astype(np.uint8)

    def to_world(col, row):
        return ox + (col + 0.5) * res, oy + (rows - row - 0.5) * res

    def to_cell(x, y):
        return int((x - ox) / res), int(rows - (y - oy) / res)

    # Racks: tall, narrow connected occupied blocks.
    count, _, stats, _ = cv2.connectedComponentsWithStats(occupied, connectivity=4)
    racks = []
    for i in range(1, count):
        cx, cy, w, h, _ = stats[i]
        if h * res >= args.min_rack_length and w < h / 3:
            x0, y_top = to_world(cx, cy)
            x1, y_bottom = to_world(cx + w - 1, cy + h - 1)
            racks.append((x0 - res / 2, x1 + res / 2, y_bottom - res / 2, y_top + res / 2))
    if not racks:
        raise SystemExit('no racks found')

    # Two (or more) blocks of racks, split where their vertical extents don't overlap.
    racks.sort(key=lambda r: -r[3])
    blocks = []
    for rack in racks:
        for block in blocks:
            if rack[2] < max(b[3] for b in block) and rack[3] > min(b[2] for b in block):
                block.append(rack)
                break
        else:
            blocks.append([rack])
    blocks.sort(key=lambda b: -max(r[3] for r in b))

    blocked = occupied.copy()
    for other in args.also_free_in:
        with open(other) as f:
            other_meta = yaml.safe_load(f)
        other_image = cv2.imread(
            str(__import__('pathlib').Path(other).parent / other_meta['image']),
            cv2.IMREAD_GRAYSCALE)
        if other_image.shape != image.shape:
            raise SystemExit(f'{other}: not the same size as {args.map_yaml}')
        blocked |= (other_image < 128).astype(np.uint8)
    clearance = cv2.dilate(
        blocked, np.ones((2 * int(math.ceil(args.clearance / res)) + 1,) * 2, np.uint8))

    def free(a, b):
        n = int(math.hypot(b[0] - a[0], b[1] - a[1]) / (res / 2)) + 1
        for t in np.linspace(0.0, 1.0, n):
            c, r = to_cell(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
            if not (0 <= r < clearance.shape[0] and 0 <= c < clearance.shape[1]) or \
                    clearance[r, c]:
                return False
        return True

    def free_y(x, y_from, y_to, step):
        """Last y from y_from towards y_to (exclusive of obstacles) that is free."""
        y = y_from
        while (y_to - y) * step > 0 and free((x, y), (x, y + step)):
            y += step
        return y

    def free_x(y, x_from, x_to, step):
        """Last x from x_from towards x_to (exclusive of obstacles) that is free."""
        x = x_from
        while (x_to - x) * step > 0 and free((x, y), (x + step, y)):
            x += step
        return x

    # Vertical routes, per block: the middle of the gap between neighbouring
    # racks; two parallel ones in the widest gap (the central aisle), at a
    # quarter and three quarters of its width; and the edges, beside the
    # outermost racks (at most halfway to the wall).
    aisles = []  # (block, x, kind)
    for bi, block in enumerate(blocks):
        block.sort(key=lambda r: r[0])
        gaps = [(right[0] - left[1], left, right) for left, right in zip(block, block[1:])]
        widest = max(gaps, key=lambda g: g[0])
        for gap, left, right in gaps:
            if gap < args.min_aisle_width:
                continue
            if gap is widest[0] and left is widest[1]:
                for frac in (0.25, 0.75):
                    aisles.append((bi, round(left[1] + frac * gap, 2), 'central'))
            else:
                aisles.append((bi, round((left[1] + right[0]) / 2.0, 2), 'aisle'))
        ys = np.linspace(min(r[2] for r in block) + 1, max(r[3] for r in block) - 1, 9)
        x_min, x_max = block[0][0], max(r[1] for r in block)
        wall_l = float(np.median([free_x(y, x_min - 0.5, x_min - 30, -0.2) for y in ys]))
        wall_r = float(np.median([free_x(y, x_max + 0.5, x_max + 30, 0.2) for y in ys]))
        # Near the racks: things are often stored along the walls.
        left = x_min - min(args.edge_offset, (x_min - wall_l) / 2.0)
        right = x_max + min(args.edge_offset, (wall_r - x_max) / 2.0)
        aisles.append((bi, round(left, 2), 'edge'))
        aisles.append((bi, round(right, 2), 'edge'))

    # Cross corridors: above the top block, between blocks, below the bottom
    # one. Above and below, halfway to whatever is next, taking the median
    # over every aisle so a single object doesn't decide it.
    tops = [max(r[3] for r in b) for b in blocks]
    bottoms = [min(r[2] for r in b) for b in blocks]
    above = [free_y(x, tops[0] + 0.5, tops[0] + 30, 0.2)
             for bi, x, kind in aisles if bi == 0 and kind == 'aisle']
    below = [free_y(x, bottoms[-1] - 0.5, bottoms[-1] - 30, -0.2)
             for bi, x, kind in aisles if bi == len(blocks) - 1 and kind == 'aisle']
    corridor_y = [(tops[0] + float(np.median(above))) / 2.0]
    for i in range(len(blocks) - 1):
        corridor_y.append((bottoms[i] + tops[i + 1]) / 2.0)
    corridor_y.append((bottoms[-1] + float(np.median(below))) / 2.0)

    routes = []
    corridor_stops = {i: set() for i in range(len(corridor_y))}
    for bi, x, kind in aisles:
        a, b = (x, corridor_y[bi + 1]), (x, corridor_y[bi])
        if free(a, b):
            routes.append((f'{kind}_{bi}_{len(routes)}', a, b))
            corridor_stops[bi].add(x)
            corridor_stops[bi + 1].add(x)

    # Spurs from the bottom corridor to each group of objects below it (e.g.
    # the rows of packages at the loading docks), along its free side.
    bottom_y = corridor_y[-1]
    count, _, stats, _ = cv2.connectedComponentsWithStats(blocked, connectivity=8)
    below_items = []
    for i in range(1, count):
        cx, cy, w, h, _ = stats[i]
        x0, y_top = to_world(cx, cy)
        x1, y_bot = to_world(cx + w - 1, cy + h - 1)
        if y_top < bottom_y - 1.5 and w * res < 30 and h * res < 30:
            below_items.append([x0, x1, y_bot, y_top])
    below_items.sort()
    groups = []
    for item in below_items:
        if groups and item[0] - groups[-1][1] < args.spur_group_gap:
            g = groups[-1]
            g[1], g[2], g[3] = max(g[1], item[1]), min(g[2], item[2]), max(g[3], item[3])
        else:
            groups.append(list(item))
    for gi, (x0, x1, y0, y1) in enumerate(groups):
        if y1 - y0 < args.min_spur_group:
            continue
        for x in (x0 - 1.0, x1 + 1.0):
            x = round(x, 2)
            a, b = (x, bottom_y), (x, round((y0 + y1) / 2.0, 2))
            if free(a, b):
                routes.append((f'dock_{gi}', a, b))
                corridor_stops[len(corridor_y) - 1].add(x)
                break

    for ci, y in enumerate(corridor_y):
        xs = sorted(corridor_stops[ci])
        for i, (xa, xb) in enumerate(zip(xs, xs[1:])):
            if free((xa, y), (xb, y)):
                routes.append((f'corridor_{ci}_{i}', (xa, y), (xb, y)))

    out = {'routes': [name for name, _, _ in routes]}
    for name, a, b in routes:
        yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        q = {'qx': 0.0, 'qy': 0.0, 'qz': round(math.sin(yaw / 2), 6),
             'qw': round(math.cos(yaw / 2), 6)}
        out[name] = {
            'start': {'x': round(float(a[0]), 2), 'y': round(float(a[1]), 2), 'z': 0.0, **q},
            'end': {'x': round(float(b[0]), 2), 'y': round(float(b[1]), 2), 'z': 0.0, **q},
        }
    with open(args.output, 'w') as f:
        source = args.map_yaml.rsplit('/', 1)[-1]
        f.write(f'# Generated by aisle_routes.py from {source}: one route\n'
                '# per rack aisle, two along the central aisle, the edges of the rack area,\n'
                '# the cross corridors that join them and spurs to the docks (map frame).\n')
        yaml.safe_dump(out, f, sort_keys=False, default_flow_style=None)
    print(f'{len(racks)} racks in {len(blocks)} blocks, corridors at y = '
          f'{[round(float(y), 2) for y in corridor_y]}, {len(routes)} routes')


if __name__ == '__main__':
    main()
