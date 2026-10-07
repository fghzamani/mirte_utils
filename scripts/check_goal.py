#!/usr/bin/env python3
"""Tell me whether a goal is reachable BEFORE sending it to Nav2.

Nav2's "failed to create plan, no valid path found" almost always means the goal
sits in a free pocket that is not connected to the robot's pocket through
*mapped* free space (unknown is not traversable because allow_unknown is false).
This script answers that offline, from the same map and footprint Nav2 uses, so
you stop guessing goals in RViz.

It reads the ACTIVE map and the ACTIVE robot shape straight out of
demo_params.yaml, so it always matches what the running stack believes.

Usage (run in the container; needs numpy+scipy):
  python3 scripts/check_goal.py --robot 0.22 1.78
  python3 scripts/check_goal.py --robot 0.22 1.78 --goal 1.10 3.07
  python3 scripts/check_goal.py --robot 0.22 1.78 -n 12
"""
import argparse
import ast
import math
import os
import sys

import numpy as np
import yaml
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS = os.path.join(HERE, '..', 'src', 'mirte_navigation', 'params', 'demo_params.yaml')


def robot_extent(params, override=None):
    """Return (margin, label) from the ACTIVE costmap shape.

    `override` replaces the active footprint with a polygon literal, to ask
    "would this goal be reachable with a different footprint?" -- the carry
    polygon's circumscribed radius decides how much of the map is passable at
    all, so it is usually the first thing worth varying.
    """
    r = params['global_costmap']['global_costmap']['ros__parameters']
    fp = override or r.get('footprint')
    if fp:
        pts = ast.literal_eval(fp)
        circ = max(math.hypot(x, y) for x, y in pts)
        insc = min(math.hypot(x, y) for x, y in pts)
        return circ, f'carry polygon (circumscribed {circ:.3f} m, inscribed {insc:.3f} m)'
    rr = float(r['robot_radius'])
    return rr, f'circular robot_radius {rr:.3f} m'


def load_map(params):
    mp = params['map_server']['ros__parameters']['yaml_filename']
    with open(mp) as f:
        m = yaml.safe_load(f)
    img_path = os.path.join(os.path.dirname(mp), m['image'])
    with open(img_path, 'rb') as f:
        assert f.readline().strip() == b'P5', 'expected binary PGM'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()
        img = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    img = np.flipud(img)                      # ROS: row 0 = min y
    res = float(m['resolution'])
    ox, oy = float(m['origin'][0]), float(m['origin'][1])
    # trinary: occupancy = (255-v)/255 ; free if occ < free_thresh
    free_thresh = float(m['free_thresh'])
    occ = (255.0 - img.astype(float)) / 255.0
    free = occ < free_thresh
    return free, res, ox, oy, os.path.basename(mp), free_thresh, img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--robot', nargs=2, type=float, required=True, metavar=('X', 'Y'),
                    help="robot pose in map frame (from RViz, or bt_navigator's "
                         "'Begin navigating from current location')")
    ap.add_argument('--goal', nargs=2, type=float, metavar=('X', 'Y'),
                    help='candidate goal to test')
    ap.add_argument('-n', type=int, default=8, help='how many reachable goals to suggest')
    ap.add_argument('--footprint', metavar='POLYGON',
                    help='override the active footprint, e.g. '
                         '"[[0.14,0.21],[0.14,-0.21],[-0.14,-0.21],[-0.14,0.21]]"')
    a = ap.parse_args()

    params = yaml.safe_load(open(PARAMS))
    margin, shape_label = robot_extent(params, a.footprint)
    free, res, ox, oy, mapname, fthr, img = load_map(params)
    h, w = free.shape

    print(f'map            : {mapname}  ({w}x{h}, res {res}, origin [{ox}, {oy}])')
    print(f'free_thresh    : {fthr}' + ('' if fthr < 0.196001 else
          '   <-- WARNING: >0.196 makes UNKNOWN read as FREE; set 0.196'))
    print(f'robot shape    : {shape_label}')
    print(f'known-free area: {int(free.sum()) * res * res:.1f} m2')

    dist = ndimage.distance_transform_edt(free) * res
    passable = (dist >= margin) & free
    lbl, n = ndimage.label(passable)

    def cell(x, y):
        return int((x - ox) / res), int((y - oy) / res)

    rx, ry = a.robot
    cx, cy = cell(rx, ry)
    if not (0 <= cx < w and 0 <= cy < h):
        print(f'\nERROR: robot ({rx}, {ry}) is outside the map.')
        return 1
    comp = lbl[cy, cx]
    print(f'\nrobot ({rx:.2f}, {ry:.2f}): clearance {dist[cy, cx]:.2f} m, '
          f'component {comp if comp else "NOT PASSABLE"}')
    if comp == 0:
        print('  The robot itself does not fit here at this margin -> Nav2 will report')
        print('  "Starting point in lethal space". Move the robot to open space,')
        print('  or stow the arm (circular footprint) and retry.')
        return 1

    reach = (lbl == comp)
    print(f'  reachable region: {int(reach.sum()) * res * res:.1f} m2 '
          f'(of {n} passable components in the map)')

    if a.goal:
        gx, gy = a.goal
        ccx, ccy = cell(gx, gy)
        inb = 0 <= ccx < w and 0 <= ccy < h
        print(f'\ngoal ({gx:.2f}, {gy:.2f}):')
        if not inb:
            print('  OUTSIDE the map -> will be rejected.')
        else:
            print(f'  clearance {dist[ccy, ccx]:.2f} m (need >= {margin:.3f})')
            if not free[ccy, ccx]:
                print('  NOT known-free (occupied or unmapped) -> no valid path.')
            elif lbl[ccy, ccx] == 0:
                print('  too tight for the robot -> no valid pose there.')
            elif lbl[ccy, ccx] != comp:
                print('  REACHABLE-looking but in a DIFFERENT component than the robot:')
                print('  the two areas are separated by unmapped/blocked space, so with')
                print('  allow_unknown:false there is no legal path. THIS is why Nav2')
                print('  rejects it. Map the corridor between them, or pick a goal below.')
            else:
                print('  OK -> reachable from the robot. Safe to send.')

    # suggest well-spread reachable goals
    ys, xs = np.nonzero(reach)
    pts = np.stack([ox + xs * res, oy + ys * res], 1)
    clr = dist[ys, xs]
    good = np.nonzero(clr >= margin + 0.03)[0]
    if len(good) == 0:
        good = np.arange(len(pts))
    # farthest-point sampling for spread
    sel = [int(good[np.argmax(clr[good])])]
    for _ in range(min(a.n, len(good)) - 1):
        d = np.min(np.linalg.norm(pts[good][:, None, :] - pts[sel][None, :, :], axis=2), axis=1)
        sel.append(int(good[int(np.argmax(d))]))
    print(f'\nreachable goals you can send (clearance, distance from robot):')
    for i in sel:
        d = math.hypot(pts[i][0] - rx, pts[i][1] - ry)
        print(f'  ({pts[i][0]:6.2f}, {pts[i][1]:6.2f})   clearance {clr[i]:.2f} m   {d:.2f} m away')
    return 0


if __name__ == '__main__':
    sys.exit(main())
