#!/usr/bin/env python3
"""Write print-ready STLs (cad/stl_print/) rotated so the up_axis chosen in print_report.json points +Z,
then dropped onto the bed and centred. The STLs in cad/stl/ stay in the robot's frame."""
import os, json, numpy as np, trimesh
from textmesh import write_ascii_stl
HERE = os.path.dirname(os.path.abspath(__file__))
rep = json.load(open(os.path.join(HERE, 'reports', 'print_report.json')))
out = os.path.join(HERE, 'stl_print'); os.makedirs(out, exist_ok=True)
AX = {'+x': [1, 0, 0], '-x': [-1, 0, 0], '+y': [0, 1, 0], '-y': [0, -1, 0], '+z': [0, 0, 1], '-z': [0, 0, -1]}
for name, r in rep.items():
    m = trimesh.load(os.path.join(HERE, 'stl', name + '.stl'))
    R = trimesh.geometry.align_vectors(AX[r['up_axis']], [0, 0, 1])
    m.apply_transform(R)
    b = m.bounds; m.apply_translation([-(b[0][0] + b[1][0]) / 2, -(b[0][1] + b[1][1]) / 2, -b[0][2]])
    write_ascii_stl(m, os.path.join(out, name + '.stl'), name)
    print(name, np.round(m.extents, 1))
