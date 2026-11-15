#!/usr/bin/env python3
"""Tessellate + decimate vendor STEP files into light meshes (cad/.cache/*.npz) for the GLB viewer model.
STS3215 STEP is committed (Apache-2.0, from SO-ARM100). The XIAO ESP32S3 STEP is fetched by fetch_vendor.py."""
import os, time, numpy as np, trimesh, cadquery as cq
HERE = os.path.dirname(os.path.abspath(__file__))
CACHE = os.path.join(HERE, '.cache'); os.makedirs(CACHE, exist_ok=True)

def run(name, path, tol, faces):
    if not os.path.exists(path):
        print('skip', name, '(missing', path + ')'); return
    t = time.time()
    s = cq.importers.importStep(path).val()
    v, f = s.tessellate(tol, 0.5)
    m = trimesh.Trimesh(np.array([[p.x, p.y, p.z] for p in v]), np.array(f), process=True); m.merge_vertices()
    n0 = len(m.faces)
    if n0 > faces:
        m = m.simplify_quadric_decimation(face_count=faces)
    print(f'{name}: {n0} -> {len(m.faces)} faces ({time.time()-t:.0f}s)')
    np.savez(os.path.join(CACHE, f'{name}_dec.npz'), v=m.vertices, f=m.faces)

if __name__ == '__main__':
    run('sts3215', os.path.join(HERE, 'vendor', 'STS3215_03a.step'), 0.15, 2500)
    run('xiao', os.path.join(HERE, 'vendor_fetched', 'xiao_esp32s3.step'), 0.1, 1500)
