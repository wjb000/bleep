#!/usr/bin/env python3
"""Build everything from cad/bleep.py:
  cad/step/*.step, cad/stl/*.stl            printed parts (+ full assembly STEP with the real STS3215 model)
  cad/reports/*.json                        interference + range-of-motion clearance, print analysis, mass properties
  sim/bleep.urdf, sim/bleep.xml, sim/meshes link meshes, masses and inertias for URDF / MuJoCo
  site/assets/bleep.glb, robot_info.json    the viewer model (link hierarchy so the viewer can replay joint motion)
Run: python cad/export.py   (needs cadquery, trimesh, manifold3d; ~1-2 min)"""
import os, sys, json, math, time
import numpy as np, trimesh, cadquery as cq
import manifold3d as mf
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import bleep
from lib import SRV_X0, SRV_X1, SRV_HY, SRV_Z0, SRV_Z1

PLA = 1.24e-3            # g/mm^3
FILL = 0.70              # effective solid fraction of the printed walls (3 perimeters + 15 % gyroid in 2.4-5 mm walls)
SERVO_G = 55.0 + 3.0     # STS3215 55 g (datasheet) + horn screws / case screws
BED = 256.0

def d(*p):
    q = os.path.join(ROOT, *p); os.makedirs(q, exist_ok=True); return q

def tess(solid, tol=0.15):
    v, f = solid.tessellate(tol, 0.3)
    m = trimesh.Trimesh(np.array([[p.x, p.y, p.z] for p in v]), np.array(f), process=True)
    m.merge_vertices(); trimesh.repair.fix_normals(m)
    return m

def to_manifold(m):
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(m.vertices, np.float32), tri_verts=np.asarray(m.faces, np.uint32)))

def box_mesh(b):
    x0, x1, y0, y1, z0, z1 = b
    m = trimesh.creation.box(extents=(x1 - x0, y1 - y0, z1 - z0)); m.apply_translation(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)); return m

def cyl_mesh(p0, p1, r, sections=24):
    return trimesh.creation.cylinder(radius=r, segment=[p0, p1], sections=sections)

# ------------------------------------------------------------------ electronics (bought parts other than servos)
def electronics():
    B, N = bleep, bleep.NECK
    zh = B.Z_N + B.GAP + B.PT   # top of the head base plate
    E = []
    def add(key, name, mesh, mass_g, link, kind, spec, label=None):
        E.append(dict(key=key, name=name, mesh=mesh, mass=mass_g, link=link, kind=kind, spec=spec, label=label))
    for sy in (-1, 1):
        add(f'cell_{"l" if sy > 0 else "r"}', 'LiFePO4 18650 cell (3.2 V 1500 mAh)',
            cyl_mesh((B.CELL_X[0], sy * B.CELL_Y, B.CELL_Z), (B.CELL_X[1], sy * B.CELL_Y, B.CELL_Z), 9.25), 40.0, 'base', 'modeled',
            '2S pack = 6.4 V nominal / 7.3 V full: inside the STS3215-C001 4-7.4 V rating with no regulator. 2 x 1.5 Ah ~ 9.6 Wh.',
            label=sy > 0)
    add('bms', '2S LiFePO4 BMS + balance board', box_mesh((-30, 2, -10, 10, 234.6, 237.4)), 5.0, 'base', 'envelope',
        'Over-charge / over-discharge / short protection; charged from a 7.3 V LiFePO4 charger through the rear jack.', True)
    add('adapter', 'Waveshare Bus Servo Adapter (A)', box_mesh((14.0, 24.0, -21, 21, 250.0, 272.0)), 8.0, 'base', 'envelope',
        'UART (TX/RX) from the XIAO -> half-duplex TTL bus for all 11 servos; passes battery power to the servo bus.', True)
    add('imu', 'MPU-6050 IMU (GY-521)', box_mesh((24.6, 27.4, -10.5, 10.5, 236.0, 252.0)), 2.0, 'base', 'envelope',
        '6-axis accel + gyro at 200 Hz on I2C; gives the policy its gravity vector and angular rate. Rigid to the body.', True)
    add('switch', 'Power switch + 5.5 x 2.1 charge jack', box_mesh((-50.5, -44.0, -20, 18, B.Z_BODY0 + 14, B.Z_BODY0 + 24)), 6.0, 'base', 'envelope',
        'Rear service panel: slide switch cuts the pack; DC jack for the 7.3 V charger.', True)
    add('harness', 'Wiring harness (servo daisy chain, neck loop)', box_mesh((-35, 10, -30, 30, 266.0, 268.0)), 15.0, 'base', 'envelope', 'Mass allowance only.', False)
    # head electronics
    xm = np.load(os.path.join(HERE, '.cache', 'xiao_dec.npz')) if os.path.exists(os.path.join(HERE, '.cache', 'xiao_dec.npz')) else None
    T = np.eye(4); T[:3, :3] = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0.]]).T @ np.eye(3)
    if xm is not None:
        xmesh = trimesh.Trimesh(xm['v'], xm['f'], process=False)
        pc, nrm = B.xiao_pose()
        M = trimesh.transformations.rotation_matrix(np.radians(B.CAM_YAW - 90.0), (0, 0, 1))   # STEP +y (component side) -> camera direction
        xmesh.apply_transform(M)
        c = xmesh.bounds.mean(0); xmesh.apply_translation(pc - c)
        kind = 'real'
    else:
        xmesh = box_mesh((16, 28, 13, 31, zh + 15, zh + 37)); kind = 'envelope'
    add('xiao', 'Seeed XIAO ESP32S3 Sense', xmesh, 10.0, 'head', kind,
        'The brain: dual-core ESP32-S3 @240 MHz, 8 MB PSRAM, Wi-Fi/BLE, runs the walking policy at 50 Hz. Sense board adds the camera connector, PDM microphone and microSD.', True)
    nrm = np.array([math.cos(math.radians(B.CAM_YAW)), math.sin(math.radians(B.CAM_YAW)), 0.0])
    cm = trimesh.creation.box(extents=(6.0, 8.5, 8.5)); cm.apply_transform(trimesh.transformations.rotation_matrix(math.radians(B.CAM_YAW), (0, 0, 1)))
    cm.apply_translation(nrm * (B.HEAD_R - 7.5) + np.array([0, 0, B.CAM_Z]))
    add('camera', 'Camera eye (OV3660/OV5640 on the XIAO Sense)', cm, 2.0, 'head', 'envelope',
        'Looks out through the lens port; streams MJPEG over Wi-Fi and feeds on-board motion / face tracking for the dome.', True)
    add('lcd', '1.28" round GC9A01 LCD eye', cyl_mesh((B.HEAD_R - 11.0, 0, B.EYE_Z), (B.HEAD_R - 6.5, 0, B.EYE_Z), 19.2, 48), 7.0, 'head', 'envelope',
        '240 x 240 round IPS on SPI: animated pupil, blinks, moods, battery and status icons.', True)
    add('buck', 'Pololu D36V6F5 5 V buck', box_mesh((-20, -7.3, -10, -2.4, zh + 2, zh + 5)), 1.0, 'head', 'envelope',
        '6.4 V pack -> 5 V / 600 mA for the XIAO, camera and LCD (peak ~0.45 A with the camera streaming).', True)
    add('buzzer', 'Passive piezo buzzer (12 mm)', cyl_mesh((-24, 14, zh + 1), (-24, 14, zh + 9), 6.0), 2.0, 'head', 'envelope',
        'Droid chirps and beeps (PWM tones), no amplifier needed.', True)
    ztop = B.Z_N + B.GAP + B.PT - 0.5 + B.HEAD_CYL + B.HEAD_DOME - 2.6
    add('touch', 'Copper-tape touch pad (inside the dome)', cyl_mesh((0, 0, ztop - 0.3), (0, 0, ztop), 16.0, 32), 1.0, 'head', 'envelope',
        'ESP32-S3 capacitive touch through 2 mm of PLA: pat its head to wake it, hold to sit.', True)
    return E

# ------------------------------------------------------------------ mass property helpers
def inertia_box_world(R, ext, mass):
    a, b, c = ext
    I = mass / 12.0 * np.diag([b * b + c * c, a * a + c * c, a * a + b * b])
    return R @ I @ R.T

def combine(elems):
    M = sum(e[0] for e in elems)
    c = sum(e[0] * np.asarray(e[1]) for e in elems) / M
    I = np.zeros((3, 3))
    for m, com, Ic in elems:
        r = np.asarray(com) - c
        I += Ic + m * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
    return M, c, I

def mesh_props(m, mass):
    m = m.copy()
    if not m.is_watertight:
        h = m.convex_hull; com = h.center_mass; I = h.moment_inertia / h.volume * mass if h.volume > 0 else np.zeros((3, 3))
        return com, I
    m.density = 1.0
    com = m.center_mass
    I = m.moment_inertia / m.mass * mass
    return com, I

# ------------------------------------------------------------------ print analysis
UPS = {'+z': np.eye(3), '-z': np.diag([1, -1, -1.]), '+x': np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0.]]),
       '-x': np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0.]]), '+y': np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0.]]),
       '-y': np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0.]])}

def orient(mesh):
    rows = []
    for k, R in UPS.items():
        m = mesh.copy(); M = np.eye(4); M[:3, :3] = R; m.apply_transform(M); m.apply_translation((0, 0, -m.bounds[0][2]))
        n, a, cz = m.face_normals, m.area_faces, m.triangles_center[:, 2]
        over = a[(n[:, 2] < -0.7071) & (cz > 0.3)].sum()
        bed = a[(n[:, 2] < -0.99) & (cz < 0.3)].sum()
        ext = m.extents
        fits = ext[0] <= BED and ext[1] <= BED and ext[2] <= BED
        rows.append((over - 0.02 * bed + 0.02 * ext[2] + (1e6 if not fits else 0), k, over, bed, ext))
    rows.sort(key=lambda r: r[0])
    s, k, over, bed, ext = rows[0]
    return dict(up_axis=k, overhang_mm2=round(float(over), 0), overhang_pct=round(float(100 * over / mesh.area), 1),
                bed_contact_mm2=round(float(bed), 0), extents_mm=[round(float(e), 1) for e in ext], fits_256_bed=bool(max(ext) <= BED))

# ------------------------------------------------------------------ main
def main():
    t0 = time.time()
    parts, servos = bleep.build_parts()
    J = bleep.joints()
    origin = {'base': np.zeros(3)}
    parent = {}
    for jn, (p, c, o, a) in J.items():
        origin[c] = np.array(o, float); parent[c] = (p, jn)
    links = ['base'] + [J[j][1] for j in J]
    print(f'built {len(parts)} parts in {time.time()-t0:.0f}s')

    # --- STEP / STL
    sd, td = d('cad', 'step'), d('cad', 'stl')
    meshes = {}
    for p in parts:
        cq.exporters.export(cq.Workplane().add(p['solid']), os.path.join(sd, p['name'] + '.step'))
        m = tess(p['solid'])
        m.export(os.path.join(td, p['name'] + '.stl'))
        meshes[p['name']] = m
    # servo meshes (real vendor CAD) placed in the world
    sv = np.load(os.path.join(HERE, '.cache', 'sts3215_dec.npz'))
    smesh = {}
    for s in servos:
        m = trimesh.Trimesh(sv['v'], sv['f'], process=False); m.apply_transform(s.step_to_world()); smesh[s.name] = m
    # assembly STEP: printed parts + real STS3215 STEP at every joint
    asm = cq.Assembly(name='bleep')
    col = {'shell': cq.Color(0.91, 0.89, 0.84), 'accent': cq.Color(0.10, 0.55, 0.60), 'dark': cq.Color(0.20, 0.22, 0.25)}
    for p in parts:
        asm.add(p['solid'], name=p['name'], color=col[p['color']])
    srv_step = cq.importers.importStep(os.path.join(HERE, 'vendor', 'STS3215_03a.step')).val()
    for s in servos:
        M = s.step_to_world()
        from OCP.gp import gp_Trsf
        tr = gp_Trsf(); tr.SetValues(*[float(x) for x in M[0]], *[float(x) for x in M[1]], *[float(x) for x in M[2]])
        asm.add(srv_step, name='STS3215_' + s.name, loc=cq.Location(tr), color=cq.Color(0.08, 0.08, 0.09))
    asm.save(os.path.join(sd, 'bleep_assembly.step'))
    print(f'STEP/STL done {time.time()-t0:.0f}s')

    # --- electronics
    E = electronics()

    # --- interference at the zero pose (printed parts vs printed parts vs servo bodies)
    rep = d('cad', 'reports')
    owner = {}
    for p in parts:
        for n in p['servos']:
            owner[n] = p['name']
    solids = {k: to_manifold(v) for k, v in meshes.items()}
    srv_env = {}
    for s in servos:
        b = s.lbox(SRV_X0, SRV_X1, -SRV_HY, SRV_HY, SRV_Z0, SRV_Z1)
        srv_env[s.name] = to_manifold(box_mesh(b))
    for e in E:
        if e['key'] in ('harness', 'touch', 'switch'):
            continue
        solids['elec:' + e['key']] = to_manifold(e['mesh'].convex_hull)
    for k, v in srv_env.items():
        solids['servo:' + k] = v
    names = list(solids)
    hits = []
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = names[i], names[j]
            if a.startswith('servo:') and b.startswith('servo:'):
                continue
            # a servo inside its own pod is the design
            if a.startswith('servo:') and owner.get(a[6:]) == b or b.startswith('servo:') and owner.get(b[6:]) == a:
                continue
            v = (solids[a] ^ solids[b]).volume()
            if v > 1.0:
                hits.append((a, b, round(v, 1)))
    print('zero-pose interferences (>1 mm3):', hits)

    # --- range-of-motion sweep: each joint at both limits and mid-way, child subtree vs everything else
    link_of = {p['name']: p['link'] for p in parts}
    link_of.update({'servo:' + s.name: next(p['link'] for p in parts if s.name in p['servos']) for s in servos})
    for e in E:
        if 'elec:' + e['key'] in solids:
            link_of['elec:' + e['key']] = e['link']
    children = {l: [] for l in links}
    for c, (p, jn) in parent.items():
        children[p].append(c)
    def subtree(l):
        out = [l]
        for c in children[l]:
            out += subtree(c)
        return out
    sweep = []
    def other_leg(n, side):
        l = link_of[n]
        return l.startswith(('l_', 'r_')) and not l.startswith(side)
    for jn, (p, c, o, a) in J.items():
        lo, hi = bleep.limit_deg(jn)
        side = jn[:2] if jn[:2] in ('l_', 'r_') else None
        sub = set(subtree(c))
        moving = [n for n in names if link_of[n] in sub]
        fixed = [n for n in names if link_of[n] not in sub]
        res = dict(joint=jn, range_deg=[lo, hi])
        for direction, end in (('neg', lo), ('pos', hi)):
            first_self, first_leg = None, None
            for ang in np.arange(0.0, end + (2.5 if end > 0 else -2.5), 2.5 if end > 0 else -2.5):
                if (end > 0 and ang > end) or (end < 0 and ang < end):
                    break
                T = trimesh.transformations.rotation_matrix(np.radians(ang), a, o)
                Tm = np.asarray(T[:3, :], np.float64)
                for n in moving:
                    mm = solids[n].transform(Tm)
                    for f in fixed:
                        if n.startswith('servo:') and f.startswith('servo:'):
                            continue
                        leg = side is not None and (other_leg(f, side) or other_leg(n, side))
                        if (leg and first_leg) or (not leg and first_self):
                            continue
                        v = (mm ^ solids[f]).volume()
                        if v > 1.0:
                            if leg:
                                first_leg = (round(float(ang), 1), n, f)
                            else:
                                first_self = (round(float(ang), 1), n, f)
                if first_self:
                    break
            res[direction + '_first_contact_self'] = first_self
            res[direction + '_first_contact_other_leg'] = first_leg
        sweep.append(res)
        print(f"  sweep {jn:14s} {lo:5.0f}..{hi:4.0f}  self-contact neg {res['neg_first_contact_self']} pos {res['pos_first_contact_self']}  other-leg neg {res['neg_first_contact_other_leg']} pos {res['pos_first_contact_other_leg']}")

    # --- mass properties per link
    LP = {}
    elems = {l: [] for l in links}
    parts_mass = {}
    for p in parts:
        m = meshes[p['name']]
        mass = abs(m.volume) * PLA * FILL
        parts_mass[p['name']] = round(mass, 1)
        com, I = mesh_props(m, mass)
        elems[p['link']].append((mass, com, I))
    for s in servos:
        l = next(p['link'] for p in parts if s.name in p['servos'])
        c = s.w((SRV_X0 + SRV_X1) / 2, 0, (SRV_Z0 + SRV_Z1) / 2)
        I = inertia_box_world(s.R, (SRV_X1 - SRV_X0, 2 * SRV_HY, SRV_Z1 - SRV_Z0), SERVO_G)
        elems[l].append((SERVO_G, c, I))
    for e in E:
        com, I = mesh_props(e['mesh'], e['mass'])
        elems[e['link']].append((e['mass'], com, I))
    for l in links:
        M, c, I = combine(elems[l])
        LP[l] = dict(mass_kg=M / 1000.0, com_world_mm=c.tolist(), com_m=((c - origin[l]) / 1000.0).tolist(), I_kgm2=(I * 1e-9).tolist())
    total = sum(v['mass_kg'] for v in LP.values())
    com_all = sum(np.array(v['com_world_mm']) * v['mass_kg'] for v in LP.values()) / total
    print(f'total mass {total*1000:.0f} g, COM {np.round(com_all,1)} mm')

    # --- print report
    pr = {}
    for p in parts:
        r = orient(meshes[p['name']])
        r['mass_g'] = parts_mass[p['name']]
        r['volume_cm3'] = round(abs(meshes[p['name']].volume) / 1000, 1)
        pr[p['name']] = r
    plastic = sum(parts_mass.values())

    bb = np.array([m.bounds for m in meshes.values()])
    height = float(bb[:, 1, 2].max()); width = float(bb[:, 1, 1].max() - bb[:, 0, 1].min()); depth = float(bb[:, 1, 0].max() - bb[:, 0, 0].min())
    summary = dict(total_mass_kg=round(total, 3), plastic_g=round(plastic, 0), height_mm=round(height, 1), width_mm=round(width, 1), depth_mm=round(depth, 1),
                   com_world_mm=np.round(com_all, 1).tolist(), n_printed_parts=len(parts), n_servos=len(servos),
                   zero_pose_interferences=hits, sweep=sweep, link_mass_g={l: round(v['mass_kg'] * 1000, 1) for l, v in LP.items()})
    json.dump(summary, open(os.path.join(rep, 'summary.json'), 'w'), indent=1)
    json.dump(pr, open(os.path.join(rep, 'print_report.json'), 'w'), indent=1)
    json.dump(LP, open(os.path.join(rep, 'mass_properties.json'), 'w'), indent=1)
    print(json.dumps({k: v for k, v in summary.items() if k not in ('sweep',)}, indent=1))

    # --- sim export (URDF + MJCF) and GLB
    import sim_export
    sim_export.write(parts, meshes, servos, smesh, E, J, origin, LP, links)
    import glb_export
    glb_export.write(parts, meshes, servos, smesh, E, J, origin, links, summary)
    print(f'all done {time.time()-t0:.0f}s')

if __name__ == '__main__':
    main()
