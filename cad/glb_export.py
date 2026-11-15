"""Write site/assets/bleep.gltf (embedded buffer) (link hierarchy, mm, z-up) and site/assets/robot_info.json (labels, joints, specs)."""
import os, json, numpy as np, trimesh
import bleep
from textmesh import glb_to_embedded_gltf
from lib import SRV_X0, SRV_X1, SRV_Z0, SRV_Z1
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, 'site', 'assets')

SERVO_SPEC = ('Feetech STS3215-C001 serial bus servo (real vendor CAD): 19.5 kg.cm stall (derated to 1.65 N.m at 6.4 V), '
              '12-bit magnetic encoder, position/speed/load/voltage/temperature feedback, 55 g.')
JOINT_TXT = {'hip_yaw': 'Hip yaw: turns the leg for steering', 'hip_roll': 'Hip roll: shifts weight side to side',
             'hip_pitch': 'Hip pitch: swings the leg', 'knee': 'Knee: crouch, stand up, push off', 'ankle': 'Ankle pitch: keeps the foot flat, balance',
             'neck_yaw': 'Dome yaw: turns the head to look around'}

def write(parts, meshes, servos, smesh, E, J, origin, links, summary):
    os.makedirs(OUT, exist_ok=True)
    sc = trimesh.Scene()
    parent = {c: p for jn, (p, c, o, a) in J.items()}
    jname = {c: jn for jn, (p, c, o, a) in J.items()}
    order = ['base'] + [J[j][1] for j in J]
    for l in order:
        T = np.eye(4)
        if l != 'base':
            T[:3, 3] = origin[l] - origin[parent[l]]
        sc.graph.update(frame_from=('link:' + parent[l]) if l != 'base' else sc.graph.base_frame, frame_to='link:' + l, matrix=T)
    def put(name, mesh, link):
        m = mesh.copy(); m.apply_translation(-origin[link])
        sc.add_geometry(m, node_name=name, geom_name=name, parent_node_name='link:' + link)
    labels = []
    for p in parts:
        m = meshes[p['name']]
        if len(m.faces) > 30000:
            m = m.simplify_quadric_decimation(face_count=30000)
        put('part:' + p['name'], m, p['link'])
    for s in servos:
        link = next(p['link'] for p in parts if s.name in p['servos'])
        put('servo:' + s.name, smesh[s.name], link)
        j = s.name[2:] if s.name[:2] in ('l_', 'r_') else s.name
        if not s.name.startswith('r_'):
            c = s.w((SRV_X0 + SRV_X1) / 2, 0, (SRV_Z0 + SRV_Z1) / 2)
            labels.append(dict(key='servo:' + s.name, node='link:' + link, local=(c - origin[link]).tolist(), kind='real',
                               title=JOINT_TXT[j] + (' (x2)' if s.name.startswith('l_') else ''), spec=SERVO_SPEC, group='servo'))
    for e in E:
        put('elec:' + e['key'], e['mesh'], e['link'])
        if e.get('label'):
            c = e['mesh'].bounds.mean(0)
            labels.append(dict(key='elec:' + e['key'], node='link:' + e['link'], local=(c - origin[e['link']]).tolist(), kind=e['kind'],
                               title=e['name'] + (' (x2)' if e['key'].startswith('cell') else ''), spec=e['spec'], group='electronics'))
    sc.export(os.path.join(OUT, 'bleep.glb'))
    glb_to_embedded_gltf(os.path.join(OUT, 'bleep.glb'), os.path.join(OUT, 'bleep.gltf')); os.remove(os.path.join(OUT, 'bleep.glb'))
    info = dict(build=summary.get('build', 'v1'), summary={k: v for k, v in summary.items() if k != 'sweep'}, sweep=summary['sweep'],
                joints={jn: dict(node='link:' + c, axis=list(a), limits_deg=list(bleep.limit_deg(jn))) for jn, (p, c, o, a) in J.items()},
                parts={p['name']: dict(color=p['color'], link=p['link']) for p in parts}, labels=labels)
    json.dump(info, open(os.path.join(OUT, 'robot_info.json'), 'w'), indent=1)
    print('wrote', os.path.join(OUT, 'bleep.glb'), os.path.getsize(os.path.join(OUT, 'bleep.glb')) // 1024, 'kB')
