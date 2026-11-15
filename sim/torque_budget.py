#!/usr/bin/env python3
"""Servo sizing from the CAD masses: static joint torques in the worst everyday poses, computed exactly from the MJCF
(torque at a joint = moment of everything on the body side of it about the joint axis; the stance foot is the only
ground contact, so this holds wherever the centre of pressure is). Compared with candidate servos.
Writes sim/results/torque_budget.json. Run: python sim/torque_budget.py"""
import os, json, numpy as np, mujoco
HERE = os.path.dirname(os.path.abspath(__file__))
m = mujoco.MjModel.from_xml_path(os.path.join(HERE, 'bleep.xml')); d = mujoco.MjData(m)
G = np.array([0, 0, -9.81])
LEG = ['hip_yaw', 'hip_roll', 'hip_pitch', 'knee', 'ankle']
CANDIDATES = {   # stall torque at the voltage it would run at, mass, unit price (see docs/BOM.md / README for sources)
    'Feetech STS3215-C001 (6.4 V)': dict(stall=1.65, rated=0.42, mass_g=55, price=13.89, feedback=True),
    'Feetech STS3032 (6 V)': dict(stall=0.44, rated=0.15, mass_g=20, price=32.99, feedback=True),
    'Feetech SCS0009 (6 V)': dict(stall=0.23, rated=0.07, mass_g=13, price=9.0, feedback=True),
    'TowerPro SG90 (5 V)': dict(stall=0.18, rated=0.05, mass_g=9, price=2.5, feedback=False),
}

def pose(q, body_roll=0.0):
    d.qpos[:] = m.key('home').qpos
    d.qpos[3:7] = [np.cos(body_roll / 2), np.sin(body_roll / 2), 0, 0]
    for k, v in q.items():
        d.qpos[m.jnt_qposadr[m.joint(k).id]] = v
    mujoco.mj_forward(m, d)

def body_side_moment(jname, stance_side):
    """moment about joint axis of all mass NOT distal to the joint (distal = towards the stance foot)"""
    j = m.joint(jname).id
    b = m.jnt_bodyid[j]                      # child body of the joint (the distal side)
    distal = set()
    for i in range(m.nbody):                 # bodies in the subtree of b
        k = i
        while k > 0:
            if k == b:
                distal.add(i); break
            k = m.body_parentid[k]
    mass = 0.0; mom = np.zeros(3)
    for i in range(1, m.nbody):
        if i in distal:
            continue
        mass += m.body_mass[i]; mom += m.body_mass[i] * d.xipos[i]
    c = mom / mass
    p = d.xanchor[j]; ax = d.xaxis[j]
    # the distal side pushes on the proximal side with -(m g) through the joint; torque the servo must supply:
    return float(np.dot(np.cross(c - p, mass * G), ax))

def scenario(name, q, stance):
    pose(q)
    out = {}
    for jn in LEG:
        out[jn] = abs(body_side_moment(f'{stance}_{jn}', stance))
    return name, out

def settle(q, T=2.0):
    """double stance: both feet on the floor, so read the servo torques from a settled simulation instead"""
    dd = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, dd, m.key('home').id)
    for k, v in q.items():
        dd.qpos[m.jnt_qposadr[m.joint(k).id]] = v
        dd.ctrl[m.actuator(k).id] = v
    mujoco.mj_forward(m, dd)
    soles = [m.geom('l_foot_sole').id, m.geom('r_foot_sole').id]
    dd.qpos[2] -= min(dd.geom_xpos[g][2] - m.geom_size[g][2] for g in soles) - 0.0005
    while dd.time < T:
        mujoco.mj_step(m, dd)
    return {j: abs(float(dd.actuator_force[m.actuator('l_' + j).id])) for j in LEG}

def com_y():
    return float(d.subtree_com[1][1])

def main():
    S = []
    S.append(('double stance, home crouch (sim, per leg)', settle({})))
    squat = {f'{s}_{j}': v for s in 'lr' for j, v in (('hip_pitch', -0.9), ('knee', 1.8), ('ankle', -0.9))}
    S.append(('double stance, deep squat for sit / get-up (sim, per leg)', settle(squat)))
    lift = {'r_hip_pitch': -0.9, 'r_knee': 1.5, 'r_ankle': -0.6}
    # single stance (no ankle roll, so in walking the COM sways between the feet and the stance hip roll holds the body
    # + swing leg, whose COM is ~ the hip spacing away from the stance hip): exact moments in the kinematic pose
    S.append(scenario('single stance, home crouch, swing leg up', lift, 'l'))
    S.append(scenario('single stance, deeper crouch (knee 70 deg)', dict(lift, l_hip_pitch=-0.6, l_knee=1.22, l_ankle=-0.62), 'l'))
    worst = {j: max(s[1][j] for s in S) for j in LEG}
    dyn = 2.0      # walking peaks ~2x the static single-stance value (swing-leg acceleration, impacts, PD tracking error)
    need = {j: worst[j] * dyn for j in LEG}
    peak = max(need.values())
    cand = {}
    mass = float(m.body_subtreemass[1])
    for c, sp in CANDIDATES.items():
        # torque scales with robot mass: a lighter servo makes a lighter robot (11 servos x mass difference)
        mscale = (mass - 11 * (55 - sp['mass_g']) / 1000) / mass
        st = max(worst.values()) * mscale
        cand[c] = dict(spec=sp, robot_mass_kg=round(mass * mscale, 3), worst_static_nm=round(st, 3), static_pct_of_stall=round(100 * st / sp['stall']),
                       walking_peak_pct_of_stall=round(100 * dyn * st / sp['stall']), ok=bool(st <= sp['stall'] / 3 + 1e-3 and sp['feedback']))
    res = dict(robot_mass_kg=round(mass, 3), scenarios={n: {k: round(v, 3) for k, v in t.items()} for n, t in S},
               worst_static_nm={k: round(v, 3) for k, v in worst.items()}, dynamic_factor=dyn,
               design_peak_nm={k: round(v, 3) for k, v in need.items()}, candidates=cand,
               rule='cheapest servo with position feedback whose worst static load is <= ~1/3 of stall, so 2x walking peaks stay <= ~2/3 of stall')
    os.makedirs(os.path.join(HERE, 'results'), exist_ok=True)
    json.dump(res, open(os.path.join(HERE, 'results', 'torque_budget.json'), 'w'), indent=1)
    print(json.dumps(res, indent=1))

if __name__ == '__main__':
    main()
