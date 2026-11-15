#!/usr/bin/env python3
"""Stand check (no learning involved): load the MJCF, hold the home pose with the stock position servos
(torque limit = datasheet stall derated to the pack voltage) and
  1. settle for 5 s on flat ground: must stay up with < 5 deg tilt and no self-collision,
  2. shove it from 4 directions with growing impulses: reports the largest shove a *passive* stand survives
     (the walking policy is what is supposed to recover from bigger ones).
Also logs the static servo torques the stand needs. Writes sim/results/stand_check.json. Run: python sim/check_stand.py"""
import os, json, numpy as np, mujoco
HERE = os.path.dirname(os.path.abspath(__file__))
M = mujoco.MjModel.from_xml_path(os.path.join(HERE, 'bleep.xml'))

def run(impulse=None, t_push=2.0, T=5.0):
    m = M; d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, m.key('home').id); mujoco.mj_forward(m, d)
    base, floor = m.body('base').id, m.geom('floor').id
    h0 = None; tilt_max = 0.0; fell = False; selfc = set(); tau = None
    while d.time < T:
        if impulse is not None and abs(d.time - t_push) < m.opt.timestep / 2:
            d.qvel[0:3] += np.array(impulse) / m.body_subtreemass[base]
        mujoco.mj_step(m, d)
        if h0 is None and d.time > 0.5:
            h0 = d.subtree_com[base][2]
        if tau is None and d.time > 1.9:
            tau = d.actuator_force.copy()
        up = d.xmat[base].reshape(3, 3)[:, 2]
        tilt = np.degrees(np.arccos(np.clip(up[2], -1, 1))); tilt_max = max(tilt_max, tilt)
        if tilt > 45 or (h0 and d.subtree_com[base][2] < 0.6 * h0):
            fell = True; break
        for i in range(d.ncon):
            c = d.contact[i]
            if c.geom1 != floor and c.geom2 != floor:
                selfc.add(tuple(sorted((m.geom(c.geom1).name, m.geom(c.geom2).name))))
    return dict(fell=fell, tilt_max=tilt_max, h0=h0, h1=d.subtree_com[base][2], tau=tau, selfc=selfc)

def main():
    m = M
    names = [m.actuator(i).name for i in range(m.nu)]
    r0 = run(None, T=5.0)
    shoves = {}
    for name, dirv in (('forward', (1, 0, 0)), ('backward', (-1, 0, 0)), ('left', (0, 1, 0)), ('right', (0, -1, 0))):
        best = 0.0
        for J in (0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.4, 0.5):
            if run(np.array(dirv) * J)['fell']:
                break
            best = J
        shoves[name] = best
    mass = float(m.body_subtreemass[m.body('base').id])
    res = dict(passed=bool(not r0['fell'] and r0['tilt_max'] < 5 and not r0['selfc'] and min(shoves.values()) >= 0.1),
               settle_5s=dict(fell=r0['fell'], max_tilt_deg=round(float(r0['tilt_max']), 2), com_height_m=round(float(r0['h0']), 4),
                              com_drift_mm=round(1000 * float(r0['h0'] - r0['h1']), 2), self_contacts=sorted(r0['selfc'])),
               passive_shove_survived_Ns=shoves,
               passive_shove_survived_m_s={k: round(v / mass, 2) for k, v in shoves.items()},
               static_torque_nm={n: round(float(t), 3) for n, t in zip(names, r0['tau'])},
               max_static_torque_nm=round(float(np.abs(r0['tau']).max()), 3), stall_limit_nm=float(m.actuator_forcerange[0, 1]), mass_kg=round(mass, 3))
    os.makedirs(os.path.join(HERE, 'results'), exist_ok=True)
    json.dump(res, open(os.path.join(HERE, 'results', 'stand_check.json'), 'w'), indent=1)
    print(json.dumps(res, indent=1))
    return res

if __name__ == '__main__':
    main()
