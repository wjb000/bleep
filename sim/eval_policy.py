#!/usr/bin/env python3
"""Evaluate a trained walking policy in sim and export a replay for the website viewer.
  python sim/eval_policy.py sim/runs/walk/model.zip
Scenarios x {nominal model, domain-randomised}: walk forward 0.15 m/s, backward 0.08 m/s, turn 0.6 rad/s, walk in place.
Writes sim/results/walk_eval.json and site/assets/walk_replay.json (joint angles + body pose at 25 fps)."""
import os, sys, json, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE); sys.path.insert(0, HERE)
from env import BleepEnv, LEG
from stable_baselines3 import PPO

SCEN = {'forward 0.15 m/s': (0.15, 0, 0), 'backward 0.08 m/s': (-0.08, 0, 0), 'turn 0.6 rad/s': (0, 0, 0.6), 'in place': (0, 0, 0)}

def rollout(model, cmd, randomize, seed, T=10.0, record=False):
    e = BleepEnv(randomize=randomize, seed=seed, fixed_cmd=cmd, episode_s=T)
    o, _ = e.reset(seed=seed)
    x0 = e.data.qpos[:2].copy(); vx, wz, taus, frames = [], [], [], []
    fell = False; yaw0 = None
    for k in range(int(T * 50)):
        a, _ = model.predict(o, deterministic=True)
        o, r, te, tr, info = e.step(a)
        if k > 50:
            vx.append(info['vx']); wz.append(info['wz'])
        taus.append(np.abs(e.data.actuator_force[:10]).copy())
        if record and k % 2 == 0:
            d = e.data
            frames.append(dict(p=[round(float(v), 4) for v in d.qpos[:3]], q=[round(float(v), 4) for v in d.qpos[3:7]],
                               j={n: round(float(d.qpos[e.qadr[i]]), 4) for i, n in enumerate(LEG + ['neck_yaw'])}))
        if te:
            fell = True; break
        if tr:
            break
    taus = np.array(taus)
    dist = float(np.linalg.norm(e.data.qpos[:2] - x0))
    return dict(fell=fell, seconds=round(k / 50, 2), mean_vx=float(np.mean(vx)) if vx else 0.0, mean_wz=float(np.mean(wz)) if wz else 0.0,
                distance_m=dist, tau_peak=taus.max(0).tolist(), tau_rms=np.sqrt((taus ** 2).mean(0)).tolist()), frames

def main():
    mz = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, 'runs', 'walk', 'model.zip')
    model = PPO.load(mz, device='cpu')
    res = {}
    for name, cmd in SCEN.items():
        for rnd in (False, True):
            runs = [rollout(model, cmd, rnd, s)[0] for s in range(10 if rnd else 3)]
            key = f'{name} | {"randomised" if rnd else "nominal"}'
            res[key] = dict(cmd=cmd, episodes=len(runs), falls=sum(r['fell'] for r in runs), mean_vx_m_s=round(float(np.mean([r['mean_vx'] for r in runs])), 3),
                            mean_yaw_rate=round(float(np.mean([r['mean_wz'] for r in runs])), 3),
                            mean_distance_m=round(float(np.mean([r['distance_m'] for r in runs])), 2),
                            peak_torque_nm=round(float(np.max([max(r['tau_peak']) for r in runs])), 3),
                            rms_torque_nm=round(float(np.mean([np.mean(r['tau_rms']) for r in runs])), 3),
                            peak_torque_by_joint={LEG[i]: round(float(np.max([r['tau_peak'][i] for r in runs])), 3) for i in range(10)})
            print(key, {k: v for k, v in res[key].items() if k != 'peak_torque_by_joint'})
    os.makedirs(os.path.join(HERE, 'results'), exist_ok=True)
    json.dump(res, open(os.path.join(HERE, 'results', 'walk_eval.json'), 'w'), indent=1)
    # replay for the site (nominal model): 6 s forward, 4 s turning
    r1, f1 = rollout(model, (0.15, 0, 0.0), False, 3, T=6.0, record=True)
    r2, f2 = rollout(model, (0.0, 0, 0.6), False, 4, T=4.0, record=True)
    json.dump(dict(fps=25, clips={'walk forward 0.15 m/s': f1, 'turn 0.6 rad/s': f2}), open(os.path.join(ROOT, 'site', 'assets', 'walk_replay.json'), 'w'))
    print('replay frames', len(f1), len(f2))

if __name__ == '__main__':
    main()
