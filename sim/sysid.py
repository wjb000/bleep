#!/usr/bin/env python3
"""Servo system identification for the STS3215 (the biggest sim2real gap is the actuator, so measure it).

Rig: one STS3215 clamped to the desk, a printed 100 mm arm (cad: use the thigh STL as the arm, or any bar) with a known
mass bolted at the end, so gravity loads the servo like a leg. The Waveshare adapter's USB-C port talks to it from a PC.

  1) record:   python sim/sysid.py record --port /dev/ttyACM0 --id 1 --arm 0.10 --mass 0.20 --out logs/servo1.csv
               (steps + chirps on the goal position at 50 Hz; logs target, position, speed, load, voltage)
  2) fit:      python sim/sysid.py fit logs/servo1.csv --arm 0.10 --mass 0.20
               (replays the same targets through a 1-DOF MuJoCo pendulum driven by sim/servo.py and fits
                kp, kd, stall torque, joint friction, damping, armature by least squares; writes sim/servo_fit.json)
  3) selftest: python sim/sysid.py selftest
               (makes a synthetic log from known parameters + noise and checks the fit recovers them; no hardware)
Then copy the fitted values into sim/servo_prior.json (or let env.py read servo_fit.json) and centre the DR ranges on them."""
import os, sys, json, csv, time, argparse, numpy as np, mujoco
from scipy.optimize import least_squares
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
from servo import ServoBank
DT_CTRL = 0.02
PARAMS = ['kp', 'kd', 'stall', 'frictionloss', 'damping', 'armature']

def pendulum_xml(arm, mass):
    return f"""<mujoco><option timestep="0.002"/><worldbody>
  <body name="arm"><joint name="j" axis="0 1 0" damping="0.05" frictionloss="0.05" armature="0.02"/>
  <geom type="capsule" fromto="0 0 0 {arm} 0 0" size="0.008" mass="0.02"/>
  <geom type="sphere" pos="{arm} 0 0" size="0.015" mass="{mass}"/></body></worldbody>
  <actuator><motor joint="j" ctrlrange="-5 5"/></actuator></mujoco>"""

def targets(seconds=24.0, seed=0):
    r = np.random.default_rng(seed); t = np.arange(0, seconds, DT_CTRL); q = np.zeros_like(t)
    for i, tt in enumerate(t):
        if tt < 8: q[i] = 0.6 * (np.floor(tt / 1.0) % 2) - 0.3                          # 1 Hz steps
        elif tt < 16: f = 0.2 + (tt - 8) * 0.35; q[i] = 0.35 * np.sin(2 * np.pi * f * (tt - 8))  # chirp 0.2 -> 3 Hz
        else: q[i] = 0.8 * (np.floor((tt - 16) / 2.0) % 2) - 0.4                        # big slow steps (saturation)
    return t, q

def simulate(p, t, qt, arm, mass):
    m = mujoco.MjModel.from_xml_string(pendulum_xml(arm, mass))
    m.dof_frictionloss[0] = p['frictionloss']; m.dof_damping[0] = p['damping']; m.dof_armature[0] = p['armature']
    d = mujoco.MjData(m); s = ServoBank(1, kp=p['kp'], kd=p['kd'], stall=p['stall'])
    out = np.zeros_like(qt); sub = int(round(DT_CTRL / m.opt.timestep))
    for i in range(len(t)):
        for _ in range(sub):
            d.ctrl[0] = s.torque(np.array([qt[i]]), d.qpos[:1], d.qvel[:1])[0]; mujoco.mj_step(m, d)
        out[i] = d.qpos[0]
    return out

def fit(t, qt, qm, arm, mass, x0=None):
    x0 = np.array(x0 or [12.0, 0.15, 1.65, 0.05, 0.08, 0.02])
    lo = np.array([2, 0.0, 0.5, 0.0, 0.0, 0.002]); hi = np.array([60, 1.0, 3.0, 0.3, 0.5, 0.1])
    f = lambda x: simulate(dict(zip(PARAMS, x)), t, qt, arm, mass) - qm
    r = least_squares(f, x0, bounds=(lo, hi), x_scale=x0, diff_step=0.05, max_nfev=200)
    return dict(zip(PARAMS, [float(v) for v in r.x])), float(np.sqrt(np.mean(r.fun ** 2)))

def record(a):
    import serial   # pip install pyserial
    ser = serial.Serial(a.port, 1000000, timeout=0.01)
    def pkt(id_, inst, params):
        b = [0xFF, 0xFF, id_, len(params) + 2, inst] + params
        return bytes(b + [(~sum(b[2:])) & 0xFF])
    def read(id_, addr, n):
        ser.reset_input_buffer(); ser.write(pkt(id_, 0x02, [addr, n])); r = ser.read(6 + n)
        return r[5:5 + n] if len(r) == 6 + n else None
    t, qt = targets()
    rows = []; t0 = time.time()
    for i in range(len(t)):
        c = int(2048 + qt[i] * 4096 / (2 * np.pi))
        ser.write(pkt(a.id, 0x03, [42, c & 0xFF, c >> 8, 0, 0, 0, 0])); ser.read(6)
        b = read(a.id, 56, 8)
        if b:
            pos = b[0] | b[1] << 8; spd = b[2] | b[3] << 8; spd = -(spd & 0x7FFF) if spd & 0x8000 else spd
            load = b[4] | b[5] << 8; load = -(load & 0x3FF) if load & 0x400 else load
            rows.append([t[i], qt[i], (pos - 2048) * 2 * np.pi / 4096, spd * 2 * np.pi / 4096, load / 1000.0, b[6] / 10.0])
        while time.time() - t0 < t[i] + DT_CTRL:
            pass
    os.makedirs(os.path.dirname(a.out) or '.', exist_ok=True)
    with open(a.out, 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['t', 'target', 'pos', 'speed', 'load', 'volt']); w.writerows(rows)
    print('wrote', a.out, len(rows), 'samples')

def main():
    ap = argparse.ArgumentParser(); ap.add_argument('mode', choices=['record', 'fit', 'selftest']); ap.add_argument('log', nargs='?')
    ap.add_argument('--port'); ap.add_argument('--id', type=int, default=1); ap.add_argument('--arm', type=float, default=0.10)
    ap.add_argument('--mass', type=float, default=0.20); ap.add_argument('--out', default='logs/servo.csv')
    a = ap.parse_args()
    if a.mode == 'record':
        return record(a)
    if a.mode == 'fit':
        rows = np.array([[float(x) for x in r] for r in list(csv.reader(open(a.log)))[1:]])
        p, rmse = fit(rows[:, 0], rows[:, 1], rows[:, 2], a.arm, a.mass)
        json.dump(dict(params=p, rmse_rad=rmse, log=a.log), open(os.path.join(HERE, 'servo_fit.json'), 'w'), indent=1)
        print(p, 'rmse', rmse); return
    # selftest
    t, qt = targets(); true = dict(kp=9.0, kd=0.22, stall=1.45, frictionloss=0.08, damping=0.12, armature=0.03)
    qm = simulate(true, t, qt, a.arm, a.mass) + np.random.default_rng(1).normal(0, 0.003, len(t))
    p, rmse = fit(t, qt, qm, a.arm, a.mass)
    err = {k: round(100 * (p[k] - true[k]) / true[k], 1) for k in PARAMS}
    res = dict(true=true, fitted={k: round(v, 4) for k, v in p.items()}, error_pct=err, rmse_rad=round(rmse, 5),
               note='synthetic log (3 mrad noise) from known parameters; checks the fitting code, not the real servo')
    os.makedirs(os.path.join(HERE, 'results'), exist_ok=True)
    json.dump(res, open(os.path.join(HERE, 'results', 'sysid_selftest.json'), 'w'), indent=1)
    print(json.dumps(res, indent=1))

if __name__ == '__main__':
    main()
