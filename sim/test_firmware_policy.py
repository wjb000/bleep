#!/usr/bin/env python3
"""Compile firmware/bleep/policy.h on the host and check it reproduces the PPO actor's deterministic actions
on observations recorded from a sim rollout. Run: python sim/test_firmware_policy.py [model.zip]"""
import os, sys, subprocess, tempfile, json, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE); sys.path.insert(0, HERE)
from env import BleepEnv
from stable_baselines3 import PPO
mz = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, 'runs', 'walk', 'model.zip')
model = PPO.load(mz, device='cpu')
e = BleepEnv(randomize=True, seed=7); o, _ = e.reset(seed=7)
obs, ref = [], []
for i in range(300):
    a, _ = model.predict(o, deterministic=True)
    obs.append(o.copy()); ref.append(np.clip(a, -1, 1))
    o, r, te, tr, _ = e.step(a)
    if te or tr:
        o, _ = e.reset()
exe = os.path.join(tempfile.mkdtemp(), 'tp')
subprocess.check_call(['gcc', '-O2', '-I', os.path.join(ROOT, 'firmware', 'bleep'), os.path.join(ROOT, 'firmware', 'test', 'test_policy.c'), '-lm', '-o', exe])
out = subprocess.run([exe], input='\n'.join(' '.join(f'{x:.9g}' for x in v) for v in obs), capture_output=True, text=True).stdout
got = np.array([[float(x) for x in l.split()] for l in out.strip().splitlines()])
err = float(np.abs(got - np.array(ref)).max())
res = dict(samples=len(obs), max_abs_error=err, passed=err < 1e-4)
os.makedirs(os.path.join(HERE, 'results'), exist_ok=True)
json.dump(res, open(os.path.join(HERE, 'results', 'firmware_policy_test.json'), 'w'), indent=1)
print(res)
