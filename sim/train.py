#!/usr/bin/env python3
"""PPO (Stable-Baselines3) on BleepEnv with domain randomisation. CPU is fine.
  python sim/train.py --steps 30000000 --envs 8 --minutes 70 --out runs/walk
Writes runs/<name>/model.zip, progress.csv, and (via export_policy.py) firmware/bleep/policy_weights.h."""
import os, sys, time, argparse, csv
import numpy as np, torch
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor
from stable_baselines3.common.callbacks import BaseCallback
from env import BleepEnv

class Log(BaseCallback):
    def __init__(self, path, minutes, out):
        super().__init__(); self.path, self.t0, self.limit, self.out = path, time.time(), minutes * 60, out
        self.f = open(path, 'w', newline=''); self.w = csv.writer(self.f)
        self.w.writerow(['steps', 'minutes', 'ep_reward', 'ep_len_s']); self.next_ckpt = 2_000_000
    def _on_step(self):
        return time.time() - self.t0 < self.limit
    def _on_rollout_end(self):
        buf = self.model.ep_info_buffer
        if len(buf):
            r = np.mean([e['r'] for e in buf]); l = np.mean([e['l'] for e in buf]) / 50
            self.w.writerow([self.num_timesteps, round((time.time() - self.t0) / 60, 1), round(float(r), 1), round(float(l), 2)]); self.f.flush()
            print(f'{self.num_timesteps:>10d} steps  {(time.time()-self.t0)/60:5.1f} min  ep_reward {r:7.1f}  ep_len {l:5.2f} s', flush=True)
        if self.num_timesteps >= self.next_ckpt:
            self.model.save(os.path.join(self.out, 'model')); self.next_ckpt += 2_000_000

def make(i, task):
    return lambda: BleepEnv(task=task, randomize=True, seed=1000 + i)

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--task', default='walk'); ap.add_argument('--steps', type=int, default=30_000_000)
    ap.add_argument('--envs', type=int, default=8); ap.add_argument('--minutes', type=float, default=70)
    ap.add_argument('--out', default=os.path.join(HERE, 'runs', 'walk')); ap.add_argument('--resume', default=None)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    torch.set_num_threads(2)
    venv = VecMonitor(SubprocVecEnv([make(i, a.task) for i in range(a.envs)], start_method='fork'))
    if a.resume:
        model = PPO.load(a.resume, env=venv)
    else:
        model = PPO('MlpPolicy', venv, n_steps=512, batch_size=2048, n_epochs=5, learning_rate=3e-4, gamma=0.99, gae_lambda=0.95,
                    clip_range=0.2, ent_coef=0.0, max_grad_norm=1.0, verbose=0, seed=0,
                    policy_kwargs=dict(net_arch=dict(pi=[128, 128], vf=[256, 256]), activation_fn=torch.nn.ELU, log_std_init=-1.0))
    t = time.time()
    model.learn(total_timesteps=a.steps, callback=Log(os.path.join(a.out, 'progress.csv'), a.minutes, a.out))
    print(f'trained {model.num_timesteps} steps in {(time.time()-t)/60:.1f} min ({model.num_timesteps/(time.time()-t):.0f} steps/s)')
    model.save(os.path.join(a.out, 'model'))
    from export_policy import export
    export(os.path.join(a.out, 'model.zip'))
