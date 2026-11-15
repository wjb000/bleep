"""STS3215 servo model used between the policy and the MuJoCo joints (the same math runs in the firmware's head:
the policy outputs position targets, the servo's own P-loop turns them into torque).
tau = clip(kp*(q_target - q) - kd*qdot, +-tau_max(qdot)), tau_max falls linearly to 0 at the no-load speed
when the motor is driving (back-EMF), full stall torque when braking. Latency = 0..1 control steps (DR)."""
import json, os, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
PRIOR = json.load(open(os.path.join(HERE, 'servo_prior.json')))

class ServoBank:
    def __init__(self, n, kp=PRIOR['kp'], kd=0.15, stall=PRIOR['stall_nm'], noload=PRIOR['noload_rad_s']):
        self.kp = np.full(n, kp); self.kd = np.full(n, kd); self.stall = np.full(n, stall); self.noload = np.full(n, noload)
        self.offset = np.zeros(n)

    def torque(self, q_target, q, qd):
        tau = self.kp * (q_target + self.offset - q) - self.kd * qd
        driving = np.sign(tau) == np.sign(qd)
        tmax = np.where(driving, self.stall * np.clip(1.0 - np.abs(qd) / self.noload, 0.0, 1.0), self.stall)
        return np.clip(tau, -tmax, tmax)
