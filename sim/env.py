"""Gymnasium + MuJoCo environment for Bleep.
task='walk': track a (vx, vy, yaw-rate) command with a clocked stepping gait, recover from shoves.
task='getup': start lying on the back or the front, get back to the home stance (reward defined; NOT trained yet).

Policy: 50 Hz, outputs 10 leg position targets (home pose + 0.5 rad * action). Physics 500 Hz.
Observations (41, all measurable on the robot): projected gravity (3, from the IMU), gyro (3), command (3),
leg joint positions - home (10), leg joint velocities (10), last action (10), gait clock sin/cos (2).
Domain randomisation: friction, link masses, body COM, servo gains/stall/offsets, joint friction/damping/armature,
0-20 ms action latency, sensor noise, random shoves. See sim/README.md."""
import os, json, numpy as np, mujoco, gymnasium as gym
from gymnasium import spaces
from servo import ServoBank, PRIOR

HERE = os.path.dirname(os.path.abspath(__file__))
LEG = ['l_hip_yaw', 'l_hip_roll', 'l_hip_pitch', 'l_knee', 'l_ankle', 'r_hip_yaw', 'r_hip_roll', 'r_hip_pitch', 'r_knee', 'r_ankle']
CONTROL_HZ = 50
ACTION_SCALE = 0.5
OBS_DIM = 41
GAIT_PERIOD = 0.5
CMD_SCALE = np.array([3.0, 3.0, 1.0])
DQ_SCALE = 0.05

class BleepEnv(gym.Env):
    metadata = {'render_modes': ['rgb_array']}

    def __init__(self, task='walk', randomize=True, episode_s=10.0, seed=0, fixed_cmd=None, render_mode=None):
        self.task, self.randomize, self.fixed_cmd = task, randomize, fixed_cmd
        self.model = mujoco.MjModel.from_xml_path(os.path.join(HERE, 'bleep.xml'))
        m = self.model
        self.data = mujoco.MjData(m)
        self.rng = np.random.default_rng(seed)
        home = json.load(open(os.path.join(HERE, 'home_pose.json')))
        self.jid = [m.joint(n).id for n in LEG] + [m.joint('neck_yaw').id]
        self.qadr = np.array([m.jnt_qposadr[j] for j in self.jid]); self.dadr = np.array([m.jnt_dofadr[j] for j in self.jid])
        self.aid = [m.actuator(n).id for n in LEG + ['neck_yaw']]
        self.q_home = np.array([home[n] for n in LEG + ['neck_yaw']])
        self.lo = np.array([m.jnt_range[j][0] for j in self.jid]); self.hi = np.array([m.jnt_range[j][1] for j in self.jid])
        # actuators become torque motors driven by the servo model
        m.actuator_gainprm[:, 0] = 1.0; m.actuator_gainprm[:, 1:] = 0.0
        m.actuator_biasprm[:] = 0.0; m.actuator_biastype[:] = 0
        m.actuator_ctrlrange[:] = [-PRIOR['stall_nm'], PRIOR['stall_nm']]
        self.n = len(self.jid)
        self.nom = dict(mass=m.body_mass.copy(), inertia=m.body_inertia.copy(), ipos=m.body_ipos.copy(), fric=m.geom_friction.copy(),
                        damp=m.dof_damping.copy(), fl=m.dof_frictionloss.copy(), arm=m.dof_armature.copy())
        self.base = m.body('base').id
        self.feet = [m.geom('l_foot_sole').id, m.geom('r_foot_sole').id]
        self.floor = m.geom('floor').id
        self.foot_bodies = [m.body('l_ankle').id, m.body('r_ankle').id]
        self.s_quat, self.s_gyro = m.sensor('imu_quat').adr[0], m.sensor('imu_gyro').adr[0]
        self.substeps = int(round(1.0 / CONTROL_HZ / m.opt.timestep))
        self.max_steps = int(episode_s * CONTROL_HZ)
        self.action_space = spaces.Box(-1.0, 1.0, (10,), np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, (OBS_DIM,), np.float32)
        self.servo = ServoBank(self.n)
        self.render_mode = render_mode; self._renderer = None

    # ---------------------------------------------------------------- domain randomisation
    def _randomize(self):
        m, nom, r = self.model, self.nom, self.rng
        m.body_mass[:] = nom['mass']; m.body_inertia[:] = nom['inertia']; m.body_ipos[:] = nom['ipos']; m.geom_friction[:] = nom['fric']
        m.dof_damping[:] = nom['damp']; m.dof_frictionloss[:] = nom['fl']; m.dof_armature[:] = nom['arm']
        self.servo = ServoBank(self.n)
        self.latency = 0; self.noise = 0.0
        if not self.randomize:
            return
        s = r.uniform(0.9, 1.1, m.nbody); s[0] = 1
        m.body_mass[:] = nom['mass'] * s; m.body_inertia[:] = nom['inertia'] * s[:, None]
        m.body_ipos[self.base] += r.uniform(-0.01, 0.01, 3)
        m.geom_friction[:, 0] = r.uniform(0.5, 1.2)
        m.dof_damping[self.dadr] = r.uniform(0.03, 0.15, self.n)
        m.dof_frictionloss[self.dadr] = r.uniform(0.02, 0.1, self.n)
        m.dof_armature[self.dadr] = r.uniform(0.01, 0.03, self.n)
        self.servo.kp *= r.uniform(0.8, 1.2, self.n); self.servo.kd = r.uniform(0.05, 0.25, self.n)
        self.servo.stall *= r.uniform(0.85, 1.1, self.n); self.servo.offset = r.uniform(-0.03, 0.03, self.n)
        self.latency = int(r.integers(0, 2)); self.noise = 1.0

    def _cmd(self):
        if self.fixed_cmd is not None:
            return np.array(self.fixed_cmd, float)
        if self.task != 'walk' or self.rng.random() < 0.1:
            return np.zeros(3)
        c = np.array([self.rng.uniform(-0.1, 0.25), self.rng.uniform(-0.08, 0.08), self.rng.uniform(-0.8, 0.8)])
        k = self.rng.random()
        if k < 0.3: c[1:] = 0          # pure forward/back
        elif k < 0.45: c[:2] = 0       # turn in place
        return c

    def reset(self, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        m, d = self.model, self.data
        self._randomize()
        mujoco.mj_resetDataKeyframe(m, d, m.key('home').id)
        if self.task == 'getup':
            q = self.q_home + self.rng.uniform(-0.3, 0.3, self.n); q[-1] = 0
            d.qpos[self.qadr] = np.clip(q, self.lo, self.hi)
            ang = np.pi / 2 * (1 if self.rng.random() < 0.5 else -1)
            d.qpos[3:7] = [np.cos(ang / 2), 0, np.sin(ang / 2), 0]
            d.qpos[2] = 0.2
        else:
            d.qpos[self.qadr] = np.clip(self.q_home + (self.rng.uniform(-0.05, 0.05, self.n) if self.randomize else 0), self.lo, self.hi)
        mujoco.mj_forward(m, d)
        if self.task != 'getup':
            zmin = min(d.geom_xpos[g][2] - m.geom_size[g][2] for g in self.feet)
            d.qpos[2] -= zmin - 0.0005
            mujoco.mj_forward(m, d)
        self.h0 = 0.162
        self.t = 0; self.phase = 0.0
        self.cmd = self._cmd()
        self.a_prev = np.zeros(10); self.a_buf = [np.zeros(10), np.zeros(10)]
        self.target = self.q_home.copy()
        self.next_push = self.rng.uniform(2.0, 5.0) if self.randomize else 1e9
        self.air = np.zeros(2)
        return self._obs(), {}

    # ---------------------------------------------------------------- observation
    def _gravity(self):
        q = self.data.sensordata[self.s_quat:self.s_quat + 4]
        R = np.zeros(9); mujoco.mju_quat2Mat(R, q); R = R.reshape(3, 3)
        return R.T @ np.array([0, 0, -1.0])

    def _obs(self):
        d = self.data
        g = self._gravity(); w = d.sensordata[self.s_gyro:self.s_gyro + 3].copy()
        q = d.qpos[self.qadr[:10]] - self.q_home[:10]; qd = d.qvel[self.dadr[:10]]
        if self.noise:
            g = g + self.rng.normal(0, 0.03, 3); w = w + self.rng.normal(0, 0.1, 3)
            q = q + self.rng.normal(0, 0.005, 10); qd = qd + self.rng.normal(0, 0.3, 10)
        ph = 2 * np.pi * self.phase
        o = np.concatenate([g, w * 0.25, self.cmd * CMD_SCALE, q, qd * DQ_SCALE, self.a_prev, [np.sin(ph), np.cos(ph)]])
        return o.astype(np.float32)

    # ---------------------------------------------------------------- step
    def step(self, action):
        m, d = self.model, self.data
        a = np.clip(np.asarray(action, float), -1, 1)
        self.a_buf.append(a); a_applied = self.a_buf[-1 - self.latency]; self.a_buf = self.a_buf[-3:]
        tgt = self.q_home.copy(); tgt[:10] += ACTION_SCALE * a_applied
        tgt = np.clip(tgt, self.lo, self.hi)
        if self.randomize and d.time > self.next_push:
            d.qvel[0:2] += self.rng.uniform(-0.3, 0.3, 2); self.next_push = d.time + self.rng.uniform(2.0, 5.0)
        tau_sq = 0.0
        for _ in range(self.substeps):
            q = d.qpos[self.qadr]; qd = d.qvel[self.dadr]
            tau = self.servo.torque(tgt, q, qd)
            d.ctrl[self.aid] = tau
            mujoco.mj_step(m, d)
            tau_sq += float(np.dot(tau[:10], tau[:10]))
        self.t += 1; self.phase = (self.phase + 1.0 / CONTROL_HZ / GAIT_PERIOD) % 1.0
        r, info = self._reward(a, tau_sq / self.substeps)
        self.a_prev = a
        up = d.xmat[self.base].reshape(3, 3)[2, 2]
        h = d.subtree_com[self.base][2]
        if self.task == 'getup':
            term = False
        else:
            term = bool(up < np.cos(np.radians(40)) or h < 0.65 * self.h0)
        trunc = self.t >= self.max_steps
        if self.fixed_cmd is None and self.task == 'walk' and self.t % 200 == 0:
            self.cmd = self._cmd()
        return self._obs(), float(r), term, trunc, info

    def _contacts(self):
        c = np.zeros(2, bool)
        d = self.data
        for i in range(d.ncon):
            g1, g2 = d.contact[i].geom1, d.contact[i].geom2
            for k, f in enumerate(self.feet):
                if (g1 == f and g2 == self.floor) or (g2 == f and g1 == self.floor):
                    c[k] = True
        return c

    def _reward(self, a, tau_sq):
        d = self.data
        R = d.xmat[self.base].reshape(3, 3)
        v_world = d.qvel[0:3]; v = R.T @ v_world
        wz = d.qvel[5]
        g = self._gravity()
        h = d.subtree_com[self.base][2]
        if self.task == 'getup':
            up = R[2, 2]
            r = 1.0 * (up + 1) / 2 + 2.0 * np.clip(h / self.h0, 0, 1) + (1.0 if up > 0.9 and h > 0.9 * self.h0 else 0.0)
            r -= 0.01 * np.sum((a - self.a_prev) ** 2) + 0.002 * tau_sq
            return r, {}
        cont = self._contacts()
        stance = np.array([self.phase < 0.5, self.phase >= 0.5])
        foot_h = np.array([d.xpos[b][2] for b in self.foot_bodies]) - 0.027
        foot_v = np.array([np.linalg.norm(d.cvel[b][3:5]) for b in self.foot_bodies])
        r_lin = np.exp(-np.sum((v[:2] - self.cmd[:2]) ** 2) / 0.02)    # sigma^2 wide enough that standing still on a 0.15 m/s command still gets a gradient
        r_yaw = np.exp(-(wz - self.cmd[2]) ** 2 / 0.15)
        r_gait = np.sum(cont == stance) / 2.0
        r_clear = np.sum((~stance) * np.exp(-((foot_h - 0.03) / 0.012) ** 2)) / 1.0
        p_orient = np.sum(g[:2] ** 2)
        p_height = (h - self.h0) ** 2
        p_rate = np.sum((a - self.a_prev) ** 2)
        p_slip = np.sum(cont * foot_v ** 2)
        p_angvel = d.qvel[3] ** 2 + d.qvel[4] ** 2
        p_vz = v_world[2] ** 2
        r = (2.5 * r_lin + 1.2 * r_yaw + 0.4 * r_gait + 0.4 * r_clear + 0.3
             - 4.0 * p_orient - 100.0 * p_height - 0.03 * p_rate - 2.0 * p_slip - 0.05 * p_angvel - 2.0 * p_vz - 0.005 * tau_sq)
        return r, dict(vx=v[0], vy=v[1], wz=wz, r_lin=r_lin, r_yaw=r_yaw, r_gait=r_gait, tau_rms=np.sqrt(tau_sq / 10))

    def render(self):
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, 360, 480)
        cam = mujoco.MjvCamera(); cam.lookat[:] = self.data.qpos[:3]; cam.distance = 0.9; cam.azimuth = 135; cam.elevation = -15
        self._renderer.update_scene(self.data, cam)
        return self._renderer.render()
