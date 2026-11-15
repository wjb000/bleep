# Sim2real for Bleep

Everything here is generated from the same CAD (`cad/bleep.py`), so the masses, inertias, joint positions and limits in
`bleep.xml` / `bleep.urdf` match the printed robot. Masses assume PLA at 70% effective fill and 58 g per servo with its horn and screws.

## 1. Check the model
- `python sim/check_stand.py`: holds the home crouch for 5 s, then applies growing shoves. Writes `results/stand_check.json`.
- `python sim/torque_budget.py`: worst-case joint torques against several servo candidates. Writes `results/torque_budget.json`.

## 2. Servo model (`servo.py`)
The STS3215 is modelled as its internal position controller, not as an ideal motor:
`tau = clip(kp*(q_target - q) - kd*qd, +-tau_max(speed))`.
- The torque limit falls linearly from stall (1.65 N·m at 6.4 V) to zero at no-load speed (4.8 rad/s).
- The model adds joint damping, friction loss and rotor armature.
- The priors are in `servo_prior.json`.

## 3. Measure your servo (`sysid.py`)
Clamp one servo, bolt a 100 mm arm with a 200 g weight to it, and connect it to a PC through the Waveshare adapter's USB-C port.
```
python sim/sysid.py record --port /dev/ttyACM0 --id 1 --arm 0.10 --mass 0.20 --out logs/servo1.csv
python sim/sysid.py fit logs/servo1.csv --arm 0.10 --mass 0.20
```
`fit` writes the fitted gain, damping, stall torque, friction and armature to `sim/servo_fit.json`. Copy them into `servo_prior.json` and centre the randomisation ranges on them.
`python sim/sysid.py selftest` checks the fitting code on a synthetic log (`results/sysid_selftest.json`).

## 4. Train (`env.py`, `train.py`)
- Observation (41 values):
  - gravity in the body frame and the gyro (IMU);
  - the command (vx, vy, yaw rate);
  - 10 joint positions and velocities;
  - the last action;
  - the gait clock (sin/cos of a 0.5 s cycle).
- Action: 10 joint targets around the home crouch, `q = q_home + 0.5 * a`, sent at 50 Hz. Physics runs at 500 Hz.
- Reward: velocity and yaw tracking, a clocked stepping pattern, foot clearance, and penalties for tilt, height change, jerk, foot slip and torque.
- Domain randomisation, re-drawn every episode:
  - friction 0.5-1.2, link mass ±10%, body COM ±1 cm;
  - servo kp ±20%, kd 0.05-0.25, stall 85-110%;
  - zero offsets ±0.03 rad;
  - joint damping 0.03-0.15, friction loss 0.02-0.1, armature 0.01-0.03;
  - 0-1 control steps of latency, IMU and encoder noise;
  - shoves of up to ±0.3 m/s every 2-5 s.
- `python sim/train.py --minutes 60 --envs 8` (SB3 PPO, CPU) writes `runs/walk/` and exports the policy to `firmware/bleep/policy_weights.h`.

## 5. Evaluate and deploy
- `python sim/eval_policy.py runs/walk/model.zip`: runs fixed commands, nominal and randomised. Writes `results/walk_eval.json` and the replay for the site.
- `python sim/test_firmware_policy.py runs/walk/model.zip`: compiles the firmware's C forward pass with gcc and checks it against PyTorch.
- Flash `firmware/bleep` (see `docs/ASSEMBLY.md`). The firmware builds the exact same observation from the IMU and servo feedback.

## Known gaps
- The walking policy is still being improved; see `results/walk_eval.json` for the real numbers.
- The get-up reward exists (`task='getup'`) but has not been trained.
- The servo priors come from the datasheet until someone runs `sysid.py` on a real STS3215.
- The CAD range-of-motion sweep moves one joint at a time.
