# Bleep

**A 34 cm open-source desk droid with 11 servos, a camera eye and a round LCD face, built for about $240 in parts.**

Site with the 3D viewer, parts list and results: **https://wjb000.github.io/bleep/**

| | |
|---|---|
| Height / mass | 34.3 cm / 1.10 kg |
| Actuators | 11 x Feetech STS3215-C001 serial-bus servos (5 per leg + dome yaw) |
| Brain | Seeed XIAO ESP32S3 Sense (camera, PDM mic, Wi-Fi, 8 MB PSRAM) |
| Senses / face | MPU-6050 IMU, copper touch pad, 1.28" GC9A01 round LCD eye, piezo buzzer |
| Power | 2S LiFePO4 (2 x 18650, 6.4 V nominal, inside the servos' 4-7.4 V rating) |
| Printed parts | 14 (PLA, ~330 g, all fit a 180 mm bed) |
| Parts cost | $240.15 per robot ([docs/BOM.md](docs/BOM.md)) |

## Status (honest)
- **Checked in CAD and sim:** no part overlaps, joint-range sweeps, standing stability, the torque budget, and that the firmware compiles.
- **Walking:** a PPO walking policy is being trained in MuJoCo with domain randomisation. Current results are in `sim/results/walk_eval.json` and on the site. It does **not** walk reliably yet.
- **Get-up after a fall:** not trained yet. The firmware detects the fall, goes limp, and recovers once you stand it up.
- **Voice:** it reacts to loudness only (no wake word yet). Touch and camera-motion tracking are in the firmware.
- **Hardware:** nothing has been printed or assembled yet. Expect clearance tweaks on the first build.

## Repo map
```
cad/        bleep.py (the parametric design), lib.py, export.py (STEP/STL, checks, masses -> URDF/MJCF/glTF)
  step/       every printed part as STEP (+ vendor STS3215 STEP in cad/vendor)
  stl_print/  print-ready STLs, already oriented (ASCII STL)
  reports/    interference / sweep / mass / print-orientation reports
sim/        bleep.xml (MuJoCo), bleep.urdf, meshes/, env.py (Gymnasium), servo.py, train.py (PPO), eval_policy.py,
            check_stand.py, torque_budget.py, sysid.py, export_policy.py, results/
firmware/   bleep/ (robot firmware for XIAO ESP32S3), calibrate/ (IDs, zeroing, joint signs), test/
docs/       BOM.md, PRINT_GUIDE.md, ASSEMBLY.md, make_bom.py, make_site_data.py
site/       GitHub Pages site (three.js viewer), deployed by .github/workflows/pages.yml
```

## Rebuild everything
```bash
pip install cadquery manifold3d trimesh mujoco gymnasium stable-baselines3 torch scipy
python cad/fetch_vendor.py        # downloads the XIAO STEP (not redistributed)
python cad/export.py              # parts, checks, URDF/MJCF, glTF, reports (also writes the full assembly STEP)
python cad/orient_stl.py          # print-ready STLs
python sim/check_stand.py         # stand check
python sim/train.py --minutes 60  # PPO walking policy -> firmware/bleep/policy_weights.h
python sim/eval_policy.py         # walking eval + site replay
python docs/make_bom.py && python docs/make_site_data.py
```
The full assembly STEP (~8 MB) is not committed because it is too large for the upload path used. `cad/export.py` writes it to `cad/step/bleep_assembly.step`.

See [sim/README.md](sim/README.md) for the sim2real workflow, and [docs/ASSEMBLY.md](docs/ASSEMBLY.md) to build one.

Bleep is an original design. It is inspired by the idea of small, friendly droids and is not affiliated with any film franchise.
