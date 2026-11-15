# Print guide

There are 14 parts in total: 5 mirror pairs for the legs, plus the body (tub and lid) and the head (shell and base).
Every part fits a 256 mm bed. It also fits a 180 mm bed: the largest part is 128 mm.

- **Print-ready files:** `cad/stl_print/`. Each part is already rotated to the orientation below and sitting on the bed.
- **Robot-frame files:** `cad/stl/` (assembled positions) and `cad/step/` (editable solids).

## Settings (PLA)

| Setting | Value |
|---|---|
| Layer | 0.2 mm |
| Walls | 3 (4 for the feet) |
| Infill | 25% gyroid |
| Top/bottom | 4 layers |
| Supports | tree supports, only where listed below (build plate only) |
| Brim | 5 mm brim on the thigh and shin |

Colours used in the renders: cream shell, teal accents (lid and head base), dark grey for the leg brackets and feet.
Any colours work.

## Parts

| Part | Qty | Up axis (robot frame) | Size on bed (mm) | Overhang | Mass (g) | Notes |
|---|---|---|---|---|---|---|
| `l_/r_hip_bracket` | 2 | +x | 34 x 55 x 40 | 4.2% | 16.1 | Dark. No supports. |
| `l_/r_hip_carrier` | 2 | +y | 61 x 72 x 44 | 5.4% | 22.6 | Dark. Support only the inside of the pitch pod, touching the build plate. |
| `l_/r_thigh` | 2 | -y | 33 x 94 x 41 | 7.9% | 17.5 | Cream. Lay it on its outer hub plate; no supports. |
| `l_/r_shin` | 2 | -y | 33 x 89 x 41 | 8.0% | 17.0 | Cream. Same as the thigh. |
| `l_/r_foot` | 2 | +z | 96 x 50 x 41 | 1.0% | 21.7 | Dark. Sole flat on the bed; no supports. 4 walls for the sole. |
| `body_tub` | 1 | +z | 80 x 128 x 42 | 4.0% | 43.2 | Cream. Upright; no supports. |
| `body_lid` | 1 | +y | 80 x 41 x 128 | 10.0% | 40.2 | Teal. Stand it on its side. Use tree supports under the neck-servo pod only. |
| `head_shell` | 1 | +x | 59 x 92 x 94 | 12.4% | 33.2 | Cream. Lay it on its back. Use tree supports for the eye bezel and the inside of the dome. |
| `head_base` | 1 | +z | 93 x 93 x 28 | 0.0% | 20.3 | Teal. Flat; no supports. |

Total plastic: about **327 g**. That figure is the CAD volume x PLA density x 0.70 to allow for infill.
Buy for about 360 g of filament, which includes brims and purge.

"Overhang" is the share of the part's surface that faces down at more than 45 degrees in that orientation.
The orientation was chosen automatically by `cad/export.py`, which picks the lowest overhang with a large flat base.

## After printing

- Melt the 4 M3 heat-set inserts into the lid-screw bosses in the corners of the tub. Use 220 C and keep the iron straight.
- Test-fit one STS3215 in one pod first. The pod clearance is 0.3 mm per side.
  If your printer runs tight, sand the pod walls rather than scaling the part.
- Check that the horn sits flat in each hub recess and that the 4 M3 holes line up.
  The pattern is a 9.8 mm square around the shaft.
