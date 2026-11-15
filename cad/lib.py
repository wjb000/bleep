"""Bleep CAD helpers (CadQuery, units mm).

STS3215 geometry measured from the SO-ARM100 STEP model (cad/vendor/STS3215_03a.step, Apache-2.0):
STEP frame: shaft axis = +z through (x=12.5, y=0); output horn disc r=10, top face at z=18.7;
main block x[-22.7,22.7] y[-12.4,12.4] z[-15.9,17.0]; rear idler disc r=10 to z=-17.7.
"""
import cadquery as cq
import numpy as np

SHAFT_X = 12.5
Z_HORN_TOP = 18.7
SRV_X0, SRV_X1 = -22.7 - SHAFT_X, 22.7 - SHAFT_X      # -35.2 .. +10.2 (tail side negative)
SRV_HY = 12.4
SRV_Z0, SRV_Z1 = -17.7 - Z_HORN_TOP, 17.0 - Z_HORN_TOP  # -36.4 .. -1.7 (local z = along output axis)
HORN_PATTERN = 4.9          # M3 pattern on the 25T output horn, 9.8 mm square about the shaft
CASE_HOLES = [(-20.3 - SHAFT_X, -10.2), (-20.3 - SHAFT_X, 10.2), (4.2 - SHAFT_X, -10.2), (4.2 - SHAFT_X, 10.2)]
WALL = 2.4                  # printed wall around a servo
CLR = 0.3                   # pocket clearance per side
PT = 4.0                    # hub plate thickness
GAP = 0.1                   # hub plate to horn face
POD_TOP = -0.4              # pod rim stops 0.4 mm short of the horn face plane
POD_OUT = (SRV_X0 - CLR - WALL, SRV_X1 + CLR + WALL, SRV_HY + CLR + WALL, -41.5)   # pod outer extents in servo-local coords


def unit(v):
    v = np.array(v, float)
    return v / np.linalg.norm(v)


class Servo:
    """An STS3215 placed by its shaft point S (centre of the horn's top face), output direction a and tail direction t."""
    def __init__(self, name, S, a, t):
        self.name = name
        self.S = np.array(S, float)
        self.a, self.t = unit(a), unit(t)
        self.e1, self.e3 = -self.t, self.a
        self.e2 = np.cross(self.e3, self.e1)
        self.R = np.stack([self.e1, self.e2, self.e3], 1)
        assert abs(np.linalg.det(self.R) - 1) < 1e-9

    def w(self, xl, yl, zl):
        return self.S + xl * self.e1 + yl * self.e2 + zl * self.e3

    def lbox(self, x0, x1, y0, y1, z0, z1):
        p = np.array([self.w(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)])
        return tuple(np.round([p[:, 0].min(), p[:, 0].max(), p[:, 1].min(), p[:, 1].max(), p[:, 2].min(), p[:, 2].max()], 4))

    def step_to_world(self):
        M = np.eye(4)
        M[:3, :3] = self.R
        M[:3, 3] = self.S - self.R @ np.array([SHAFT_X, 0, Z_HORN_TOP])
        return M

    def mirrored(self, name):
        """Mirror about the XZ plane (y -> -y). The tail and output directions are mirrored; the frame stays right-handed."""
        m = np.array([1, -1, 1.0])
        return Servo(name, self.S * m, self.a * m, self.t * m)


def box(b):
    x0, x1, y0, y1, z0, z1 = b
    return cq.Workplane('XY').box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def rbox(b, r, sel=None):
    """box with rounded edges; r is clamped so the fillet always succeeds"""
    x0, x1, y0, y1, z0, z1 = b
    m = min(x1 - x0, y1 - y0, z1 - z0)
    r = min(r, 0.49 * m)
    w = box(b)
    if r > 0.3:
        w = (w.edges(sel) if sel else w.edges()).fillet(r)
    return w


def cyl(p, d, r, h):
    return cq.Workplane('XY').add(cq.Solid.makeCylinder(r, h, cq.Vector(*map(float, p)), cq.Vector(*map(float, d))))


def axis_sel(v):
    return ('|X', '|Y', '|Z')[int(np.argmax(np.abs(v)))]


class Part:
    """A printed part = union of rounded solids minus cuts. Servos it carries are listed for mass/BOM/rendering."""
    def __init__(self, name, color='shell', link=None):
        self.name, self.color, self.link = name, color, link
        self.adds, self.cuts, self.servos = [], [], []

    def add(self, wp):
        self.adds.append(wp)

    def cut(self, wp):
        self.cuts.append(wp)

    def sleeve(self, sv, r=5.0):
        """Pod that holds servo sv: closed on 5 sides, open on the horn side; 4 M2 screws through the back wall."""
        self.servos.append(sv)
        x0, x1, hy, zb = POD_OUT
        self.add(rbox(sv.lbox(x0, x1, -hy, hy, zb, POD_TOP), r))
        self.cut(box(sv.lbox(SRV_X0 - CLR, SRV_X1 + CLR, -(SRV_HY + CLR), SRV_HY + CLR, -34.9, 3.0)))
        self.cut(cyl(sv.w(0, 0, -38.6), sv.a, 10.5, 3.8))                       # rear idler disc recess
        self.cut(box(sv.lbox(-30.5, -12.5, -9.7, 9.7, -38.6, -34.8)))            # rear pad recess
        for (x, y) in CASE_HOLES:                                                # M2 x 8 case screws + counterbores
            self.cut(cyl(sv.w(x, y, -42.0), sv.a, 1.2, 7.3))
            self.cut(cyl(sv.w(x, y, -42.0), sv.a, 2.2, 3.6))
        self.cut(box(sv.lbox(SRV_X0 - CLR - WALL - 0.1, SRV_X0 - CLR + 0.01, -5, 5, -30, -2.0)))   # cable exit in the tail wall

    def hub(self, sv, r=16.0):
        """Round hub disc on the output horn of servo sv (this part is the child that the servo turns)."""
        self.add(cyl(sv.w(0, 0, GAP), sv.a, r, PT))
        self.cut(cyl(sv.w(0, 0, -1), sv.a, 4.2, GAP + PT + 2))                   # horn screw access
        for sx in (-1, 1):
            for sy in (-1, 1):
                p = sv.w(sx * HORN_PATTERN, sy * HORN_PATTERN, -1)
                self.cut(cyl(p, sv.a, 1.7, GAP + PT + 2))                        # M3 clearance
                self.cut(cyl(sv.w(sx * HORN_PATTERN, sy * HORN_PATTERN, GAP + PT - 2.2), sv.a, 2.9, 3.0))   # head counterbore

    def build(self):
        s = self.adds[0]
        for a in self.adds[1:]:
            s = s.union(a)
        for c in self.cuts:
            s = s.cut(c)
        return s.clean().val()
