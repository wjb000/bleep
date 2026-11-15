"""Bleep: parametric design (CadQuery, mm, world frame at the zero pose).

x forward, y left, z up, soles on z = 0. Legs straight at the zero pose.
11 x Feetech STS3215 (10 leg + 1 dome yaw). 14 printed parts (7 per side are mirror pairs).
Every printed part is one link of the robot, so CAD part == sim link (except the body = tub + lid, head = shell + base).
"""
import numpy as np
import cadquery as cq
from lib import *

# ------------------------------------------------------------------ key dimensions (edit here; everything follows)
YL = 46.0                 # leg line (hip yaw / roll axis) distance from the centre plane
Y_PF = YL + 12.0          # hip-pitch horn face (outer side of the hip pod)
Y_KF = Y_PF + 4.5         # knee horn face  (each joint steps 4.5 mm out so hub plates sit flush on the next pod)
Y_AF = Y_KF + 4.5         # ankle horn face
Z_A = 27.0                # ankle axis height
SHIN = 60.0               # knee - ankle
THIGH = 65.0              # hip pitch - knee
Z_K = Z_A + SHIN
Z_P = Z_K + THIGH
Z_R = Z_P + 41.5          # hip roll axis (clears the hip pod through +-20 deg of roll)
Z_Y = Z_R + 18.7          # hip yaw horn face (underside of the body)
Z_N = Z_Y + 67.3          # dome-yaw horn face (top of the body)
X_R = 19.0                # roll horn face (front of the roll pod)
SOLE = 5.0                # sole thickness
FOOT_X = (-50.0, 46.0)    # foot length along x
FOOT_Y = (YL - 25.0, Y_AF + 4.1)
BODY_X = (-50.0, 30.0)
BODY_HY = 64.0
Z_BODY0 = Z_Y + 0.4       # underside of the body (= rim of the yaw pods)
Z_SPLIT = Z_BODY0 + 42.0  # tub / lid split line
Z_TOP = Z_N - 0.4         # top of the lid
HEAD_R = 46.0
HEAD_CYL = 36.0
HEAD_DOME = 22.0
EYE_Z = Z_N + GAP + PT - 0.5 + 20.5   # eye (round LCD) centre height
CAM_YAW, CAM_Z = 32.0, EYE_Z + 15.0  # camera lens port direction / height
CELL_Y, CELL_Z, CELL_X = 10.6, Z_BODY0 + 2.5 + 9.6, (-42.0, 23.0)    # two 18650 cells along x

LIMITS_DEG = {             # joint ranges used by CAD clearance checks, URDF/MJCF and firmware
    'hip_yaw': (-25, 25), 'hip_roll': (-20, 20), 'hip_pitch': (-90, 30), 'knee': (-5, 110), 'ankle': (-45, 45),
    'neck_yaw': (-120, 120)}


def servos_left():
    return dict(
        hip_yaw=Servo('l_hip_yaw', (0, YL, Z_Y), (0, 0, -1), (-1, 0, 0)),
        hip_roll=Servo('l_hip_roll', (X_R, YL, Z_R), (1, 0, 0), (0, -1, 0)),
        hip_pitch=Servo('l_hip_pitch', (0, Y_PF, Z_P), (0, 1, 0), (-1, 0, 0)),
        knee=Servo('l_knee', (0, Y_KF, Z_K), (0, 1, 0), (0, 0, 1)),
        ankle=Servo('l_ankle', (0, Y_AF, Z_A), (0, 1, 0), (0, 0, 1)))


NECK = Servo('neck_yaw', (0, 0, Z_N), (0, 0, 1), (-1, 0, 0))


def blob(b, r):
    return rbox(b, r)


# ------------------------------------------------------------------ leg parts (left; right = mirror)
def hip_bracket(s):
    """Child of hip yaw. Round hub under the yaw horn + pod for the roll servo hanging below it."""
    p = Part('hip_bracket', 'dark', 'hip_yaw')
    p.sleeve(s['hip_roll'], r=6.0)
    p.hub(s['hip_yaw'], r=17.0)
    return p


def hip_carrier(s):
    """Child of hip roll. Hub on the roll horn (front), a front web, and the pod for the hip-pitch servo."""
    p = Part('hip_carrier', 'dark', 'hip_roll')
    sp = s['hip_pitch']
    p.sleeve(sp, r=6.0)
    p.hub(s['hip_roll'], r=15.0)
    p.add(rbox((X_R + GAP, X_R + GAP + PT, YL - 13, YL + 11, Z_P - 6, Z_R), 4.0))          # front web
    p.add(rbox((11.0, X_R + GAP + PT, Y_PF - 38, Y_PF - 0.4, Z_P - 15.1, Z_P + 15.1), 5.0))  # web -> pod
    p.cut(box((14.0, X_R + GAP - 0.01, Y_PF - 35, Y_PF - 3.4, Z_P - 17, Z_P + 12.1)))           # hollow it (opens downward)
    return p


def thigh(s):
    """Child of hip pitch. Flat outer hub plate over the hip, flowing into the knee-servo pod."""
    p = Part('thigh', 'shell', 'hip_pitch')
    p.sleeve(s['knee'], r=7.0)
    p.hub(s['hip_pitch'], r=16.5)
    y0, y1 = Y_PF + GAP, Y_PF + GAP + PT
    p.add(rbox((-15.1, 15.1, y0, y1, Z_K + 20, Z_P), 6.0))                  # outer plate (flush with the knee pod)
    p.add(rbox((-15.1, 15.1, Y_KF - 9, y1, Z_K + 20, Z_K + 40.5), 6.0))      # plate -> pod root
    return p


def shin(s):
    p = Part('shin', 'shell', 'knee')
    p.sleeve(s['ankle'], r=7.0)
    p.hub(s['knee'], r=16.5)
    y0, y1 = Y_KF + GAP, Y_KF + GAP + PT
    p.add(rbox((-15.1, 15.1, y0, y1, Z_A + 20, Z_K), 6.0))
    p.add(rbox((-15.1, 15.1, Y_AF - 9, y1, Z_A + 20, Z_A + 40.5), 6.0))
    return p


def foot(s):
    """Child of the ankle: outer hub plate + a rounded sole pad with a raised toe bumper."""
    p = Part('foot', 'dark', 'ankle')
    p.hub(s['ankle'], r=14.0)
    y0, y1 = Y_AF + GAP, Y_AF + GAP + PT
    p.add(rbox((-14, 14, y0, y1, 2.0, Z_A), 5.0))
    x0, x1 = FOOT_X
    yy0, yy1 = FOOT_Y
    sole = (cq.Workplane('XY').box(x1 - x0, yy1 - yy0, SOLE, centered=False).translate((x0, yy0, 0))
            .edges('|Z').fillet(18.0).edges('>Z').fillet(2.5))
    p.add(sole)
    toe = (cq.Workplane('XY').box(16, yy1 - yy0 - 8, 7, centered=False).translate((x1 - 22, yy0 + 4, SOLE - 1))
           .edges().fillet(3.0))
    p.add(toe)
    for xc in (x0 + 18, x1 - 30):     # weight-saving pockets in the sole top (open upward, no supports)
        p.cut(rbox((xc - 10, xc + 10, yy0 + 7, yy1 - 9, 2.0, SOLE + 1), 4.0, '|Z'))
    return p


# ------------------------------------------------------------------ body
def body_shape(z0, z1, inset=0.0):
    x0, x1 = BODY_X
    w = (cq.Workplane('XY').box(x1 - x0 - 2 * inset, 2 * (BODY_HY - inset), z1 - z0, centered=False)
         .translate((x0 + inset, -BODY_HY + inset, z0)))
    return w


def body_tub(sl, sr):
    """Lower body: floor + walls, carries both hip-yaw servos, the two cells and the electronics. Prints floor-down."""
    p = Part('body_tub', 'shell', 'base')
    outer = body_shape(Z_BODY0, Z_SPLIT).edges('|Z').fillet(20.0).faces('<Z').edges().fillet(10.0)
    inner = body_shape(Z_BODY0 + 2.4, Z_SPLIT + 1, 2.4).edges('|Z').fillet(17.6).faces('<Z').edges().fillet(7.6)
    p.add(outer)
    p.cut(inner)
    p.sleeve(sl['hip_yaw'], r=3.0)
    p.sleeve(sr['hip_yaw'], r=3.0)
    for sv in (sl['hip_yaw'], sr['hip_yaw']):          # horn openings through the floor
        p.cut(cyl(sv.w(0, 0, -2), (0, 0, 1), 12.0, 6))
    for sy in (-1, 1):                                    # cell cradles (half pipes on the floor)
        y = sy * CELL_Y
        c = (cq.Workplane('YZ').circle(11.6).extrude(CELL_X[1] - CELL_X[0] - 20).translate((CELL_X[0] + 10, y, CELL_Z)))
        c = c.intersect(box((CELL_X[0] + 10, CELL_X[1] - 10, y - 12, y + 12, Z_BODY0 + 1, CELL_Z - 3)))
        p.add(box((CELL_X[0] + 10, CELL_X[1] - 10, y - 11.5, y + 11.5, Z_BODY0 + 1, CELL_Z - 3)).cut(c))
    # rear service panel: USB-C / charge jack + power switch openings
    p.cut(rbox((BODY_X[0] - 1, BODY_X[0] + 4, -20, -8, Z_BODY0 + 14, Z_BODY0 + 24), 2.0, '|X'))
    p.cut(cyl((BODY_X[0] - 1, 14, Z_BODY0 + 19), (1, 0, 0), 4.1, 6))
    # lid screw bosses (M3 heat-set) at the 4 inner corners
    for (x, y) in ((BODY_X[0] + 9, -BODY_HY + 9), (BODY_X[0] + 9, BODY_HY - 9), (BODY_X[1] - 9, -BODY_HY + 9), (BODY_X[1] - 9, BODY_HY - 9)):
        p.add(cyl((x, y, Z_SPLIT - 14), (0, 0, 1), 4.0, 14))
        p.cut(cyl((x, y, Z_SPLIT - 8), (0, 0, 1), 2.1, 9))
    # side accent grooves (cosmetic)
    for sy in (-1, 1):
        p.cut(box((BODY_X[0] + 12, BODY_X[1] - 12, sy * BODY_HY - 0.8, sy * BODY_HY + 0.8, Z_BODY0 + 14, Z_BODY0 + 15.2)))
    return p


def body_lid(n):
    """Upper body: rounded shoulders, carries the dome-yaw servo. Prints top-down (flat lid face on the bed)."""
    p = Part('body_lid', 'accent', 'base')
    outer = body_shape(Z_SPLIT, Z_TOP).edges('|Z').fillet(20.0).faces('>Z').edges().fillet(12.0)
    inner = body_shape(Z_SPLIT - 1, Z_TOP - 2.4, 2.4).edges('|Z').fillet(17.6).faces('>Z').edges().fillet(9.6)
    p.add(outer)
    p.cut(inner)
    p.sleeve(n, r=3.0)
    p.cut(cyl(n.w(0, 0, -3), (0, 0, 1), 12.0, 6))
    for (x, y) in ((BODY_X[0] + 9, -BODY_HY + 9), (BODY_X[0] + 9, BODY_HY - 9), (BODY_X[1] - 9, -BODY_HY + 9), (BODY_X[1] - 9, BODY_HY - 9)):
        p.add(cyl((x, y, Z_SPLIT), (0, 0, 1), 4.0, Z_TOP - Z_SPLIT - 1))
        p.cut(cyl((x, y, Z_SPLIT - 1), (0, 0, 1), 1.7, Z_TOP - Z_SPLIT))
        p.cut(cyl((x, y, Z_TOP - 4), (0, 0, 1), 3.0, 6))
    for sy in (-1, 1):                                    # wire pass-throughs from the lid to the head
        p.cut(cyl((-20, sy * 18, Z_TOP - 5), (0, 0, 1), 3.5, 8))
    return p


# ------------------------------------------------------------------ head
def xiao_pose():
    """XIAO ESP32S3 Sense: board centre and outward normal (camera looks along it through the lens port)"""
    nrm = np.array([np.cos(np.radians(CAM_YAW)), np.sin(np.radians(CAM_YAW)), 0.0])
    return nrm * (HEAD_R - 17.0) + np.array([0, 0, CAM_Z - 2.0]), nrm


def head_shell():
    """Dome head: cylinder + elliptical dome, round eye window, camera lens port, antenna boss. Prints open-side down."""
    p = Part('head_shell', 'shell', 'head')
    z0 = Z_N + GAP + PT - 0.5
    cylw = cq.Workplane('XY').circle(HEAD_R).extrude(HEAD_CYL).translate((0, 0, z0))
    dome = cq.Workplane('XY').add(cq.Solid.makeSphere(HEAD_R, angleDegrees1=0, angleDegrees2=90)).val().scale(1.0)
    dome = cq.Workplane('XY').add(dome).val()
    sc = HEAD_DOME / HEAD_R
    m = cq.Matrix(); 
    dome = cq.Workplane('XY').add(dome.transformGeometry(cq.Matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, sc, 0]]))).translate((0, 0, z0 + HEAD_CYL))
    outer = cylw.union(dome)
    ri = HEAD_R - 2.2
    cyli = cq.Workplane('XY').circle(ri).extrude(HEAD_CYL + 0.01).translate((0, 0, z0 - 0.01))
    domei = cq.Workplane('XY').add(cq.Solid.makeSphere(ri, angleDegrees1=0, angleDegrees2=90))
    domei = cq.Workplane('XY').add(domei.val().transformGeometry(cq.Matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, (HEAD_DOME - 2.2) / ri, 0]]))).translate((0, 0, z0 + HEAD_CYL))
    p.add(outer)
    p.cut(cyli.union(domei))
    # eye: round window for the 1.28" GC9A01 (active area 32.4 mm) with a raised bezel ring
    bez = cyl((HEAD_R - 3.0, 0, EYE_Z), (1, 0, 0), 20.0, 5.0)
    p.add(bez)
    p.cut(cyl((HEAD_R - 6, 0, EYE_Z), (1, 0, 0), 17.0, 14))
    p.cut(cyl((HEAD_R + 0.8, 0, EYE_Z), (1, 0, 0), 18.2, 2))          # bezel recess
    # camera lens port (XIAO ESP32S3 Sense camera) with a small hood, upper left of the eye
    nrm = np.array([np.cos(np.radians(CAM_YAW)), np.sin(np.radians(CAM_YAW)), 0.0])
    cam = tuple(nrm * (HEAD_R - 2.0) + np.array([0, 0, CAM_Z]))
    p.add(cyl(np.array(cam) - nrm * 1.5, nrm, 7.5, 5.0))
    p.cut(cyl(np.array(cam) - nrm * 6, nrm, 4.2, 14))
    # little indicator port on the right (status LED light pipe)
    p.cut(cyl((HEAD_R * np.cos(np.radians(-35)) - 4, HEAD_R * np.sin(np.radians(-35)), EYE_Z + 9), (np.cos(np.radians(-35)), np.sin(np.radians(-35)), 0), 2.0, 8))
    # antenna boss on the dome
    p.add(cyl((-16, -16, z0 + HEAD_CYL + 12), (0, 0, 1), 2.6, 8))                # stub antenna (light pipe)
    p.add(cq.Workplane('XY').sphere(3.4).translate((-16, -16, z0 + HEAD_CYL + 20)))
    # dome panel lines (cosmetic grooves): ring at the dome seam
    ring = cq.Workplane('XY').circle(HEAD_R + 1).circle(HEAD_R - 0.8).extrude(1.2).translate((0, 0, z0 + HEAD_CYL - 3))
    p.cut(ring)
    # mounting lugs inside the lower rim (3 x M2 into the head base)
    for ang in (90, 210, 330):
        a = np.radians(ang)
        c = (np.cos(a) * (HEAD_R - 5.5), np.sin(a) * (HEAD_R - 5.5), z0)
        p.add(cyl(c, (0, 0, 1), 4.2, 8))
        p.cut(cyl(c, (0, 0, 1), 1.6, 8))
    return p


def head_base(n):
    """Disc that bolts to the dome-yaw horn and carries the XIAO ESP32S3 Sense, the eye LCD and the 5 V buck."""
    p = Part('head_base', 'accent', 'head')
    z0 = Z_N + GAP
    p.hub(n, r=18.0)
    p.add(cq.Workplane('XY').circle(HEAD_R - 2.6).extrude(3.0).translate((0, 0, z0)))
    p.add(cq.Workplane('XY').circle(HEAD_R + 0.6).circle(HEAD_R - 2.6).extrude(PT - 0.5).translate((0, 0, z0)))   # trim ring under the shell
    # eye LCD cradle (vertical plate with a 38 mm round seat), XIAO cradle behind the camera
    p.add(rbox((HEAD_R - 13, HEAD_R - 10, -21, 21, z0, EYE_Z + 4), 1.0))
    p.cut(cyl((HEAD_R - 14, 0, EYE_Z), (1, 0, 0), 19.5, 6))
    xp = xiao_pose()[0]
    p.add(rbox((xp[0] - 4, xp[0] + 4, xp[1] - 4, xp[1] + 4, z0, xp[2] - 9.5), 1.5))     # XIAO post (board screws to it)
    p.cut(cyl(n.w(0, 0, 0), (0, 0, 1), 4.2, 10))
    for (x, y) in ((-22, 22), (-22, -22)):                 # harness pass-through
        p.cut(cyl((x, y, z0 - 1), (0, 0, 1), 4.0, 6))
    for ang in (90, 210, 330):
        a = np.radians(ang)
        p.cut(cyl((np.cos(a) * (HEAD_R - 5.5), np.sin(a) * (HEAD_R - 5.5), z0 - 1), (0, 0, 1), 1.2, 6))
    return p


# ------------------------------------------------------------------ assembly
def build_parts():
    sl = servos_left()
    sr = {k: v.mirrored(v.name.replace('l_', 'r_')) for k, v in sl.items()}
    left = [hip_bracket(sl), hip_carrier(sl), thigh(sl), shin(sl), foot(sl)]
    parts = []
    for p in left:
        solid = p.build()
        parts.append(dict(name='l_' + p.name, color=p.color, link='l_' + p.link, solid=solid,
                          servos=[s.name for s in p.servos]))
        parts.append(dict(name='r_' + p.name, color=p.color, link='r_' + p.link, solid=solid.mirror('XZ'),
                          servos=[s.name.replace('l_', 'r_') for s in p.servos]))
    for p in (body_tub(sl, sr), body_lid(NECK), head_shell(), head_base(NECK)):
        parts.append(dict(name=p.name, color=p.color, link=p.link, solid=p.build(), servos=[s.name for s in p.servos]))
    servos = list(sl.values()) + list(sr.values()) + [NECK]
    return parts, servos


# joints: name -> (parent link, child link, origin (mm, world at zero pose), axis)
def joints():
    J = {}
    for side, sg in (('l', 1), ('r', -1)):
        J[f'{side}_hip_yaw'] = ('base', f'{side}_hip_yaw', (0, sg * YL, Z_Y), (0, 0, 1))
        J[f'{side}_hip_roll'] = (f'{side}_hip_yaw', f'{side}_hip_roll', (X_R, sg * YL, Z_R), (1, 0, 0))
        J[f'{side}_hip_pitch'] = (f'{side}_hip_roll', f'{side}_hip_pitch', (0, sg * Y_PF, Z_P), (0, 1, 0))
        J[f'{side}_knee'] = (f'{side}_hip_pitch', f'{side}_knee', (0, sg * Y_KF, Z_K), (0, 1, 0))
        J[f'{side}_ankle'] = (f'{side}_knee', f'{side}_ankle', (0, sg * Y_AF, Z_A), (0, 1, 0))
    J['neck_yaw'] = ('base', 'head', (0, 0, Z_N), (0, 0, 1))
    return J


def limit_deg(jname):
    for k, v in LIMITS_DEG.items():
        if jname.endswith(k):
            return v
    raise KeyError(jname)
