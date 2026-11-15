"""
KMS Ream Stacker - parametric model + baked animation + AR export (GLB / USDZ)

Run headless:
  blender --background --factory-startup --python build_stacker.py -- <output_dir>

Units: metres, Z up (exporters convert to Y up). Origin = floor, machine centre.
Layout:
  * Column (stepper, lead screw, guide rods) at the back (+Y).
  * Fork-lift cradle cantilevers forward (-Y); two tines straddle the belt.
  * Transfer belt runs along -Y (towards the viewer) and feeds an outfeed roller table.
"""
import bpy, bmesh, math, os, sys, json, struct
from mathutils import Vector

# ----------------------------------------------------------------------------
# Parameters (confirmed with user)
# ----------------------------------------------------------------------------
REAM_L, REAM_W, REAM_H = 0.297, 0.210, 0.052     # A4 ream, 500 x 80 g/m2
N_REAMS = 5
STEP = 0.052                                      # cradle drop per ream
LEAD = 0.008                                      # T8 lead screw, 8 mm lead
BELT_W = 0.140
ROLLER_R = 0.020
BELT_T = 0.003
ROLLER_Z = 0.080
ROLLER_Y_BACK, ROLLER_Y_FRONT = 0.11, -0.27
BELT_TOP = ROLLER_Z + ROLLER_R + BELT_T           # 0.103
Z_LOAD = 0.36                                     # tine top height when empty
Z_HANDOVER = BELT_TOP - 0.015                     # tines finish 15 mm below belt
TINE_X, TINE_W, TINE_T = 0.100, 0.030, 0.015
STACK_Y = 0.0                                     # stack centre (Y) while loading
OUT_TRAVEL_TARGET = 0.55                          # stack travel onto outfeed

FPS = 24
T_TOTAL = 28.0
FRAME_END = int(T_TOTAL * FPS)                    # 672 (== frame 0 state -> seamless loop)

OUT_DIR = sys.argv[sys.argv.index("--") + 1] if "--" in sys.argv else os.path.dirname(__file__)
os.makedirs(OUT_DIR, exist_ok=True)

# ----------------------------------------------------------------------------
# Scene reset
# ----------------------------------------------------------------------------
bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.render.fps = FPS
scene.frame_start = 0
scene.frame_end = FRAME_END
scene.unit_settings.system = 'METRIC'
scene.unit_settings.scale_length = 1.0

# ----------------------------------------------------------------------------
# Materials
# ----------------------------------------------------------------------------
def material(name, rgb, metal=0.0, rough=0.5, emit=None, emit_strength=0.0, alpha=1.0):
    m = bpy.data.materials.new(name)
    try:
        m.use_nodes = True
    except Exception:
        pass
    m.diffuse_color = (*rgb, alpha)
    bsdf = m.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = (*rgb, 1.0)
    bsdf.inputs["Metallic"].default_value = metal
    bsdf.inputs["Roughness"].default_value = rough
    if emit is not None:
        key = "Emission Color" if "Emission Color" in bsdf.inputs else "Emission"
        bsdf.inputs[key].default_value = (*emit, 1.0)
        bsdf.inputs["Emission Strength"].default_value = emit_strength
    if alpha < 1.0:
        bsdf.inputs["Alpha"].default_value = alpha
        for attr, val in (("surface_render_method", "BLENDED"), ("blend_method", "BLEND")):
            try:
                setattr(m, attr, val)
            except Exception:
                pass
    return m

M_ALU     = material("Aluminium",   (0.80, 0.81, 0.83), metal=1.0, rough=0.35)
M_ALU_DK  = material("AluDark",     (0.55, 0.56, 0.58), metal=1.0, rough=0.45)
M_STEEL   = material("Steel",       (0.70, 0.70, 0.72), metal=1.0, rough=0.25)
M_BLACK   = material("BlackAnod",   (0.03, 0.03, 0.035), metal=0.3, rough=0.45)
M_BRASS   = material("Brass",       (0.85, 0.62, 0.28), metal=1.0, rough=0.3)
M_BELT    = material("BeltGreen",   (0.05, 0.32, 0.17), rough=0.8)
M_CLEAT   = material("BeltCleat",   (0.03, 0.22, 0.11), rough=0.8)
M_RUBBER  = material("Rubber",      (0.02, 0.02, 0.02), rough=0.9)
M_PAPER   = material("ReamWrap",    (0.93, 0.94, 0.95), rough=0.6)
M_BAND    = material("ReamBand",    (0.10, 0.45, 0.75), rough=0.5)
M_SENSOR  = material("SensorBody",  (0.12, 0.12, 0.14), rough=0.5)
M_LED_OFF = material("LedOff",      (0.25, 0.02, 0.02), rough=0.3)
M_LED_ON  = material("LedOn",       (1.0, 0.1, 0.05), rough=0.3, emit=(1.0, 0.1, 0.05), emit_strength=8.0)
M_BEAM    = material("IRBeam",      (1.0, 0.15, 0.1), rough=0.5, emit=(1.0, 0.15, 0.1), emit_strength=3.0, alpha=0.45)
M_LABEL   = material("Label",       (0.02, 0.05, 0.14), rough=0.6)
M_LABEL_HI= material("LabelHi",     (0.85, 0.35, 0.05), rough=0.5, emit=(0.85, 0.35, 0.05), emit_strength=0.6)
M_LEADER  = material("Leader",      (0.25, 0.30, 0.40), rough=0.5)

# ----------------------------------------------------------------------------
# Geometry helpers
# ----------------------------------------------------------------------------
COLL = bpy.data.collections.new("KMS_Stacker")
scene.collection.children.link(COLL)

def finish(name, verts, faces, mat, smooth_faces=None, parent=None):
    me = bpy.data.meshes.new(name)
    me.from_pydata(verts, [], faces)
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me)
    bm.free()
    if smooth_faces is not None:
        for p in me.polygons:
            p.use_smooth = p.index in smooth_faces
    me.materials.append(mat)
    ob = bpy.data.objects.new(name, me)
    COLL.objects.link(ob)
    if parent:
        ob.parent = parent
    return ob

def map3(axis, u, v, w):
    """2D cross-section (u,v) + position w along axis -> xyz."""
    if axis == 'Z':
        return (u, v, w)
    if axis == 'Y':
        return (u, w, v)
    return (w, u, v)  # 'X'

def prism(name, poly, axis, center, length, mat, smooth_sides=False, parent=None):
    cx, cy, cz = center
    off = {'X': cx, 'Y': cy, 'Z': cz}[axis]
    n = len(poly)
    verts = []
    for w in (off - length / 2, off + length / 2):
        for (u, v) in poly:
            x, y, z = map3(axis, u, v, w)
            # add the cross-section centre offsets
            if axis == 'Z':
                x, y = x + cx, y + cy
            elif axis == 'Y':
                x, z = x + cx, z + cz
            else:
                y, z = y + cy, z + cz
            verts.append((x, y, z))
    faces = [list(range(n))[::-1], list(range(n, 2 * n))]
    for i in range(n):
        j = (i + 1) % n
        faces.append([i, j, n + j, n + i])
    smooth = set(range(2, 2 + n)) if smooth_sides else set()
    return finish(name, verts, faces, mat, smooth, parent)

def rect(a, b):
    return [(-a / 2, -b / 2), (a / 2, -b / 2), (a / 2, b / 2), (-a / 2, b / 2)]

def circle(r, seg=24):
    return [(r * math.cos(2 * math.pi * i / seg), r * math.sin(2 * math.pi * i / seg)) for i in range(seg)]

def tslot(a, b, module=None):
    """T-slot style cross-section (simplified slot) a x b."""
    module = module or min(a, b)
    slot = 0.008 if module >= 0.04 else 0.006
    depth = 0.004 if module >= 0.04 else 0.003
    corners = [(-a / 2, -b / 2), (a / 2, -b / 2), (a / 2, b / 2), (-a / 2, b / 2)]
    pts = []
    for k in range(4):
        p0, p1 = Vector(corners[k]), Vector(corners[(k + 1) % 4])
        d = p1 - p0
        L = d.length
        d.normalize()
        nrm = Vector((-d.y, d.x))
        pts.append(tuple(p0))
        nmod = max(1, round(L / module))
        for m in range(nmod):
            c = (m + 0.5) * L / nmod
            for (t, dep) in ((c - slot / 2, 0), (c - slot / 2, depth), (c + slot / 2, depth), (c + slot / 2, 0)):
                q = p0 + d * t + nrm * dep
                pts.append((q.x, q.y))
    return pts

def box(name, center, size, mat, parent=None):
    sx, sy, sz = size
    return prism(name, rect(sx, sy), 'Z', center, sz, mat, parent=parent)

def cyl(name, center, r, length, axis, mat, seg=24, parent=None):
    return prism(name, circle(r, seg), axis, center, length, mat, smooth_sides=True, parent=parent)

def profile(name, center, length, axis, a, b, mat=M_ALU):
    return prism(name, tslot(a, b), axis, center, length, mat)

def empty(name, loc=(0, 0, 0), parent=None):
    e = bpy.data.objects.new(name, None)
    e.empty_display_size = 0.05
    e.location = loc
    COLL.objects.link(e)
    if parent:
        e.parent = parent
    return e

def join(objs, name):
    """Join meshes (static parts) to keep node count low."""
    base = objs[0]
    me = bpy.data.meshes.new(name)
    bm = bmesh.new()
    mats = []
    for ob in objs:
        tmp = ob.data.copy()
        tmp.transform(ob.matrix_basis)
        for p in tmp.polygons:
            pass
        m_idx = {}
        for i, m in enumerate(tmp.materials):
            if m not in mats:
                mats.append(m)
            m_idx[i] = mats.index(m)
        start = len(bm.faces)
        bm.from_mesh(tmp)
        bm.faces.ensure_lookup_table()
        for f in bm.faces[start:]:
            f.material_index = m_idx.get(f.material_index, 0)
        bpy.data.meshes.remove(tmp)
    bm.to_mesh(me)
    bm.free()
    for m in mats:
        me.materials.append(m)
    parent = base.parent
    for ob in objs:
        bpy.data.objects.remove(ob)
    ob = bpy.data.objects.new(name, me)
    COLL.objects.link(ob)
    if parent:
        ob.parent = parent
    return ob

def text(name, body, loc, size, mat, align='CENTER', parent=None, extrude=0.0012):
    cu = bpy.data.curves.new(name + "_curve", type='FONT')
    cu.body = body
    cu.size = size
    cu.extrude = extrude
    cu.align_x = align
    cu.align_y = 'CENTER'
    tmp = bpy.data.objects.new(name + "_tmp", cu)
    COLL.objects.link(tmp)
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(tmp.evaluated_get(dg))
    bpy.data.objects.remove(tmp)
    bpy.data.curves.remove(cu)
    me.materials.clear()
    me.materials.append(mat)
    ob = bpy.data.objects.new(name, me)
    ob.location = loc
    ob.rotation_euler = (math.pi / 2, 0, 0)   # face the front (-Y)
    COLL.objects.link(ob)
    if parent:
        ob.parent = parent
    return ob

def leader(name, p0, p1, parent=None):
    p0, p1 = Vector(p0), Vector(p1)
    mid = (p0 + p1) / 2
    L = (p1 - p0).length
    ob = cyl(name, (0, 0, 0), 0.0012, L, 'Z', M_LEADER, seg=6, parent=parent)
    ob.location = mid
    ob.rotation_mode = 'QUATERNION'
    ob.rotation_quaternion = Vector((0, 0, 1)).rotation_difference((p1 - p0).normalized())
    return ob

# ----------------------------------------------------------------------------
# Static frame
# ----------------------------------------------------------------------------
static = []
RAIL_X = 0.18
# base 40x80 rails along Y (80 tall)
for sx in (-1, 1):
    static.append(profile(f"BaseRail{sx}", (sx * RAIL_X, 0.0, 0.04), 0.60, 'Y', 0.04, 0.08))
# base cross members 40x40 (low, under the belt)
static.append(profile("BaseCrossFront", (0, -0.28, 0.02), 0.32, 'X', 0.04, 0.04))
static.append(profile("BaseCrossBack", (0, 0.22, 0.02), 0.32, 'X', 0.04, 0.04))
# column uprights 40x40 on the rails + top beam
for sx in (-1, 1):
    static.append(profile(f"Upright{sx}", (sx * RAIL_X, 0.26, 0.37), 0.58, 'Z', 0.04, 0.04))
static.append(profile("TopBeam", (0, 0.26, 0.68), 0.40, 'X', 0.04, 0.04))
# motor plate on top of the beam
static.append(box("MotorPlate", (0, 0.23, 0.703), (0.16, 0.10, 0.006), M_ALU_DK))
# guide rods + bottom bearing block
SCREW_Y = 0.215
for sx in (-1, 1):
    static.append(cyl(f"GuideRod{sx}", (sx * 0.06, SCREW_Y, 0.37), 0.006, 0.66, 'Z', M_STEEL))
    static.append(box(f"RodFoot{sx}", (sx * 0.06, SCREW_Y, 0.045), (0.03, 0.03, 0.01), M_BLACK))
static.append(box("BearingBlock", (0, SCREW_Y, 0.045), (0.04, 0.03, 0.01), M_BLACK))
# stepper NEMA17
static.append(box("StepperBody", (0, SCREW_Y, 0.706 + 0.024), (0.042, 0.042, 0.048), M_BLACK))
static.append(box("StepperCap", (0, SCREW_Y, 0.706 + 0.050), (0.040, 0.040, 0.004), M_ALU_DK))
static.append(box("StepperConn", (0.024, SCREW_Y, 0.74), (0.006, 0.016, 0.008), M_ALU_DK))
# belt: rollers' shafts, side plates, motor
for y in (ROLLER_Y_BACK, ROLLER_Y_FRONT):
    static.append(cyl(f"Shaft{y:+.2f}", (0, y, ROLLER_Z), 0.004, 0.32, 'X', M_STEEL, seg=12))
static.append(cyl("BeltMotor", (0.235, ROLLER_Y_FRONT, ROLLER_Z), 0.022, 0.07, 'X', M_BLACK))
static.append(cyl("BeltGearbox", (0.21, ROLLER_Y_FRONT, ROLLER_Z), 0.016, 0.02, 'X', M_ALU_DK))

# belt loop solid (stadium in YZ)
def stadium_pts(r, seg=14):
    pts = []
    for i in range(seg + 1):  # back arc, bottom -> top on +Y side
        a = -math.pi / 2 + math.pi * i / seg
        pts.append((ROLLER_Y_BACK + r * math.cos(a), ROLLER_Z + r * math.sin(a)))
    for i in range(seg + 1):  # front arc, top -> bottom on -Y side
        a = math.pi / 2 + math.pi * i / seg
        pts.append((ROLLER_Y_FRONT + r * math.cos(a), ROLLER_Z + r * math.sin(a)))
    return pts

def belt_loop():
    outer = stadium_pts(ROLLER_R + BELT_T)
    inner = stadium_pts(ROLLER_R)
    n = len(outer)
    verts = []
    for x in (-BELT_W / 2, BELT_W / 2):
        verts += [(x, y, z) for (y, z) in outer]
        verts += [(x, y, z) for (y, z) in inner]
    def idx(side, ring, i):
        return side * 2 * n + ring * n + (i % n)
    faces = []
    for i in range(n):
        faces.append([idx(0, 0, i), idx(0, 0, i + 1), idx(1, 0, i + 1), idx(1, 0, i)])   # outer
        faces.append([idx(0, 1, i), idx(1, 1, i), idx(1, 1, i + 1), idx(0, 1, i + 1)])   # inner
        faces.append([idx(0, 0, i), idx(0, 1, i), idx(0, 1, i + 1), idx(0, 0, i + 1)])   # side L
        faces.append([idx(1, 0, i), idx(1, 0, i + 1), idx(1, 1, i + 1), idx(1, 1, i)])   # side R
    return finish("Belt", verts, faces, M_BELT)

static.append(belt_loop())

# IR sensor post (20x20) on left rail, sensor head facing +X across the stack
IR_Y = -0.12
IR_Z = Z_LOAD + REAM_H - 0.02              # just under the top of a freshly loaded ream
static.append(profile("SensorPost", (-RAIL_X, IR_Y, 0.08 + 0.20), 0.40, 'Z', 0.02, 0.02))
static.append(box("SensorBracket", (-RAIL_X + 0.015, IR_Y, IR_Z), (0.02, 0.025, 0.004), M_ALU_DK))
static.append(box("SensorBody", (-RAIL_X + 0.03, IR_Y, IR_Z), (0.018, 0.03, 0.022), M_SENSOR))
static.append(cyl("SensorLens", (-RAIL_X + 0.04, IR_Y, IR_Z), 0.005, 0.004, 'X', M_RUBBER, seg=12))

# outfeed gravity-roller table
OUT_Y0, OUT_Y1 = -0.30, -0.76
OUT_RX = 0.14
OUT_ROLLER_R = 0.015
OUT_ROLLER_Z = BELT_TOP - OUT_ROLLER_R
for sx in (-1, 1):
    static.append(profile(f"OutRail{sx}", (sx * OUT_RX, (OUT_Y0 + OUT_Y1) / 2, 0.085), abs(OUT_Y1 - OUT_Y0), 'Y', 0.02, 0.04))
    for y in (OUT_Y0 - 0.03, OUT_Y1 + 0.03):
        static.append(profile(f"OutLeg{sx}{y:+.2f}", (sx * OUT_RX, y, 0.0325), 0.065, 'Z', 0.02, 0.02))

frame_static = join(static, "Frame_Static")

# ----------------------------------------------------------------------------
# Moving parts
# ----------------------------------------------------------------------------
# --- lead screw (helical thread so rotation is visible)
def lead_screw(z0, z1, r_core=0.0032, r_out=0.004, pitch=LEAD / 2, seg=12):
    verts, faces = [], []
    turns = (z1 - z0) / pitch
    nsteps = int(turns * seg)
    w = pitch * 0.3
    for k in range(nsteps + 1):
        a = 2 * math.pi * k / seg
        z = z0 + pitch * k / seg
        c, s = math.cos(a), math.sin(a)
        verts += [(r_core * c, r_core * s, z - w), (r_out * c, r_out * s, z), (r_core * c, r_core * s, z + w)]
    for k in range(nsteps):
        b0, b1 = 3 * k, 3 * (k + 1)
        faces.append([b0, b1, b1 + 1, b0 + 1])
        faces.append([b0 + 1, b1 + 1, b1 + 2, b0 + 2])
    thread = finish("ScrewThread", verts, faces, M_STEEL, smooth_faces=set(range(len(faces))))
    core = cyl("ScrewCore", (0, 0, (z0 + z1) / 2), r_core, z1 - z0, 'Z', M_STEEL, seg=12)
    coupler = cyl("Coupler", (0, 0, z1 + 0.012), 0.009, 0.024, 'Z', M_ALU_DK, seg=16)
    grub = box("CouplerGrub", (0.009, 0, z1 + 0.012), (0.004, 0.004, 0.004), M_BLACK)
    return join([thread, core, coupler, grub], "LeadScrew")

screw = lead_screw(0.05, 0.676)
screw.location = (0, SCREW_Y, 0)
screw_empty = None  # rotate the mesh object directly (origin on its axis)

# --- fork-lift cradle (children of an empty that moves in Z); local z=0 is the tine top surface
cradle = empty("Cradle", (0, 0, Z_LOAD))
cr_parts = []
TINE_Y0, TINE_Y1 = -0.155, 0.195
for sx in (-1, 1):
    cr_parts.append(box(f"Tine{sx}", (sx * TINE_X, (TINE_Y0 + TINE_Y1) / 2, -TINE_T / 2),
                        (TINE_W, TINE_Y1 - TINE_Y0, TINE_T), M_ALU))
    cr_parts.append(box(f"TineTip{sx}", (sx * TINE_X, TINE_Y0 + 0.01, -TINE_T / 2 - 0.0005),
                        (TINE_W + 0.001, 0.02, TINE_T + 0.001), M_RUBBER))
cr_parts.append(box("CrossBar", (0, 0.175, -0.015), (2 * TINE_X + TINE_W, 0.04, 0.03), M_ALU))
cr_parts.append(box("Carriage", (0, 0.215, 0.0), (0.16, 0.04, 0.06), M_ALU_DK))
for sx in (-1, 1):
    cr_parts.append(cyl(f"Bushing{sx}", (sx * 0.06, SCREW_Y, 0.0), 0.011, 0.064, 'Z', M_BLACK, seg=16))
cr_parts.append(cyl("LeadNut", (0, SCREW_Y, 0.036), 0.011, 0.014, 'Z', M_BRASS, seg=6))
cr_parts.append(cyl("LeadNutFlange", (0, SCREW_Y, 0.031), 0.016, 0.004, 'Z', M_BRASS, seg=16))
cradle_mesh = join(cr_parts, "Cradle_Mesh")
cradle_mesh.parent = cradle

# --- belt rollers (rotate about X); bolt heads make rotation visible
def roller(name, y, z, r, length, mat_body, bolts_x):
    parts = [cyl(name + "_body", (0, 0, 0), r, length, 'X', mat_body, seg=24)]
    for k in range(3):
        a = 2 * math.pi * k / 3
        for bx in bolts_x:
            parts.append(cyl(name + f"_bolt{k}{bx}", (bx, r * 0.55 * math.cos(a), r * 0.55 * math.sin(a)),
                             r * 0.15, 0.004, 'X', M_BLACK, seg=6))
    ob = join(parts, name)
    ob.location = (0, y, z)
    return ob

belt_rollers = [roller(f"BeltRoller{i}", y, ROLLER_Z, ROLLER_R - 0.0006, BELT_W + 0.01, M_ALU, (-(BELT_W + 0.01) / 2 - 0.002, (BELT_W + 0.01) / 2 + 0.002))
                for i, y in enumerate((ROLLER_Y_BACK, ROLLER_Y_FRONT))]
out_rollers = []
y = OUT_Y0 - 0.025
i = 0
while y > OUT_Y1 + 0.02:
    out_rollers.append(roller(f"OutRoller{i}", y, OUT_ROLLER_Z, OUT_ROLLER_R, 2 * OUT_RX - 0.02, M_ALU,
                              (-(OUT_RX - 0.01) - 0.002, (OUT_RX - 0.01) + 0.002)))
    y -= 0.05
    i += 1

# --- belt cleats running around the stadium path
CLEAT_R = ROLLER_R + BELT_T + 0.0022
L_TOP = ROLLER_Y_BACK - ROLLER_Y_FRONT
PERIM = 2 * L_TOP + 2 * math.pi * CLEAT_R
N_CLEATS = 16
CLEAT_PITCH = PERIM / N_CLEATS
OUT_TRAVEL = round(OUT_TRAVEL_TARGET / CLEAT_PITCH) * CLEAT_PITCH   # seamless loop
cleats = [box(f"Cleat{k}", (0, 0, 0), (BELT_W - 0.006, 0.006, 0.004), M_CLEAT) for k in range(N_CLEATS)]

def cleat_pose(s):
    s %= PERIM
    if s < L_TOP:
        return ROLLER_Y_BACK - s, ROLLER_Z + CLEAT_R, 0.0
    s -= L_TOP
    if s < math.pi * CLEAT_R:
        f = s / CLEAT_R
        return ROLLER_Y_FRONT - CLEAT_R * math.sin(f), ROLLER_Z + CLEAT_R * math.cos(f), f
    s -= math.pi * CLEAT_R
    if s < L_TOP:
        return ROLLER_Y_FRONT + s, ROLLER_Z - CLEAT_R, math.pi
    s -= L_TOP
    f = s / CLEAT_R
    return ROLLER_Y_BACK + CLEAT_R * math.sin(f), ROLLER_Z - CLEAT_R * math.cos(f), math.pi + f

# --- reams
def ream(i):
    h = REAM_H - 0.0025   # small visual seam between stacked reams
    parts = [box(f"Ream{i}_paper", (0, 0, 0), (REAM_W, REAM_L, h), M_PAPER)]
    # blue band wrapped around the ream, like a typical ream wrapper
    parts.append(box(f"Ream{i}_band", (0, -REAM_L / 2 + 0.06, 0), (REAM_W + 0.001, 0.07, h + 0.001), M_BAND))
    ob = join(parts, f"Ream{i}")
    return ob

reams = [ream(i) for i in range(N_REAMS)]

# --- IR LED + beam
led_off = cyl("LedOff", (-RAIL_X + 0.03, IR_Y, IR_Z + 0.0125), 0.003, 0.003, 'Z', M_LED_OFF, seg=12)
frame_static = join([frame_static, led_off], "Frame_Static")
led_on = cyl("LedOn", (0, 0, 0), 0.0045, 0.005, 'Z', M_LED_ON, seg=12)
led_on.location = (-RAIL_X + 0.03, IR_Y, IR_Z + 0.0135)
beam_len = (-REAM_W / 2) - (-RAIL_X + 0.042)
beam = cyl("IRBeam", (0, 0, 0), 0.003, beam_len, 'X', M_BEAM, seg=10)
beam.location = (-RAIL_X + 0.042 + beam_len / 2, IR_Y, IR_Z)

# ----------------------------------------------------------------------------
# Labels
# ----------------------------------------------------------------------------
LBL = 0.026
LX = 0.28
labels = []
def side_label(name, body, part_pt, lx=LX, parent=None, align='LEFT'):
    px, py, pz = part_pt
    labels.append(text(name, body, (lx + (0.005 if align == 'LEFT' else -0.005), py - 0.002, pz), LBL, M_LABEL, align, parent, extrude=0.002))
    labels.append(leader(name + "_leader", (px, py, pz), (lx, py, pz), parent))

side_label("LblStepper", "STEPPER MOTOR (NEMA 17)", (0.021, SCREW_Y - 0.03, 0.73))
side_label("LblScrew", "LEAD SCREW T8 (8 mm lead)", (0.004, SCREW_Y, 0.60))
side_label("LblRods", "LINEAR GUIDE RODS", (0.066, SCREW_Y, 0.50))
side_label("LblBelt", "TRANSFER BELT", (BELT_W / 2, -0.22, BELT_TOP))
side_label("LblOut", "OUTFEED ROLLERS", (OUT_RX, -0.62, 0.10))
side_label("LblCradle", "FORK-LIFT CRADLE", (TINE_X + TINE_W / 2, -0.05, -0.007), parent=cradle)
side_label("LblIR", "IR SENSOR", (-RAIL_X - 0.01, IR_Y, IR_Z + 0.03), lx=-LX, align='RIGHT')
side_label("LblProfile", "40x80 ALU PROFILE", (-RAIL_X - 0.02, 0.10, 0.04), lx=-LX, align='RIGHT')

# status banner (toggled)
STATUS_POS = (0, 0.235, 0.80)
status_defs = [("READY - WAITING FOR REAM", None)]
status_defs += [(f"REAM {i + 1} / 5 STACKED", i) for i in range(N_REAMS)]
status_defs += [("STACK READY - LOWERING TO BELT", "lower"), ("TRANSFERRING STACK", "transfer"), ("RETURNING TO LOAD POSITION", "reset")]
status_objs = [text(f"Status{k}", s, STATUS_POS, 0.032, M_LABEL_HI) for k, (s, _) in enumerate(status_defs)]
title = text("Title", "KMS VERTICAL REAM STACKER", (0, 0.235, 0.86), 0.036, M_LABEL)

# ----------------------------------------------------------------------------
# Motion functions (time in seconds)
# ----------------------------------------------------------------------------
def ease(u):
    u = max(0.0, min(1.0, u))
    return u * u * u * (u * (u * 6 - 15) + 10)   # smootherstep

def seg_value(t, segs, v0):
    v = v0
    for (ta, tb, va, vb) in segs:
        if t >= tb:
            v = vb
        elif t >= ta:
            return va + (vb - va) * ease((t - ta) / (tb - ta))
    return v

T0 = 1.5
CYCLE = 3.0
def t_spawn(i):
    return T0 + CYCLE * i
T_LOWER = (t_spawn(4) + 2.5, t_spawn(4) + 5.0)       # 16.0 -> 18.5
T_BELT = (19.0, 23.0)
T_HIDE = (23.0, 23.4)
T_RESET = (23.5, 27.0)

cradle_segs = []
for i in range(N_REAMS - 1):
    ts = t_spawn(i)
    cradle_segs.append((ts + 2.0, ts + 2.8, Z_LOAD - STEP * i, Z_LOAD - STEP * (i + 1)))
cradle_segs.append((*T_LOWER, Z_LOAD - STEP * (N_REAMS - 1), Z_HANDOVER))
cradle_segs.append((*T_RESET, Z_HANDOVER, Z_LOAD))

def cradle_z(t):
    return seg_value(t, cradle_segs, Z_LOAD)

def belt_s(t):
    return OUT_TRAVEL * ease((t - T_BELT[0]) / (T_BELT[1] - T_BELT[0]))

def stack_bottom(t):
    return max(cradle_z(t), BELT_TOP)

SPAWN_Y = STACK_Y - 0.42
HOVER = 0.10
HIDDEN = 0.001

def ream_pose(i, t):
    """returns (loc, scale)"""
    ts = t_spawn(i)
    hover_z = Z_LOAD + HOVER + REAM_H / 2
    spawn = (0, SPAWN_Y, hover_z)
    if t < ts or t >= T_HIDE[1]:
        return spawn, HIDDEN
    if t < ts + 0.3:
        return spawn, max(HIDDEN, ease((t - ts) / 0.3))
    if t < ts + 1.2:
        return (0, SPAWN_Y + (STACK_Y - SPAWN_Y) * ease((t - ts - 0.3) / 0.9), hover_z), 1.0
    rest_z = stack_bottom(t) + STEP * i + REAM_H / 2
    if t < ts + 1.6:
        return (0, STACK_Y, hover_z + (rest_z - hover_z) * ease((t - ts - 1.2) / 0.4)), 1.0
    y = STACK_Y - belt_s(t)
    sc = 1.0
    if t >= T_HIDE[0]:
        sc = max(HIDDEN, 1.0 - ease((t - T_HIDE[0]) / (T_HIDE[1] - T_HIDE[0])))
    return (0, y, rest_z), sc

def ir_on(t):
    return any(t_spawn(i) + 1.6 <= t < t_spawn(i) + 2.2 for i in range(N_REAMS))

def status_index(t):
    if t < t_spawn(0) + 1.6 or t >= T_RESET[1]:
        return 0
    for i in range(N_REAMS - 1, -1, -1):
        if t >= T_RESET[0]:
            return len(status_defs) - 1
        if t >= T_BELT[0]:
            return len(status_defs) - 2
        if t >= T_LOWER[0]:
            return len(status_defs) - 3
        if t >= t_spawn(i) + 1.6:
            return 1 + i
    return 0

# roller angle scale, snapped so the 3-bolt pattern lands identically at the loop point
def snapped_k(r):
    raw = OUT_TRAVEL / r
    sym = 2 * math.pi / 3
    return round(raw / sym) * sym / OUT_TRAVEL

K_BELT_ROLLER = snapped_k(ROLLER_R + BELT_T / 2)
K_OUT_ROLLER = snapped_k(OUT_ROLLER_R)

# ----------------------------------------------------------------------------
# Bake keyframes (one key per frame -> exact, exporter strips redundant keys)
# ----------------------------------------------------------------------------
for ob in [screw, cradle, *belt_rollers, *out_rollers, *cleats, *reams, led_on, beam, *status_objs]:
    ob.rotation_mode = 'XYZ'

def key(ob, path, f):
    ob.keyframe_insert(data_path=path, frame=f)

for f in range(0, FRAME_END + 1):
    t = f / FPS
    cz = cradle_z(t)
    cradle.location = (0, 0, cz)
    key(cradle, "location", f)

    screw.rotation_euler = (0, 0, -(Z_LOAD - cz) / LEAD * 2 * math.pi)
    key(screw, "rotation_euler", f)

    s = belt_s(t)
    for r in belt_rollers:
        r.rotation_euler = (s * K_BELT_ROLLER, 0, 0)
        key(r, "rotation_euler", f)
    for r in out_rollers:
        r.rotation_euler = (s * K_OUT_ROLLER, 0, 0)
        key(r, "rotation_euler", f)
    for k, c in enumerate(cleats):
        y, z, th = cleat_pose(k * CLEAT_PITCH + s)
        c.location = (0, y, z)
        c.rotation_euler = (th, 0, 0)
        key(c, "location", f)
        key(c, "rotation_euler", f)

    for i, rm in enumerate(reams):
        loc, sc = ream_pose(i, t)
        rm.location = loc
        rm.scale = (sc, sc, sc)
        key(rm, "location", f)
        key(rm, "scale", f)

    on = ir_on(t)
    for ob in (led_on, beam):
        v = 1.0 if on else HIDDEN
        ob.scale = (v, v, v)
        key(ob, "scale", f)

    si = status_index(t)
    for k, ob in enumerate(status_objs):
        v = 1.0 if k == si else HIDDEN
        ob.scale = (v, v, v)
        key(ob, "scale", f)

# make every key LINEAR (values are already eased, per frame)
def set_linear():
    for ob in bpy.data.objects:
        ad = ob.animation_data
        if not ad or not ad.action:
            continue
        act = ad.action
        curves = []
        try:
            curves = list(act.fcurves)
        except Exception:
            try:
                for layer in act.layers:
                    for strip in layer.strips:
                        for bag in strip.channelbags:
                            curves += list(bag.fcurves)
            except Exception:
                pass
        for fc in curves:
            for kp in fc.keyframe_points:
                kp.interpolation = 'LINEAR'
set_linear()

scene.frame_set(0)

# ----------------------------------------------------------------------------
# Stats + save
# ----------------------------------------------------------------------------
tris = 0
for ob in COLL.objects:
    if ob.type == 'MESH':
        ob.data.calc_loop_triangles()
        tris += len(ob.data.loop_triangles)
print(f"[stats] objects={len(COLL.objects)} triangles={tris} cleat_pitch={CLEAT_PITCH:.4f} out_travel={OUT_TRAVEL:.4f}")

blend_path = os.path.join(OUT_DIR, "stacker.blend")
bpy.ops.wm.save_as_mainfile(filepath=blend_path)

# ----------------------------------------------------------------------------
# Export
# ----------------------------------------------------------------------------
def call_filtered(op, **kw):
    props = set(op.get_rna_type().properties.keys())
    use = {k: v for k, v in kw.items() if k in props}
    skipped = sorted(set(kw) - set(use))
    if skipped:
        print(f"[export] {op.idname()} skipped unsupported options: {skipped}")
    return op(**use)

glb_path = os.path.join(OUT_DIR, "stacker.glb")
call_filtered(bpy.ops.export_scene.gltf,
              filepath=glb_path, export_format='GLB', use_selection=False,
              export_yup=True, export_apply=True, export_cameras=False, export_lights=False,
              export_animations=True, export_animation_mode='SCENE', export_force_sampling=True,
              export_anim_scene_split_object=False, export_nla_strips_merged_animation_name="StackerCycle",
              export_frame_range=True, export_optimize_animation_size=True,
              export_anim_single_armature=True, export_def_bones=False,
              export_materials='EXPORT', export_texcoords=True, export_normals=True)

usdz_path = os.path.join(OUT_DIR, "stacker.usdz")
call_filtered(bpy.ops.wm.usd_export,
              filepath=usdz_path, selected_objects_only=False, visible_objects_only=False,
              export_animation=True, export_materials=True, generate_preview_surface=True,
              export_textures=True, export_normals=True, export_uvmaps=True,
              export_cameras=False, export_lights=False,
              convert_orientation=True, export_global_forward_selection='NEGATIVE_Z',
              export_global_up_selection='Y', meters_per_unit=1.0,
              root_prim_path="/KMS_Stacker", evaluation_mode='RENDER')

# ----------------------------------------------------------------------------
# Verify GLB
# ----------------------------------------------------------------------------
with open(glb_path, "rb") as fh:
    data = fh.read()
jlen = struct.unpack_from("<I", data, 12)[0]
gj = json.loads(data[20:20 + jlen])
anims = gj.get("animations", [])
print(f"[glb] size={len(data)/1e6:.2f} MB nodes={len(gj.get('nodes', []))} meshes={len(gj.get('meshes', []))} "
      f"animations={len(anims)} channels={[len(a['channels']) for a in anims]}")
print(f"[usdz] size={os.path.getsize(usdz_path)/1e6:.2f} MB" if os.path.exists(usdz_path) else "[usdz] MISSING")
