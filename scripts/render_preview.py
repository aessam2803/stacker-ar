"""
Preview render of stacker.blend (camera, lights and floor are added here only, never exported).

  blender --background stacker.blend --python render_preview.py -- <out_dir> [stills|video]
"""
import bpy, os, sys, math

args = sys.argv[sys.argv.index("--") + 1:]
OUT = args[0]
MODE = args[1] if len(args) > 1 else "stills"
os.makedirs(OUT, exist_ok=True)
scene = bpy.context.scene

# engine
for eng in ("BLENDER_EEVEE", "BLENDER_EEVEE_NEXT"):
    try:
        scene.render.engine = eng
        break
    except Exception:
        pass
try:
    scene.eevee.taa_render_samples = 16
except Exception:
    pass
scene.render.resolution_x, scene.render.resolution_y = (1280, 720)
scene.render.resolution_percentage = 100
scene.render.image_settings.file_format = 'PNG'
try:
    scene.view_settings.view_transform = 'AgX'
except Exception:
    pass

# world
world = bpy.data.worlds.new("Studio")
scene.world = world
try:
    world.use_nodes = True
except Exception:
    pass
bg = world.node_tree.nodes.get("Background")
bg.inputs[0].default_value = (0.55, 0.57, 0.60, 1)
bg.inputs[1].default_value = 0.35

# floor
me = bpy.data.meshes.new("Floor")
s = 6
me.from_pydata([(-s, -s, 0), (s, -s, 0), (s, s, 0), (-s, s, 0)], [], [(0, 1, 2, 3)])
fm = bpy.data.materials.new("FloorMat")
try:
    fm.use_nodes = True
except Exception:
    pass
fm.node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value = (0.62, 0.63, 0.65, 1)
fm.node_tree.nodes["Principled BSDF"].inputs["Roughness"].default_value = 0.7
me.materials.append(fm)
floor = bpy.data.objects.new("Floor", me)
scene.collection.objects.link(floor)

# lights
def light(name, kind, loc, energy, size=1.0, rot=(0, 0, 0)):
    ld = bpy.data.lights.new(name, kind)
    ld.energy = energy
    if kind == 'AREA':
        ld.size = size
    ob = bpy.data.objects.new(name, ld)
    ob.location = loc
    ob.rotation_euler = rot
    scene.collection.objects.link(ob)
    return ob

light("Key", 'AREA', (1.2, -1.4, 1.8), 120, 1.5, (math.radians(50), 0, math.radians(40)))
light("Fill", 'AREA', (-1.6, -0.8, 1.2), 40, 2.0, (math.radians(60), 0, math.radians(-60)))
light("Sun", 'SUN', (0, 0, 3), 1.2, rot=(math.radians(35), math.radians(15), math.radians(20)))

# camera
target = bpy.data.objects.new("CamTarget", None)
target.location = (0, -0.20, 0.40)
scene.collection.objects.link(target)
cd = bpy.data.cameras.new("Cam")
cd.lens = 40
cam = bpy.data.objects.new("Cam", cd)
cam.location = (1.75, -2.05, 1.15)
scene.collection.objects.link(cam)
c = cam.constraints.new('TRACK_TO')
c.target = target
c.track_axis = 'TRACK_NEGATIVE_Z'
c.up_axis = 'UP_Y'
scene.camera = cam

if MODE == "stills":
    for t in (0.5, 5.0, 13.9, 17.0, 18.8, 21.0, 25.0):
        f = int(t * scene.render.fps)
        scene.frame_set(f)
        scene.render.filepath = os.path.join(OUT, f"still_{f:04d}.png")
        bpy.ops.render.render(write_still=True)
else:
    scene.frame_start, scene.frame_end = 0, scene.frame_end - 1
    scene.render.filepath = os.path.join(OUT, "f_")
    bpy.ops.render.render(animation=True)
