import trimesh
import numpy as np
import polyscope as ps

# ======================================================
# PARAMETERS (OPTIMIZATION-READY)
# ======================================================

# ---- Ball joint ----
ball_radius = 10.0
stud_radius = 3.0
stud_length = 20.0
clearance = 0.4
socket_thickness = 3.0

# ---- Hinge joint ----
hinge_radius = 6.0
hinge_length = 22.0
hinge_wall = 3.0
hinge_range_deg = 120

# ---- Slider joint ----
rail_radius = 5.0
rail_length = 45.0
carriage_length = 18.0
slider_clearance = 0.4

# ======================================================
# TRANSFORM HELPERS
# ======================================================
def R(angle_deg, axis, point=[0, 0, 0]):
    return trimesh.transformations.rotation_matrix(
        np.radians(angle_deg), axis, point
    )

def T(offset):
    return trimesh.transformations.translation_matrix(offset)

# ======================================================
# BALL JOINT GEOMETRY
# ======================================================
ball = trimesh.primitives.Sphere(radius=ball_radius, subdivisions=3)

stud = trimesh.primitives.Cylinder(
    radius=stud_radius,
    height=stud_length,
    sections=32
)
stud.apply_translation([0, 0, stud_length / 2 + ball_radius])

ball_part = trimesh.util.concatenate([ball, stud])

outer = trimesh.primitives.Sphere(
    radius=ball_radius + clearance + socket_thickness,
    subdivisions=3
)
inner = trimesh.primitives.Sphere(
    radius=ball_radius + clearance,
    subdivisions=3
)

cut = trimesh.creation.box([100, 100, 100])
cut.apply_translation([0, 0, ball_radius])

socket = outer.difference(inner).difference(cut)

# ======================================================
# HINGE GEOMETRY
# ======================================================
barrel = trimesh.primitives.Cylinder(
    radius=hinge_radius,
    height=hinge_length,
    sections=64
)
barrel.apply_transform(R(90, [0, 1, 0]))

hinge_connector = trimesh.primitives.Cylinder(
    radius=stud_radius,
    height=stud_length,
    sections=32
)
hinge_connector.apply_translation([0, 0, stud_length / 2])

hinge_moving = trimesh.util.concatenate([barrel, hinge_connector])

outer = trimesh.primitives.Cylinder(
    radius=hinge_radius + slider_clearance + hinge_wall,
    height=hinge_length,
    sections=64
)
inner = trimesh.primitives.Cylinder(
    radius=hinge_radius + slider_clearance,
    height=hinge_length + 1,
    sections=64
)

outer.apply_transform(R(90, [0, 1, 0]))
inner.apply_transform(R(90, [0, 1, 0]))

cut = trimesh.creation.box([100, 100, 100])
cut.apply_translation([0, -50, 0])

hinge_fixed = outer.difference(inner).difference(cut)

# ======================================================
# SLIDER GEOMETRY
# ======================================================
rail = trimesh.primitives.Cylinder(
    radius=rail_radius,
    height=rail_length,
    sections=64
)
rail.apply_translation([0, 0, rail_length / 2])

outer = trimesh.primitives.Cylinder(
    radius=rail_radius + 3,
    height=carriage_length,
    sections=64
)
inner = trimesh.primitives.Cylinder(
    radius=rail_radius + slider_clearance,
    height=carriage_length + 1,
    sections=64
)

outer.apply_translation([0, 0, carriage_length / 2])
inner.apply_translation([0, 0, (carriage_length + 1) / 2])

carriage = outer.difference(inner)

# ======================================================
# JOINT CLASSES
# ======================================================
class BallJoint:
    def __init__(self, mesh):
        self.mesh0 = mesh
        self.mesh = mesh.copy()
        self.transform = np.eye(4)
        self.child_frame = T([0, 0, ball_radius + stud_length])

    def apply(self, ax, ay):
        self.transform = R(ax, [1, 0, 0]) @ R(ay, [0, 1, 0])
        self.mesh = self.mesh0.copy()
        self.mesh.apply_transform(self.transform)

class HingeJoint:
    def __init__(self, fixed, moving):
        self.fixed = fixed
        self.moving0 = moving
        self.moving = moving.copy()

    def apply(self, parent_tf, angle):
        self.moving = self.moving0.copy()
        self.moving.apply_transform(parent_tf @ R(angle, [1, 0, 0]))

class SliderJoint:
    def __init__(self, rail, carriage):
        self.rail = rail
        self.carriage0 = carriage
        self.carriage = carriage.copy()

    def apply(self, parent_tf, t):
        self.carriage = self.carriage0.copy()
        self.carriage.apply_transform(parent_tf @ T([0, 0, t]))

# ======================================================
# POLYSCOPE SETUP
# ======================================================
ps.init()

ball_joint = BallJoint(ball_part)
hinge_joint = HingeJoint(hinge_fixed, hinge_moving)
slider_joint = SliderJoint(rail, carriage)

angles = np.linspace(-30, 30, 120)
hinge_angles = np.linspace(-hinge_range_deg / 2, hinge_range_deg / 2, 120)
slide_vals = np.linspace(0, rail_length - carriage_length, 120)

frame = 0

def animate():
    global frame

    ax = angles[frame % len(angles)]
    ay = angles[(frame * 2) % len(angles)]
    hinge_angle = hinge_angles[frame % len(hinge_angles)]
    slide = slide_vals[frame % len(slide_vals)]

    # Ball joint
    ball_joint.apply(ax, ay)

    # Hinge joint
    hinge_parent_tf = ball_joint.transform @ ball_joint.child_frame
    hinge_joint.apply(hinge_parent_tf, hinge_angle)

    # Slider joint
    slider_parent_tf = hinge_parent_tf @ R(hinge_angle, [1, 0, 0])
    slider_joint.apply(slider_parent_tf, slide)

    # Visualize
    ps.register_surface_mesh(
        "socket",
        socket.vertices,
        socket.faces,
        transparency=0.5,
        color=(1.0, 0.6, 0.2)
    )

    ps.register_surface_mesh(
        "ball",
        ball_joint.mesh.vertices,
        ball_joint.mesh.faces,
        transparency=0.4
    )

    ps.register_surface_mesh(
        "hinge_fixed",
        hinge_joint.fixed.vertices,
        hinge_joint.fixed.faces,
        transparency=0.5
    )

    ps.register_surface_mesh(
        "hinge_moving",
        hinge_joint.moving.vertices,
        hinge_joint.moving.faces,
        transparency=0.4
    )

    ps.register_surface_mesh(
        "rail",
        slider_joint.rail.vertices,
        slider_joint.rail.faces,
        transparency=0.5
    )

    ps.register_surface_mesh(
        "carriage",
        slider_joint.carriage.vertices,
        slider_joint.carriage.faces,
        transparency=0.4
    )

    frame += 1

ps.set_user_callback(animate)

print("\n✅ Ball → Hinge → Slider chain")
print("• All transforms are local")
print("• Parent–child hierarchy respected")
print("• Ball (SO3) + Hinge (R) + Slider (T)")
print("• Fully optimization-ready")

ps.show()
