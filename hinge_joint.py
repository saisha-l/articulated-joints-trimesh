import trimesh
import numpy as np
import polyscope as ps

# ======================================================
# PARAMETERS (OPTIMIZATION-READY)
# ======================================================
barrel_radius = 6.0          # main rotating cylinder
barrel_length = 22.0

saddle_wall = 3.0            # thickness of the cradle
saddle_width = 16.0          # length along rotation axis

clearance = 0.4              # FDM-friendly clearance
max_angle_deg = 120          # articulation limit

connector_radius = 5.0       # vertical connection ports
connector_length = 18.0

# ======================================================
# BARREL (MALE ROTATING PART)
# ======================================================
barrel = trimesh.primitives.Cylinder(
    radius=barrel_radius,
    height=barrel_length,
    sections=64
)

# Rotate barrel so axis is X
barrel.apply_transform(
    trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0])
)

# ======================================================
# ADD VERTICAL CONNECTOR TO BARREL
# ======================================================
barrel_connector = trimesh.primitives.Cylinder(
    radius=connector_radius,
    height=connector_length,
    sections=48
)
barrel_connector.apply_translation([0, 0, connector_length / 2])

barrel_part = trimesh.util.concatenate([barrel, barrel_connector])

# ======================================================
# SADDLE (FEMALE CRADLE)
# ======================================================
outer_radius = barrel_radius + clearance + saddle_wall
inner_radius = barrel_radius + clearance

outer = trimesh.primitives.Cylinder(
    radius=outer_radius,
    height=saddle_width,
    sections=64
)

inner = trimesh.primitives.Cylinder(
    radius=inner_radius,
    height=saddle_width + 1,
    sections=64
)

# Rotate saddle to match barrel
outer.apply_transform(
    trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0])
)
inner.apply_transform(
    trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0])
)

# Cut bottom half to create open cradle
cut_box = trimesh.creation.box(
    [outer_radius * 3, outer_radius * 3, outer_radius * 3]
)
cut_box.apply_translation([0, -outer_radius * 1.5, 0])

saddle = outer.difference(inner).difference(cut_box)

# ======================================================
# ADD VERTICAL CONNECTOR TO SADDLE
# ======================================================

saddle_part = saddle

# ======================================================
# EXPORT FOR PRINTING
# ======================================================
barrel_part.export("elbow_hinge_barrel.obj")
saddle_part.export("elbow_hinge_saddle.obj")

print("✅ Exported:")
print(" - elbow_hinge_barrel.obj")
print(" - elbow_hinge_saddle.obj")

# ======================================================
# POLYSCOPE VISUALIZATION
# ======================================================
ps.init()

class ElbowHingeAnimator:
    def __init__(self, fixed, moving):
        self.fixed = fixed
        self.moving0 = moving
        self.moving = moving.copy()
        self.frame = 0
        self.angles = np.linspace(
            -max_angle_deg / 2,
            max_angle_deg / 2,
            160
        )

    def step(self):
        self.moving = self.moving0.copy()
        angle = self.angles[self.frame % len(self.angles)]

        R = trimesh.transformations.rotation_matrix(
            np.radians(angle),
            [1, 0, 0],   # hinge axis
            point=[0, 0, 0]
        )

        self.moving.apply_transform(R)

        ps.register_surface_mesh(
            "saddle",
            self.fixed.vertices,
            self.fixed.faces,
            color=(1.0, 0.7, 0.3),
            transparency=0.55
        )

        ps.register_surface_mesh(
            "barrel",
            self.moving.vertices,
            self.moving.faces,
            color=(0.8, 0.8, 0.8),
            transparency=0.35
        )

        self.frame += 1

anim = ElbowHingeAnimator(saddle_part, barrel_part)
ps.set_user_callback(anim.step)

print("\n🎮 Elbow Hinge Joint")
print(f"Rotation range: ±{max_angle_deg / 2}°")
ps.show()
