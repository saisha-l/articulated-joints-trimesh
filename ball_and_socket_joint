import trimesh
import numpy as np
import polyscope as ps

# ========================================
# PARAMETERS (ADJUSTED FOR 3D PRINTING)
# ========================================
ball_radius = 10
stud_radius = 3
stud_length = 20
socket_thickness = 3
socket_opening = 0.6
clearance = 0.5  # Increased from 0.2 to 0.4 for reliable FDM printing

# ========================================
# 1. CREATE BALL with STUD
# ========================================
ball = trimesh.primitives.Sphere(radius=ball_radius, subdivisions=3)
stud = trimesh.primitives.Cylinder(radius=stud_radius, height=stud_length, sections=32)
stud.apply_translation([0, 0, stud_length/2 + ball_radius])
ball_assembly = trimesh.util.concatenate([ball, stud])

# ========================================
# 2. CREATE SOCKET
# ========================================
outer_radius = ball_radius + clearance + socket_thickness
inner_radius = ball_radius + clearance

outer_sphere = trimesh.primitives.Sphere(radius=outer_radius, subdivisions=3)
inner_sphere = trimesh.primitives.Sphere(radius=inner_radius, subdivisions=3)

cut_height = ball_radius * socket_opening
cutting_box = trimesh.creation.box([outer_radius*3, outer_radius*3, outer_radius*2])
cutting_box.apply_translation([0, 0, cut_height + outer_radius])

socket_shell = outer_sphere.difference(inner_sphere)
socket_assembly = socket_shell.difference(cutting_box)

# ========================================
# 3. CLEARANCE VERIFICATION (NO FCL REQUIRED)
# ========================================
print("\n🔍 Ball Joint Clearance Analysis")
print("=" * 50)

print(f"\nGeometry:")
print(f"  Ball radius: {ball_radius} mm")
print(f"  Socket inner radius: {inner_radius} mm")
print(f"  Socket outer radius: {outer_radius} mm")
print(f"  Design clearance: {clearance} mm")

# Simple distance check using bounding boxes and sample points
def check_minimum_distance(mesh1, mesh2, num_samples=1000):
    """
    Estimate minimum distance between two meshes using point sampling
    Returns minimum distance found
    """
    # Sample points on both surfaces
    points1 = mesh1.sample(num_samples)
    points2 = mesh2.sample(num_samples)
    
    # For each point on mesh1, find closest point on mesh2
    min_distances = []
    for p1 in points1:
        distances_to_mesh2 = np.linalg.norm(points2 - p1, axis=1)
        min_distances.append(np.min(distances_to_mesh2))
    
    return np.min(min_distances)

print(f"\nDistance Check:")
print(f"  Computing minimum distance between ball and socket...")

# Estimate minimum distance
min_dist = check_minimum_distance(ball_assembly, socket_assembly, num_samples=500)
print(f"  Estimated minimum distance: {min_dist:.3f} mm")

# Check for intersection/overlap
if min_dist < 0.05:  # Very close = likely overlapping
    print(f"\n⚠️  WARNING: Ball and socket appear to intersect or touch!")
    print(f"   → Increase 'clearance' parameter to at least {clearance + 0.2}")
elif min_dist < clearance * 0.5:  # Distance less than half the design clearance
    print(f"\n⚠️  WARNING: Actual clearance ({min_dist:.3f} mm) is less than design clearance ({clearance} mm)")
    print(f"   → This may indicate overlapping geometry")
elif clearance < 0.3:
    print(f"\n⚠️  WARNING: Clearance < 0.3mm may be too tight for FDM printing")
    print(f"   → Recommend clearance = 0.3-0.5 mm for FDM")
    print(f"   → Recommend clearance = 0.2-0.3 mm for SLA")
else:
    print(f"\n✅ Clearance looks good for FDM printing")

# Check stud clearance through socket opening
stud_tip_height = stud_length + ball_radius
socket_opening_height = cut_height
print(f"\nStud Check:")
print(f"  Stud tip height: {stud_tip_height:.1f} mm")
print(f"  Socket opening at: {socket_opening_height:.1f} mm")

if stud_tip_height > socket_opening_height:
    print(f"  ✅ Stud extends through socket opening")
else:
    print(f"  ⚠️  Stud may be blocked by socket")

# Bounding box check
ball_bounds = ball_assembly.bounds
socket_bounds = socket_assembly.bounds
print(f"\nBounding Box Check:")
print(f"  Ball: {ball_bounds[0]} to {ball_bounds[1]}")
print(f"  Socket: {socket_bounds[0]} to {socket_bounds[1]}")

# ========================================
# 4. EXPORT MESHES FOR 3D PRINTING (Downloads)
# ========================================
import os

downloads_dir = os.path.expanduser("/Users/saishalakkoju/code_projects")

print("\n📦 Exporting meshes to Downloads...")

ball_assembly.export(os.path.join(downloads_dir, "ball_joint_ball.obj"))
socket_assembly.export(os.path.join(downloads_dir, "ball_joint_socket.obj"))

# Combined preview (for visualization, not printing)
combined = trimesh.util.concatenate([ball_assembly, socket_assembly])
combined.export(os.path.join(downloads_dir, "ball_joint_complete.obj"))

print("✅ Exported files to ~/Downloads:")
print("   - ball_joint_ball.obj (print this)")
print("   - ball_joint_socket.obj (print this)")
print("   - ball_joint_complete.obj (preview only)")

# ========================================
# 5. INITIALIZE POLYSCOPE VISUALIZATION
# ========================================
ps.init()

# ========================================
# 6. BALL JOINT SIMULATOR CLASS
# ========================================
class BallJointSimulator:
    def __init__(self, socket_mesh, ball_mesh, pivot_point=None):
        self.socket = socket_mesh.copy()
        self.ball = ball_mesh.copy()
        self.ball_original = ball_mesh.copy()  # Keep original for reset
        self.pivot = pivot_point if pivot_point is not None else np.array([0, 0, 0])
        
        # Constraint limits (in radians)
        self.swing_limit = np.pi / 4  # 45 degrees
        self.twist_limit = np.pi      # 180 degrees
        
        # Current rotation state
        self.current_rotation = np.eye(4)
        
        # Animation state
        self.frame = 0
        self.angles = np.linspace(-30, 30, 100)
        
    def swing(self, angle_x, angle_y):
        """Swing motion (rotation perpendicular to twist axis)"""
        # Reset ball to original position
        self.ball = self.ball_original.copy()
        
        # Apply X rotation (side-to-side swing)
        if abs(angle_x) <= np.degrees(self.swing_limit):
            rot_x = trimesh.transformations.rotation_matrix(
                np.radians(angle_x), [1, 0, 0], self.pivot
            )
            self.ball.apply_transform(rot_x)
        
        # Apply Y rotation (forward-back swing)
        if abs(angle_y) <= np.degrees(self.swing_limit):
            rot_y = trimesh.transformations.rotation_matrix(
                np.radians(angle_y), [0, 1, 0], self.pivot
            )
            self.ball.apply_transform(rot_y)
    
    def twist(self, angle_z):
        """Twist motion (rotation around twist axis)"""
        if abs(angle_z) <= np.degrees(self.twist_limit):
            rot_z = trimesh.transformations.rotation_matrix(
                np.radians(angle_z), [0, 0, 1], self.pivot
            )
            self.ball.apply_transform(rot_z)
    
    def update_visualization(self):
        """Update the polyscope visualization"""
        # Update socket (fixed)
        ps.register_surface_mesh(
            "socket",
            self.socket.vertices,
            self.socket.faces,
            color=(1.0, 0.6, 0.2),  # Orange
            transparency=0.5
        )
        
        # Update ball (moving)
        ps.register_surface_mesh(
            "ball",
            self.ball.vertices,
            self.ball.faces,
            color=(0.8, 0.8, 0.8),  # Light gray
            transparency=0.3
        )
    
    def animation_callback(self):
        """Callback for animated swing"""
        angle = self.angles[self.frame % len(self.angles)]
        self.swing(angle, angle * 0.5)
        self.update_visualization()
        self.frame += 1

# ========================================
# 7. CREATE AND VISUALIZE
# ========================================
simulator = BallJointSimulator(socket_assembly, ball_assembly, pivot_point=np.array([0, 0, 0]))

print("\n🎮 Ball Joint Simulator - Polyscope")
print("=" * 50)
print(f"\nConstraints:")
print(f"  Swing limit: ±{np.degrees(simulator.swing_limit):.1f}°")
print(f"  Twist limit: ±{np.degrees(simulator.twist_limit):.1f}°")

# Initial visualization
simulator.update_visualization()

# Set up animation callback
ps.set_user_callback(simulator.animation_callback)

# Show the viewer
print("\n✅ Opening Polyscope viewer...")
print("   Press SPACE to pause/resume animation")
print("   Use mouse to rotate view")
ps.show()
