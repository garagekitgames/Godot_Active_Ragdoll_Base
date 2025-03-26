extends Node3D

#--------------------------------------------------
# Exported Parameters (Set these in the Inspector)
#--------------------------------------------------
@export var enable_linear_correction: bool = true
@export var enable_angular_correction: bool = true

# Linear correction parameters
@export var linear_spring_stiffness: float = 120000
@export var linear_spring_damping: float = 100

# Angular correction parameters
@export var angular_spring_stiffness: float = 500000
@export var angular_spring_damping: float = 100
@export var max_angular_force: float = 5000.0

#--------------------------------------------------
# Node References (Assign these in the Inspector)
#--------------------------------------------------
@export var physical_skel: Skeleton3D 
@export var physical_bone_simulator_3d: PhysicalBoneSimulator3D
@export var animated_skel: Skeleton3D

#--------------------------------------------------
# Internal Variables
#--------------------------------------------------
var physics_bones = []       # Array of PhysicalBone3D nodes
var current_delta: float = 0.0

#--------------------------------------------------
# Helper Function: Saturating PD Controller
# Computes a PD output where the error is scaled using an exponential saturating function.
func saturating_pd(error: Vector3, velocity: Vector3, stiffness: float, alpha: float, damping: float) -> Vector3:
	var mag = error.length()
	if mag < 0.0001:
		return -damping * velocity
	var control = stiffness * (1.0 - exp(-alpha * mag)) * (error / mag)
	return control - damping * velocity

#--------------------------------------------------
# Main Functions
#--------------------------------------------------
func _ready() -> void:
	# Connect the skeleton_updated signal so that _on_skeleton_3d_skeleton_updated() is called
	physical_skel.connect("skeleton_updated", Callable(self, "_on_skeleton_3d_skeleton_updated"))
	# Start the physical bone simulation (i.e. the ragdoll mode)
	physical_bone_simulator_3d.physical_bones_start_simulation()
	# Collect all PhysicalBone3D nodes from the simulator
	physics_bones = physical_bone_simulator_3d.get_children().filter(func(x): return x is PhysicalBone3D)

func _physics_process(delta: float) -> void:
	current_delta = delta

# This function is called each time the skeleton is updated.
func _on_skeleton_3d_skeleton_updated() -> void:
	# Loop through all physical bones and update their correction.
	for b in physics_bones:
		var bone_id = b.get_bone_id()
		# Compute global target and current transforms.
		var target_transform: Transform3D = animated_skel.global_transform * animated_skel.get_bone_global_pose(bone_id)
		var current_transform: Transform3D = physical_skel.global_transform * physical_skel.get_bone_global_pose(bone_id)
		
		# --- Linear Correction ---
		if enable_linear_correction:
			var pos_diff: Vector3 = target_transform.origin - current_transform.origin
			var force: Vector3 = saturating_pd(pos_diff, b.linear_velocity, linear_spring_stiffness, 0.01, linear_spring_damping)
			b.linear_velocity += force * current_delta
		
		# --- Angular Correction ---
		if enable_angular_correction:
			# Compute the rotation difference in global space.
			var rotation_difference: Basis = target_transform.basis * current_transform.basis.inverse()
			# Convert the rotation difference to Euler angles (error in radians).
			var error: Vector3 = rotation_difference.get_euler()
			var torque: Vector3 = saturating_pd(error, b.angular_velocity, angular_spring_stiffness, 0.01, angular_spring_damping)
			torque = torque.limit_length(max_angular_force)
			b.angular_velocity += torque * current_delta
