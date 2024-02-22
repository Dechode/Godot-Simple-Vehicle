extends Node

@onready var bike = get_parent() as RigidBody3D

func _physics_process(delta: float) -> void:
	var lean_vec := Input.get_vector("lean_left", "lean_right", "lean_forward", "lean_backward")
	var turn_input := Input.get_action_strength("SteerLeft") - Input.get_action_strength("SteerRight") 
	lean_vec.x *= 0.2
	lean_vec.y *= 0.3
	
	turn_input = absf(turn_input / 0.4) * sign(turn_input)
	turn_input = clampf(turn_input, -1.0, 1.0)
	
	var turn_lean := Vector3(-turn_input * 0.1, 0.0, 0.0)
	var com := Vector3(lean_vec.x, 0.0, lean_vec.y) + turn_lean
	
	bike.center_of_mass = com
