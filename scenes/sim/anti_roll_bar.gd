class_name AntiRollBar
extends Node

@export var stiffness := 100.0
@export var tire_left: WheelSuspension = null
@export var tire_right: WheelSuspension = null


func _physics_process(delta: float) -> void:
	tire_left.antirollbar = stiffness * (tire_left.compress - tire_right.compress)
	tire_right.antirollbar = stiffness * (tire_right.compress - tire_left.compress)

