extends RigidBody3D

@export var max_steer := 0.5 # In radians
@export var steer_speed := 5.0
@export var max_brake_force := 2500 # Total braking force in newtons

#### Engine related stuff ####
@export var max_torque := 250
@export var max_engine_rpm := 8000.0
@export var rpm_idle := 900.0
@export var torque_curve: Curve
@export var engine_drag := 0.03
@export var engine_brake := 10.0
@export var engine_moment := 0.25
@export var engine_sound: AudioStream

#### Drivetrain related stuff ####
@export var gear_ratios := [ 3.0, 2.2, 1.7, 1.4, 1.0, 0.9 ] 
@export var final_drive := 3.7
@export var reverse_ratio := 3.9
@export var gearbox_inertia := 0.2

#### Constants ####
const AV_2_RPM: float = 60 / TAU

#### Controller inputs ####
var throttle_input: float = 0.0
var steering_input: float = 0.0
var brake_input: float = 0.0

#### Misc ####
var steering_amount: float = 0.0
var selected_gear: int = 0
var drag_torque: float = 0.0
var torque_out: float = 0.0

var brake_force: float = 0.0
var rpm: float = 0.0
#var speedo: int = 0
var drive_inertia: float = 0.0 #includes every inertia of the drivetrain
#var r_split: float = 0.5
var wheel_radius: float = 0.0

var wheels := []

@onready var audioplayer = $EngineSound

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	wheels.clear()
	for child in get_children():
		if child is WheelSuspension:
			wheels.append(child)


func _unhandled_input(event: InputEvent) -> void:
	if event.is_action_pressed("ShiftUp"):
		shiftUp()
	if event.is_action_pressed("ShiftDown"):
		shiftDown()


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	brake_input = Input.get_action_strength("Brake")
	steering_input = Input.get_action_strength("SteerLeft") - Input.get_action_strength("SteerRight")
	throttle_input = Input.get_action_strength("Throttle")
	
	brake_force = max_brake_force * brake_input / wheels.size() # Per Wheel
	
	#speedo = avg_front_spin * wheel_radius * 3.6
	engineSound()
	
	
func _physics_process(delta: float) -> void:
	
	##### Steering with steer speed #####
	if (steering_input < steering_amount):
		steering_amount -= steer_speed * delta
		if (steering_input > steering_amount):
			steering_amount = steering_input
	
	elif (steering_input > steering_amount):
		steering_amount += steer_speed * delta
		if (steering_input < steering_amount):
			steering_amount = steering_input
	
	for w in wheels:
		if w.is_steering:
			w.steer(steering_amount, max_steer)
	
	#### Engine Loop ####
	drag_torque = engine_brake + rpm * engine_drag
	torque_out = (get_engine_torque(rpm) + drag_torque ) * throttle_input
	rpm += AV_2_RPM * delta * (torque_out - drag_torque) / engine_moment
	
	if rpm >= max_engine_rpm:
		torque_out = 0
		rpm -= 500
	
	if selected_gear == 0:
		freewheel(delta)
	else:
		engage(delta)
	
	var clutch_rpm = rpm_idle
	if abs(selected_gear) == 1:
		clutch_rpm += throttle_input * 2000
	rpm = max(rpm, clutch_rpm)
	rpm = max(rpm , rpm_idle)
	
	for w in wheels:
		w.apply_forces(delta)


func get_engine_torque(r_p_m) -> float: 
	var rpm_factor = clamp(r_p_m / max_engine_rpm, 0.0, 1.0)
	var torque_factor = torque_curve.sample_baked(rpm_factor)
	return torque_factor * max_torque


func freewheel(delta):
	for w in wheels:
		w.apply_torque(0.0, 0.0, brake_force, delta)


func engage(delta):
	var avg_spin := 0.0
	for w in wheels:
		if w.is_driven:
			avg_spin += w.spin
	var net_drive = (torque_out - drag_torque) * get_gear_ratios()
	avg_spin /= wheels.size()
	if avg_spin * sign(get_gear_ratios()) < 0:
		net_drive += drag_torque * get_gear_ratios()
	
	drive(net_drive, delta)
	rpm = avg_spin * get_gear_ratios() * AV_2_RPM
	

func get_gear_ratios():
	if selected_gear > 0:
		return gear_ratios[selected_gear - 1] * final_drive
	elif selected_gear == -1:
		return -reverse_ratio * final_drive
	else:
		return 0.0


func drive(drive_torque, delta):
	drive_inertia = engine_moment + pow(abs(get_gear_ratios()), 2) * gearbox_inertia
	var driven_count := 0
	for w in wheels:
		if w.is_driven:
			driven_count += 1
		
	for w in wheels:
		if w.is_driven:
			w.apply_torque(drive_torque / driven_count, drive_inertia, brake_force, delta)
		else:
			w.apply_torque(0.0, 0.0, brake_force, delta)


func engineSound():
	var pitch_scaler = rpm / 1000
	if rpm >= rpm_idle and rpm < max_engine_rpm:
		if audioplayer.stream != engine_sound:
			audioplayer.set_stream(engine_sound)
		if !audioplayer.playing:
			audioplayer.play()
	
	if pitch_scaler > 0.1:
		audioplayer.pitch_scale = pitch_scaler


func shiftUp():
	if selected_gear < gear_ratios.size():
		selected_gear += 1


func shiftDown():
	if selected_gear > -1:
		selected_gear -= 1
