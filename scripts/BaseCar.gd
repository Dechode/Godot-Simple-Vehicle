extends RigidBody

export (float) var max_steer = 0.5 # In radians
export (float) var Steer_Speed = 5.0
export (float) var max_brake_force = 2500 # Total braking force in newtons

#### Engine related stuff ####
export (float) var max_torque = 250
export (float) var max_engine_rpm = 8000.0
export (float) var rpm_idle = 900
export (Curve) var torque_curve = null
export (float) var engine_drag = 0.03
export (float) var engine_brake = 10.0
export (float) var engine_moment = 0.25
export (AudioStream) var engine_sound

#### Drivetrain related stuff ####
export (Array) var gear_ratios = [ 3.0, 2.2, 1.7, 1.4, 1.0, 0.9 ] 
export (float) var final_drive = 3.7
export (float) var reverse_ratio = 3.9

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
var speedo: int = 0
var drive_inertia: float = 0.0 #includes every inertia of the drivetrain
var r_split: float = 0.5
var wheel_radius: float = 0.0

var avg_rear_spin = 0.0
var avg_front_spin = 0.0

onready var wheel_fl = $WheelFL
onready var wheel_fr = $WheelFR
onready var wheel_bl = $WheelBL
onready var wheel_br = $WheelBR
onready var audioplayer = $EngineSound

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.


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
	prints("front left rotation",wheel_fl.rotation_degrees.y)
	prints("front right rotation",wheel_fr.rotation_degrees.y)
	
	brake_force = max_brake_force * brake_input * 0.25 # Per Wheel
	
	speedo = avg_front_spin * wheel_radius * 3.6
	engineSound()
	
func _physics_process(delta: float) -> void:
	
	wheel_bl.apply_forces(delta)
	wheel_br.apply_forces(delta)
	wheel_fl.apply_forces(delta)
	wheel_fr.apply_forces(delta)
	
	##### Steering with steer speed #####
	if (steering_input < steering_amount):
		steering_amount -= Steer_Speed * delta
		if (steering_input > steering_amount):
			steering_amount = steering_input
	
	elif (steering_input > steering_amount):
		steering_amount += Steer_Speed * delta
		if (steering_input < steering_amount):
			steering_amount = steering_input
	
	wheel_fl.steer(steering_amount, max_steer)
	wheel_fr.steer(steering_amount, max_steer)
	
	#### Engine Loop ####
	drag_torque = engine_brake + rpm * engine_drag
	torque_out = (engineTorque(rpm) + drag_torque ) * throttle_input
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


func engineTorque(r_p_m) -> float: 
	var rpm_factor = clamp(r_p_m / max_engine_rpm, 0.0, 1.0)
	var torque_factor = torque_curve.interpolate_baked(rpm_factor)
	return torque_factor * max_torque


func freewheel(delta):
#	print(brake_force)
	avg_front_spin = 0.0
	avg_front_spin += (wheel_fl.spin + wheel_fr.spin) * 0.5
	wheel_bl.apply_torque(0.0, 0.0, brake_force, delta)
	wheel_br.apply_torque(0.0, 0.0, brake_force, delta)
	wheel_fl.apply_torque(0.0, 0.0, brake_force, delta)
	wheel_fr.apply_torque(0.0, 0.0, brake_force, delta)


func engage(delta):
	avg_rear_spin = 0.0
	avg_front_spin = 0.0
	avg_rear_spin += (wheel_bl.spin + wheel_br.spin) * 0.5
	avg_front_spin += (wheel_fl.spin + wheel_fr.spin) * 0.5
	var net_drive = (torque_out - drag_torque) * gearRatios()
	
	if avg_rear_spin * sign(gearRatios()) < 0:
		net_drive += drag_torque * gearRatios()
	
	rwd(net_drive, delta)
	rpm = avg_rear_spin * gearRatios() * AV_2_RPM
	

func gearRatios():
	if selected_gear > 0:
		return gear_ratios[selected_gear - 1] * final_drive
	elif selected_gear == -1:
		return -reverse_ratio * final_drive
	else:
		return 0.0


func rwd(drive, delta):
	drive_inertia = engine_moment + pow(abs(gearRatios()), 2) * 0.3 
	wheel_bl.apply_torque(drive * 0.5, drive_inertia, brake_force, delta)
	wheel_br.apply_torque(drive * 0.5, drive_inertia, brake_force, delta)
	wheel_fl.apply_torque(0.0, 0.0, brake_force, delta)
	wheel_fr.apply_torque(0.0, 0.0, brake_force, delta)


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
