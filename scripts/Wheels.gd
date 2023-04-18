extends RayCast3D


@export var spring_length := 0.2
@export var springstiffness := 8000
@export var bump := 5000
@export var rebound := 6000

@export var wheel_inertia := 1.6
@export var tire_radius := 0.3
@export var ackermann := 0.15

############# For curve tire formula #############
@export var lateral_force: Curve = null
@export var longitudinal_force: Curve = null

var mu := 1.0 # Friction coefficient
var y_force: float = 0.0

var spin: float = 0.0
var z_vel: float = 0.0
var local_vel

var force_vec = Vector2.ZERO
var slip_vec: Vector2 = Vector2.ZERO
var prev_pos: Vector3 = Vector3.ZERO

var peak_sr: float = 0.10
var peak_sa: float = 0.10

var prev_compress: float = 0.0
var spring_curr_length: float = spring_length

@onready var car = $'..' #Get the parent node as car


# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	set_target_position(Vector3.DOWN * (spring_length + tire_radius))
	peak_sa = lateral_force.get_point_position(1).x
	peak_sr = longitudinal_force.get_point_position(1).x


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	$WheelMesh.position.y = -spring_curr_length
	$WheelMesh.rotate_x(wrapf(-spin * delta,0, TAU))


func apply_forces(delta):
	############# Local forward velocity #############
	local_vel = ((global_transform.origin - prev_pos) / delta) * global_transform.basis
	z_vel = -local_vel.z
	var planar_vect = Vector2(local_vel.x, local_vel.z).normalized()
	prev_pos = global_transform.origin
	
	############# Suspension #################
	if is_colliding():
		spring_curr_length = get_collision_point().distance_to(global_transform.origin) - tire_radius
	else:
		spring_curr_length = spring_length
		
	var compress = clamp(1 - spring_curr_length / spring_length, 0.0, 1.0) 
	y_force = springstiffness * compress# * spring_length
#	print(compress)
	var compress_speed = compress - prev_compress
	
	if compress_speed >= 0:
		y_force += (bump) * (compress - prev_compress) * spring_length / delta
	else:
		y_force += rebound * (compress - prev_compress) * spring_length  / delta
		
	y_force = max(0, y_force)
	prev_compress = compress
	
	slip_vec.x = asin(clamp(-planar_vect.x, -1, 1)) # X slip is lateral slip
	slip_vec.y = 0.0 # Y slip is the longitudinal Z slip
	
#	if is_colliding() and z_vel != 0:
	if z_vel != 0:
#		slip_vec.y = (z_vel - spin * tire_radius) / abs(z_vel)
		slip_vec.y = -(spin * tire_radius - z_vel) / abs(z_vel)
	else:
		if spin == 0:
			slip_vec.y = 0.0
		else:
			slip_vec.y = 0.01 * spin
	
	############### Slip combination ###############
	
	var slip_ratio = slip_vec.y 
	var slip_angle = slip_vec.x
	
	var normalised_sr = slip_ratio / peak_sr
	var normalised_sa = slip_angle / peak_sa
	var resultant_slip = sqrt(pow(normalised_sr, 2) + pow(normalised_sa, 2))

	var sr_modified = resultant_slip * peak_sr
	var sa_modified = resultant_slip * peak_sa
	
	var x_force: float = 0.0
	var z_force: float = 0.0
	
	############### Apply the forces #######################
	
	x_force = TireForce(abs(sa_modified), y_force, lateral_force) * sign(slip_vec.x)
	z_force = TireForce(abs(sr_modified), y_force, longitudinal_force) * sign(slip_vec.y)
	
	if resultant_slip != 0:
		force_vec.x = x_force * abs(normalised_sa / resultant_slip)
		force_vec.y = z_force * abs(normalised_sr / resultant_slip)
	else:
		force_vec.x = 0.0
		force_vec.y = 0.0
	
	if is_colliding():
		var contact = get_collision_point() - car.global_transform.origin
		var normal = get_collision_normal()
		
		car.apply_force(normal * y_force, contact)
		car.apply_force(global_transform.basis.x * force_vec.x, contact)
		car.apply_force(global_transform.basis.z * force_vec.y, contact)
	else:
		spin -= sign(spin) * delta * 2 / wheel_inertia

func apply_torque(drive, drive_inertia, brake_torque, delta):
	var prev_spin = spin

	var net_torque = force_vec.y * tire_radius
	net_torque += drive
	
	if spin < 5 and brake_torque > abs(net_torque):
		spin = 0
	else:
		net_torque -= brake_torque * sign(spin)
		spin += delta * net_torque / (wheel_inertia + drive_inertia)

	if drive * delta == 0:
		return 0.5
	else:
		return (spin - prev_spin) * (wheel_inertia + drive_inertia) / (drive * delta)


func TireForce(slip: float, normal_load: float, tire_curve: Curve) -> float:
	var friction = normal_load * mu
	return tire_curve.sample_baked(abs(slip)) * friction * sign(slip)


func steer(input, max_steer):
	rotation.y = max_steer * (input + (1 - cos(input * 0.5 * PI)) * ackermann)
