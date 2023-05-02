using Godot;

public partial class Car : RigidBody3D
{
	[Export]
	float MaxSteer = 0.5f;
	[Export]
	float SteerSpeed = 5.0f;
	[Export]
	float MaxBrakeForce = 2500.0f;

	[Export]
	float MaxTorque = 250;
	[Export]
	float MaxEngineRPM = 8000;
	[Export]
	float IdleEngineRPM = 900;

	[Export]
	Curve EngineTorqueCurve;

	[Export]
	float EngineDrag = 0.03f;
	[Export]
	float EngineBrake = 10.0f;
	[Export]
	float EngineMoment = 0.25f;
	[Export]
	AudioStream EngineSound;

	[Export]
	float[] GearRatios = { 3.0f, 2.2f, 1.7f, 1.4f, 1.0f };
	[Export]
	float FinalDrive = 3.91f;
	[Export]
	float ReverseRatio = 3.9f;
	[Export]
	float GearboxInertia = 0.2f;


	const float AV_2_RPM = 60.0f / Mathf.Tau;

	float InputThrottle = 0;
	float InputSteering = 0;
	float InputBrake = 0;
	float InputHandbrake = 0;

	float SteeringAmount = 0;
	int SelectedGear = 0;
	float DragTorque = 0;
	float TorqueOut = 0;

	float BrakeForce = 0;
	float RPM = 0;
	float Speedo = 0;
	float DriveInertia = 0;
	float R_Split = 0.5f;
	float WheelRadius = 0;

	float AverageWheelspinFront = 0;
	float AverageWheelspinRear = 0;

	[Export]
	NodePath PathToAudioPlayer;
	AudioStreamPlayer AudioPlayer;

	[Export]
	NodePath PathToWheelFL;
	[Export]
	NodePath PathToWheelFR;
	[Export]
	NodePath PathToWheelRL;
	[Export]
	NodePath PathToWheelRR;

	WheelSuspension Wheel_FL;
	WheelSuspension Wheel_FR;
	WheelSuspension Wheel_RL;
	WheelSuspension Wheel_RR;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
		AudioPlayer = GetNode<AudioStreamPlayer>(PathToAudioPlayer);

		Wheel_FL = GetNode<WheelSuspension>(PathToWheelFL);
		Wheel_FR = GetNode<WheelSuspension>(PathToWheelFR);
		Wheel_RL = GetNode<WheelSuspension>(PathToWheelRL);
		Wheel_RR = GetNode<WheelSuspension>(PathToWheelRR);
	}

	public override void _UnhandledInput(InputEvent @event)
	{
		if (@event.IsActionPressed("ShiftUp"))
		{
			ShiftUp();
		}
		if (@event.IsActionPressed("ShiftDown"))
		{
			ShiftDown();
		}
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
		InputThrottle = Input.GetActionStrength("Throttle");
		InputBrake = Input.GetActionStrength("Brake");
		InputSteering = Input.GetActionStrength("SteerLeft") - Input.GetActionStrength("SteerRight");

		BrakeForce = MaxBrakeForce * InputBrake * 0.25f;

		Speedo = AverageWheelspinFront * WheelRadius * 3.6f;

		EngineSoundCalc();

		//GD.Print(AverageWheelspinRear);
	}

	public override void _PhysicsProcess(double delta)
	{
		Wheel_RL.ApplyForces((float)delta);
		Wheel_RR.ApplyForces((float)delta);
		Wheel_FL.ApplyForces((float)delta);
		Wheel_FR.ApplyForces((float)delta);

		if (InputSteering < SteeringAmount)
		{
			SteeringAmount -= SteerSpeed * (float)delta;
			if (InputSteering > SteeringAmount)
			{
				SteeringAmount = InputSteering;
			}
		}
		else if (InputSteering > SteeringAmount)
		{
			SteeringAmount += SteerSpeed * (float)delta;
			if (InputSteering < SteeringAmount)
			{
				SteeringAmount = InputSteering;
			}
		}

		Wheel_FL.Steer(SteeringAmount, MaxSteer);
		Wheel_FR.Steer(SteeringAmount, MaxSteer);

		DragTorque = EngineBrake + RPM * EngineDrag;
		TorqueOut = (EngineTorque(RPM) + DragTorque) * InputThrottle;
		RPM += AV_2_RPM * (float)delta * (TorqueOut - DragTorque) / EngineMoment;

		if (RPM > MaxEngineRPM)
		{
			TorqueOut = 0.0f;
			RPM -= 500;
		}

		if (SelectedGear == 0)
		{
			FreeWheel((float)delta);
		}
		else
		{
			Engage((float)delta);
		}
		float ClutchRPM = IdleEngineRPM;
		if (Mathf.Abs(SelectedGear) == 1)
		{
			ClutchRPM += InputThrottle * 2000.0f;
		}
		RPM = Mathf.Max(RPM, ClutchRPM);
		RPM = Mathf.Max(RPM, IdleEngineRPM);
	}

	public float EngineTorque(float _rpm)
	{
		float RPMFactor = Mathf.Clamp(_rpm / MaxEngineRPM, 0.0f, 1.0f);
		float TorqueFactor = EngineTorqueCurve.SampleBaked(RPMFactor);
		return TorqueFactor * MaxTorque;

	}

	public void FreeWheel(float _delta)
	{
		AverageWheelspinFront = 0.0f;
		AverageWheelspinFront += (Wheel_FL.Spin + Wheel_FR.Spin) * 0.5f;

		Wheel_FL.ApplyTorque(0.0f, 0.0f, BrakeForce, _delta);
		Wheel_FR.ApplyTorque(0.0f, 0.0f, BrakeForce, _delta);
		Wheel_RL.ApplyTorque(0.0f, 0.0f, BrakeForce, _delta);
		Wheel_RR.ApplyTorque(0.0f, 0.0f, BrakeForce, _delta);
	}

	public void Engage(float _delta)
	{
		AverageWheelspinRear = 0.0f;
		AverageWheelspinFront = 0.0f;

		AverageWheelspinRear += (Wheel_RL.Spin + Wheel_RR.Spin) * 0.5f;
		AverageWheelspinFront += (Wheel_FL.Spin + Wheel_FR.Spin) * 0.5f;

		float NetDrive = (TorqueOut - DragTorque) * GearRatio();

		if (AverageWheelspinRear * (float)Mathf.Sign(GearRatio()) < 0)
		{
			NetDrive += DragTorque * GearRatio();
		}

		RWD(NetDrive, _delta);
		RPM = AverageWheelspinRear * GearRatio() * AV_2_RPM;
	}

	public void RWD(float _drive, float _delta)
	{
		DriveInertia = EngineMoment + (Mathf.Pow(Mathf.Abs(GearRatio()), 2) * GearboxInertia);

		Wheel_RL.ApplyTorque(_drive * 0.5f, DriveInertia, BrakeForce, _delta);
		Wheel_RR.ApplyTorque(_drive * 0.5f, DriveInertia, BrakeForce, _delta);
		Wheel_FL.ApplyTorque(0.0f, 0.0f, BrakeForce, _delta);
		Wheel_FR.ApplyTorque(0.0f, 0.0f, BrakeForce, _delta);
	}

	public void FWD(float _drive, float _delta)
	{
		DriveInertia = EngineMoment + (Mathf.Pow(Mathf.Abs(GearRatio()), 2) * GearboxInertia);

		Wheel_FL.ApplyTorque(_drive * 0.5f, DriveInertia, BrakeForce, _delta);
		Wheel_FR.ApplyTorque(_drive * 0.5f, DriveInertia, BrakeForce, _delta);
		Wheel_RL.ApplyTorque(0.0f, 0.0f, BrakeForce, _delta);
		Wheel_RR.ApplyTorque(0.0f, 0.0f, BrakeForce, _delta);
	}

	public void AWD(float _drive, float _delta)
	{
		DriveInertia = EngineMoment + (Mathf.Pow(Mathf.Abs(GearRatio()), 2) * GearboxInertia);

		Wheel_FL.ApplyTorque(_drive * 0.15f, DriveInertia, BrakeForce, _delta);
		Wheel_FR.ApplyTorque(_drive * 0.15f, DriveInertia, BrakeForce, _delta);
		Wheel_RL.ApplyTorque(_drive * 0.35f, DriveInertia, BrakeForce, _delta);
		Wheel_RR.ApplyTorque(_drive * 0.35f, DriveInertia, BrakeForce, _delta);
	}

	public float GearRatio()
	{
		if (SelectedGear > 0)
		{
			return GearRatios[SelectedGear - 1] * FinalDrive;
		}
		else if (SelectedGear == -1)
		{
			return -ReverseRatio * FinalDrive;
		}
		else
		{
			return 0.0f;
		}
	}

	public void ShiftUp()
	{
		if (SelectedGear < GearRatios.Length)
		{
			SelectedGear += 1;
		}
	}
	public void ShiftDown()
	{
		if (SelectedGear > -1)
		{
			SelectedGear -= 1;
		}
	}
	public void EngineSoundCalc()
	{
		float PitchScaler = RPM / 1000;
		if (RPM >= IdleEngineRPM && RPM < MaxEngineRPM)
		{
			if (AudioPlayer.Stream != EngineSound)
			{
				AudioPlayer.Stream = EngineSound;
			}
			if (!AudioPlayer.Playing)
			{
				AudioPlayer.Play();
			}
		}
		if (PitchScaler > 0.1f)
		{
			AudioPlayer.PitchScale = PitchScaler;
		}
	}
}
