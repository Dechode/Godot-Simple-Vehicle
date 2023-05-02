using Godot;
using System;

public partial class WheelSuspension : RayCast3D
{
    [Export]
    public float SpringLength = 0.2f;
    [Export]
    public float SpringStiffnes = 10000;
    [Export]
    public float Damping_Bump = 5000;
    [Export]
    public float Damping_Rebound = 6000;

    [Export]
    private float WheelInertia = 1.6f;
    [Export]
    private float TireRadius = 0.35f;
    [Export]
    private float Ackermann = 0.15f;

    [Export]
    public Curve LateralForce;
    [Export]
    public Curve LongitudinalForce;

    float mu = 1.0f;
    float Y_Force = 0.0f;

    public float Spin = 0.0f;
    float Velocity_Z = 0;

    Vector2 ForceVector = Vector2.Zero;
    Vector2 SlipVector = Vector2.Zero;
    Vector3 PrevPosition = Vector3.Zero;

    float peak_sr = 0.10f;
    float peak_sa = 0.10f;

    Vector3 GlobalVelocity_ms;
    Vector3 LocalVelocity;

    Car CarParent;
    Node3D WheelMesh;

    float PrevCompress = 0.0f;
    float SpringCurrLength = 0.0f;


    private float SpringLoad_MM = 0;
    private float SpringLoad_Newton = 0;
    private float SpringVelocity_MM_Sec = 0;

    private Vector3 PrevGlobalPosition;
    private float PrevSpringLoad_MM = 0;

    float NetTorque = 0.0f;

    public override void _Ready()
    {
        SpringCurrLength = SpringLength;

        TargetPosition = Vector3.Down * (SpringLength + TireRadius);

        CarParent = GetParent<Car>();
        PrevGlobalPosition = GlobalPosition;
        WheelMesh = GetNode<Node3D>("WheelMesh");

        peak_sa = LateralForce.GetPointPosition(1).X;
        peak_sr = LongitudinalForce.GetPointPosition(1).X;
    }


    public override void _Process(double delta)
    {
        Vector3 WheelPos = WheelMesh.Position;
        WheelPos.Y = -SpringCurrLength;
        WheelMesh.Position = WheelPos;

        WheelMesh.RotateX(Mathf.Wrap(-Spin * (float)delta, 0, Mathf.Tau));

    }

    public void ApplyForces(float _delta)
    {
        LocalVelocity = ((GlobalTransform.Origin - PrevGlobalPosition) / _delta) * GlobalTransform.Basis;
        Velocity_Z = -LocalVelocity.Z;
        Vector2 PlanarVector = new Vector2(LocalVelocity.X, LocalVelocity.Z).Normalized();
        PlanarVector = PlanarVector.Normalized();
        PrevGlobalPosition = GlobalTransform.Origin;

        if (IsColliding())
        {
            SpringCurrLength = GetCollisionPoint().DistanceTo(GlobalTransform.Origin) - TireRadius;
        }
        else
        {
            SpringCurrLength = SpringLength;
        }

        float Compress = Mathf.Clamp(1.0f - SpringCurrLength / SpringLength, 0.0f, 1.0f);
        Y_Force = SpringStiffnes * Compress;

        float CompressSpeed = Compress - PrevCompress;

        if (CompressSpeed >= 0)
        {
            Y_Force += Damping_Bump * CompressSpeed * SpringLength / _delta;
        }
        else
        {
            Y_Force += Damping_Rebound * CompressSpeed * SpringLength / _delta;
        }

        Y_Force = Mathf.Max(0.0f, Y_Force);
        PrevCompress = Compress;

        SlipVector.X = Mathf.Asin(Mathf.Clamp(-PlanarVector.X, -1.0f, 1.0f));
        SlipVector.Y = 0.0f;

        if (Velocity_Z != 0.0f)
        {
            SlipVector.Y = -(Spin * TireRadius - Velocity_Z) / Mathf.Abs(Velocity_Z);
        }
        else
        {
            if (Spin == 0.0f)
            {
                SlipVector.Y = 0.0f;
            }
            else
            {
                SlipVector.Y = 0.01f * Spin;
            }
        }

        float SlipRatio = SlipVector.Y;
        float SlipAngle = SlipVector.X;

        float Normalized_sr = SlipRatio / peak_sr;
        float Normalized_sa = SlipAngle / peak_sa;
        float ResultantSlip = MathF.Sqrt(Mathf.Pow(Normalized_sr, 2) + Mathf.Pow(Normalized_sa, 2));

        float sr_Modified = ResultantSlip * peak_sr;
        float sa_Modified = ResultantSlip * peak_sa;

        float Force_X = 0.0f;
        float Force_Z = 0.0f;

        Force_X = TireForce(Mathf.Abs(sa_Modified), Y_Force, LateralForce) * (float)Mathf.Sign(SlipVector.X);
        Force_Z = TireForce(Mathf.Abs(sr_Modified), Y_Force, LongitudinalForce) * (float)Mathf.Sign(SlipVector.Y);

        if (ResultantSlip != 0.0f)
        {
            ForceVector.X = Force_X * Mathf.Abs(Normalized_sa / ResultantSlip);
            ForceVector.Y = Force_Z * Mathf.Abs(Normalized_sr / ResultantSlip);
        }
        else
        {
            ForceVector.X = 0.0f;
            ForceVector.Y = 0.0f;
        }



        //Apply the forces on the body
        if (IsColliding())
        {
            Vector3 Contact = GetCollisionPoint() - CarParent.GlobalTransform.Origin;
            Vector3 Normal = GetCollisionNormal();

            CarParent.ApplyForce(Normal * Y_Force, Contact);
            CarParent.ApplyForce(GlobalTransform.Basis.X * ForceVector.X, Contact);
            CarParent.ApplyForce(GlobalTransform.Basis.Z * ForceVector.Y, Contact);
        }
        else
        {
            Spin -= (float)Mathf.Sign(Spin) * _delta * 2 / WheelInertia;
        }

    }

    public float ApplyTorque(float _drive, float _drive_inertia, float _braketorque, float _delta)
    {
        float PrevSpin = Spin;

        NetTorque = ForceVector.Y * TireRadius;
        NetTorque += _drive;

        if (Spin < 5 && _braketorque > Mathf.Abs(NetTorque))
        {
            Spin = 0.0f;
        }
        else
        {
            NetTorque -= _braketorque * (float)Mathf.Sign(Spin);
            Spin += _delta * NetTorque / (WheelInertia + _drive_inertia);
        }

        if (_drive * _delta == 0)
        {
            return 0.5f;
        }
        else
        {
            return (Spin - PrevSpin) * (WheelInertia + _drive_inertia) / (_drive * _delta);
        }
    }

    public float TireForce(float _slip, float _normal_load, Curve _tire_curve)
    {
        float friction = _normal_load * mu;
        return _tire_curve.SampleBaked(Mathf.Abs(_slip)) * friction * (float)Mathf.Sign(_slip);
    }

    public void Steer(float _input, float _maxSteere)
    {
        Vector3 SteereVector = Rotation;
        SteereVector.Y = _maxSteere * (_input + (1.0f - Mathf.Cos(_input * 0.5f * Mathf.Pi)) * Ackermann);
        Rotation = SteereVector;
    }

}
