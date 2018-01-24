package org.usfirst.frc.team6695.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class AlphaMDrive extends RobotDriveBase {
	TalonSRX frontLeft;
	TalonSRX rearLeft;
	TalonSRX frontRight;
	TalonSRX rearRight;

	ControlMode cm;

	public AlphaMDrive(TalonSRX frontLeftMotor, TalonSRX rearLeftMotor, TalonSRX frontRightMotor,
			TalonSRX rearRightMotor, ControlMode mode) {
		frontLeft = frontLeftMotor;
		frontRight = frontRightMotor;
		rearLeft = rearLeftMotor;
		rearRight = rearRightMotor;
		cm = mode;
	}

	public void driveCartesian(double ySpeed, double xSpeed, double zRotation, double gyroAngle, double throttle) {
		System.out.println("yspeed: "+ySpeed);
		System.out.println("xspeed: "+ xSpeed);
		System.out.println("zrotation: "+zRotation);

		ySpeed = limit(ySpeed);
		ySpeed = applyDeadband(ySpeed, m_deadband);

		xSpeed = limit(xSpeed);
		xSpeed = applyDeadband(xSpeed, m_deadband);

		// Compensate for gyro angle.
		Vector2d input = new Vector2d(ySpeed, xSpeed);
		input.rotate(-gyroAngle);

		double[] wheelSpeeds = new double[4];
		wheelSpeeds[MotorType.kFrontLeft.value] = input.x + input.y + zRotation;
		wheelSpeeds[MotorType.kFrontRight.value] = input.x - input.y + zRotation;
		wheelSpeeds[MotorType.kRearLeft.value] = -input.x + input.y + zRotation;
		wheelSpeeds[MotorType.kRearRight.value] = -input.x - input.y + zRotation;

		normalize(wheelSpeeds);
		throttle = 1 - ((throttle + 1) / 2);
		frontLeft.set(cm, wheelSpeeds[MotorType.kFrontLeft.value] * m_maxOutput * throttle);
		frontRight.set(cm, wheelSpeeds[MotorType.kFrontRight.value] * m_maxOutput * throttle);
		rearLeft.set(cm, wheelSpeeds[MotorType.kRearLeft.value] * m_maxOutput * throttle);
		rearRight.set(cm, wheelSpeeds[MotorType.kRearRight.value] * m_maxOutput * throttle);

		m_safetyHelper.feed();
	}
	
	public void driveRigid(double ySpeed, double xSpeed, double gyroAngle, double throttle) {
		driveCartesian(ySpeed, xSpeed, 0.0, gyroAngle, throttle);
	}
	public void driveTurn(double zRotation, double gyroAngle, double throttle) {
		driveCartesian(0.0, 0.0, zRotation, gyroAngle, throttle);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		// TODO Auto-generated method stub
	}

	@Override
	public void stopMotor() {
		frontLeft.set(cm, 0);
		frontRight.set(cm, 0);
		rearLeft.set(cm, 0);
		rearRight.set(cm, 0);
	}

	@Override
	public String getDescription() {
		// TODO Auto-generated method stub
		return null;
	}

}
