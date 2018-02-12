package org.usfirst.frc.team6695.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PIDController;
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

	public void driveCartesianMichael(double ySpeed, double xSpeed, double zRotation, double gyroAngle, double throttle,
			boolean mode) {
		ySpeed = limit(ySpeed);
		ySpeed = applyDeadband(ySpeed, m_deadband);

		xSpeed = limit(xSpeed);
		xSpeed = applyDeadband(xSpeed, m_deadband);

		if (!mode) zRotation = 0;

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

	public void driveLinearY(double speed, PIDController frontLeftPID, PIDController frontRightPID,
			PIDController rearLeftPID, PIDController rearRightPID) {
		Vector2d input = new Vector2d(0.0, speed);
		input.rotate(-90);

		double[] wheelSpeeds = new double[4];
		wheelSpeeds[MotorType.kFrontLeft.value] = input.y;
		wheelSpeeds[MotorType.kFrontRight.value] = -input.y;
		wheelSpeeds[MotorType.kRearLeft.value] = input.y;
		wheelSpeeds[MotorType.kRearRight.value] = -input.y;

		normalize(wheelSpeeds);
		frontLeftPID.setSetpoint(wheelSpeeds[MotorType.kFrontLeft.value] * m_maxOutput);
		frontRightPID.setSetpoint(wheelSpeeds[MotorType.kFrontRight.value] * m_maxOutput);
		rearLeftPID.setSetpoint(wheelSpeeds[MotorType.kRearLeft.value] * m_maxOutput);
		rearRightPID.setSetpoint(wheelSpeeds[MotorType.kRearRight.value] * m_maxOutput);
	}

	public void driveLinearX(double speed, PIDController frontLeftPID, PIDController frontRightPID,
			PIDController rearLeftPID, PIDController rearRightPID) {
		Vector2d input = new Vector2d(speed, 0.0);
		input.rotate(-90);

		double[] wheelSpeeds = new double[4];
		wheelSpeeds[MotorType.kFrontLeft.value] = input.x;
		wheelSpeeds[MotorType.kFrontRight.value] = input.x;
		wheelSpeeds[MotorType.kRearLeft.value] = -input.x;
		wheelSpeeds[MotorType.kRearRight.value] = -input.x;

		normalize(wheelSpeeds);
		frontLeftPID.setSetpoint(wheelSpeeds[MotorType.kFrontLeft.value] * m_maxOutput);
		frontRightPID.setSetpoint(wheelSpeeds[MotorType.kFrontRight.value] * m_maxOutput);
		rearLeftPID.setSetpoint(wheelSpeeds[MotorType.kRearLeft.value] * m_maxOutput);
		rearRightPID.setSetpoint(wheelSpeeds[MotorType.kRearRight.value] * m_maxOutput);
	}

	public void driveRotational(double speed, PIDController frontLeftPID, PIDController frontRightPID,
			PIDController rearLeftPID, PIDController rearRightPID) {
		double[] wheelSpeeds = new double[4];
		wheelSpeeds[MotorType.kFrontLeft.value] = speed;
		wheelSpeeds[MotorType.kFrontRight.value] = speed;
		wheelSpeeds[MotorType.kRearLeft.value] = speed;
		wheelSpeeds[MotorType.kRearRight.value] = speed;

		normalize(wheelSpeeds);
		frontLeftPID.setSetpoint(wheelSpeeds[MotorType.kFrontLeft.value] * m_maxOutput);
		frontRightPID.setSetpoint(wheelSpeeds[MotorType.kFrontRight.value] * m_maxOutput);
		rearLeftPID.setSetpoint(wheelSpeeds[MotorType.kRearLeft.value] * m_maxOutput);
		rearRightPID.setSetpoint(wheelSpeeds[MotorType.kRearRight.value] * m_maxOutput);
	}

	public void driveTurn(double zRotation, double gyroAngle, double throttle) {
		driveCartesian(0.0, 0.0, zRotation, gyroAngle, throttle);
	}

	public void driveArcade(double ySpeed, double xSpeed, double zRotation, double throttle) {
		ySpeed = limit(ySpeed);
		ySpeed = applyDeadband(ySpeed, m_deadband);

		xSpeed = limit(xSpeed);
		xSpeed = applyDeadband(xSpeed, m_deadband);
		Vector2d input = new Vector2d(ySpeed, xSpeed);

		double[] wheelSpeeds = new double[4];
		wheelSpeeds[MotorType.kFrontLeft.value] = input.x - input.y;
		wheelSpeeds[MotorType.kRearLeft.value] = input.x - input.y;
		wheelSpeeds[MotorType.kFrontRight.value] = input.x + input.y;
		wheelSpeeds[MotorType.kRearRight.value] = input.x + input.y;

		normalize(wheelSpeeds);
		throttle = 1 - ((throttle + 1) / 2);
		frontLeft.set(cm, wheelSpeeds[MotorType.kFrontLeft.value] * m_maxOutput * throttle);
		frontRight.set(cm, wheelSpeeds[MotorType.kFrontRight.value] * m_maxOutput * throttle);
		rearLeft.set(cm, wheelSpeeds[MotorType.kRearLeft.value] * m_maxOutput * throttle);
		rearRight.set(cm, wheelSpeeds[MotorType.kRearRight.value] * m_maxOutput * throttle);

		m_safetyHelper.feed();
	}

	public Vector2d driveCurveDown(double ySpeed, double xSpeed, double gyroAngle, double throttle) {
		ySpeed /= 2;
		xSpeed /= 2;
		driveCartesian(ySpeed, xSpeed, 0.0, gyroAngle, throttle);

		return new Vector2d(ySpeed, xSpeed);
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
