package org.usfirst.frc.team6695.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.Vector2d;

public class Robot extends IterativeRobot {
	private static final int kFrontLeftChannel = 1;
	private static final int kRearLeftChannel = 2;
	private static final int kFrontRightChannel = 4;
	private static final int kRearRightChannel = 3;

	private static final int kJoystickChannel = 0;

	private AlphaMDrive driveTrain;
	private Joystick joystick;
	XboxController xbox;

	private boolean prevTriggered = false;
	private boolean triggered = false;
	private Vector2d xySpeed;

	@Override
	public void robotInit() {
		CameraServer.getInstance().startAutomaticCapture();

		TalonSRX frontLeft = new TalonSRX(kFrontLeftChannel);
		TalonSRX rearLeft = new TalonSRX(kRearLeftChannel);
		TalonSRX frontRight = new TalonSRX(kFrontRightChannel);
		TalonSRX rearRight = new TalonSRX(kRearRightChannel);

		driveTrain = new AlphaMDrive(frontLeft, rearLeft, frontRight, rearRight, ControlMode.PercentOutput);
		driveTrain.setDeadband(.1);
		joystick = new Joystick(kJoystickChannel);
		xbox = new XboxController(1);
	}

	@Override
	public void teleopPeriodic() {

		driveTrain.driveCartesianMichael(joystick.getY(), joystick.getX(), joystick.getZ(), -90, joystick.getThrottle(),
				joystick.getTrigger());

	}
}
