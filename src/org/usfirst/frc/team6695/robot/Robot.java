package org.usfirst.frc.team6695.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.Vector2d;

public class Robot extends IterativeRobot {
	private static final int kFrontLeftChannel = 1;
	private static final int kRearLeftChannel = 2;
	private static final int kFrontRightChannel = 4;
	private static final int kRearRightChannel = 3;

	private static final int kJoystickChannel = 0;

	private AlphaMDrive driveTrain;
	private Joystick joystick;
	
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
	}

	@Override
	public void teleopPeriodic() {
		if (joystick.getTrigger()) {
			triggered = true;
			xySpeed = new Vector2d(-joystick.getX(), joystick.getY());
		} else {
			triggered = false;
		}
		
		if (prevTriggered != triggered) {
			xySpeed = driveTrain.driveCurveDown(xySpeed.x, xySpeed.y, 0.0, joystick.getThrottle());
			if (xySpeed.x < 0.1 && xySpeed.y < 0.1)
				prevTriggered = triggered;
		}
		else if (triggered)
			driveTrain.driveTurn(joystick.getZ(), 0.0, joystick.getThrottle());
		else
			driveTrain.driveCartesian(-joystick.getX(), joystick.getY(), 0.0, 0.0, joystick.getThrottle());
	}
}
