/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6695.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive
 * class.
 */
public class Robot extends IterativeRobot {
	private static final int kFrontLeftChannel = 1;
	private static final int kRearLeftChannel = 2;
	private static final int kFrontRightChannel = 4;
	private static final int kRearRightChannel = 3;

	private static final int kJoystickChannel = 0;

	private AlphaMDrive driveTrain;
	private Joystick joystick;

	@Override
	public void robotInit() {
		TalonSRX frontLeft = new TalonSRX(kFrontLeftChannel);
		TalonSRX rearLeft = new TalonSRX(kRearLeftChannel);
		TalonSRX frontRight = new TalonSRX(kFrontRightChannel);
		TalonSRX rearRight = new TalonSRX(kRearRightChannel);

		// Invert the left side motors.
		// You may need to change or remove this to match your robot.
		//frontLeft.setInverted(true);
		//rearLeft.setInverted(true);

		driveTrain = new AlphaMDrive(frontLeft, rearLeft, frontRight, rearRight, ControlMode.PercentOutput);
		driveTrain.setDeadband(.01);
		joystick = new Joystick(kJoystickChannel);
	}

	@Override
	public void teleopPeriodic() {
		// Use the joystick X axis for lateral movement, Y axis for forward
		// movement, and Z axis for rotation.
		driveTrain.driveCartesian(joystick.getX(), joystick.getY(),
				joystick.getZ(), 0.0, joystick.getThrottle());
	}
}
