package org.usfirst.frc.team6695.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends IterativeRobot {
	private static final int kFrontLeftChannel = 1;
	private static final int kRearLeftChannel = 2;
	private static final int kFrontRightChannel = 4;
	private static final int kRearRightChannel = 3;

	private static final int kJoystickChannel = 0;

	private AlphaMDrive driveTrain;
	private Joystick joystick;
	XboxController xbox;

	TalonSRX rightBoxLiftMotor;
	TalonSRX leftBoxLiftMotor;
	TalonSRX closingBoxLiftMotor;

	Counter c = new Counter(0);

	@Override
	public void robotInit() {
		CameraServer.getInstance().startAutomaticCapture();

		TalonSRX frontLeft = new TalonSRX(kFrontLeftChannel);
		TalonSRX rearLeft = new TalonSRX(kRearLeftChannel);
		TalonSRX frontRight = new TalonSRX(kFrontRightChannel);
		TalonSRX rearRight = new TalonSRX(kRearRightChannel);

		rightBoxLiftMotor = new TalonSRX(5);
		leftBoxLiftMotor = new TalonSRX(6);
		closingBoxLiftMotor = new TalonSRX(7);

		rightBoxLiftMotor.enableCurrentLimit(false);
		leftBoxLiftMotor.enableCurrentLimit(false);
		closingBoxLiftMotor.enableCurrentLimit(true);

		rightBoxLiftMotor.configContinuousCurrentLimit(100, 500);
		leftBoxLiftMotor.configContinuousCurrentLimit(100, 500);
		closingBoxLiftMotor.configContinuousCurrentLimit(2, 500);

		driveTrain = new AlphaMDrive(frontLeft, rearLeft, frontRight, rearRight, ControlMode.PercentOutput);
		driveTrain.setDeadband(.1);
		joystick = new Joystick(kJoystickChannel);
		xbox = new XboxController(1);
	}

	@Override
	public void teleopPeriodic() {
		driveTrain.driveCartesianMichael(joystick.getY(), joystick.getX(), joystick.getZ(), -90, joystick.getThrottle(),
				joystick.getTrigger());
		boxLift(xbox.getYButton(), xbox.getAButton(), xbox.getBButton(), xbox.getXButton());
		//System.out.println(c.get());
	}

	// @Override
	// public void autonomousInit() {
	// //
	// https://wpilib.screenstepslive.com/s/currentCS/m/getting_started/l/826278-2018-game-data-details
	//
	// }
	@Override
	public void autonomousInit() {

		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();

		LEFT: // starting position 'Left'
		if (gameData.charAt(0) == 'L') { // L-- (always going for switch)
			// put switch code for L here
		} else { // R-- (never go for switch)
			if (gameData.charAt(1) == 'L') { // RL-
				// put scale code for L here
			} else { // RR-

				// idk :)
			}
		}
		CENTER: // starting position 'Center'
		if (gameData.charAt(0) == 'L') { // L--
			// put switch code for C here
		} else { // R--
			// put switch code for C here
		}
		RIGHT: // starting position 'Right'
		if (gameData.charAt(0) == 'L') { // L--
			if (gameData.charAt(1) == 'L') { // LL-
				// idk :
			} else { // LR-
				// put scale code for L here
			}
		} else { // R-- (always go for switch)
			// put switch code for R here
		}
	}

	@Override
	public void autonomousPeriodic() {
	}

	public void boxLift(boolean goUp, boolean goDown, boolean close, boolean open) {
		if (goUp) {
			rightBoxLiftMotor.set(ControlMode.PercentOutput, 1);
			leftBoxLiftMotor.set(ControlMode.PercentOutput, 1);
		} else if (goDown) {
			rightBoxLiftMotor.set(ControlMode.PercentOutput, -1);
			leftBoxLiftMotor.set(ControlMode.PercentOutput, -1);
		} else {
			rightBoxLiftMotor.set(ControlMode.PercentOutput, 0);
			leftBoxLiftMotor.set(ControlMode.PercentOutput, 0);
		}
		if (open) closingBoxLiftMotor.set(ControlMode.PercentOutput, 1);
		else if (close) closingBoxLiftMotor.set(ControlMode.PercentOutput, -1);
		else closingBoxLiftMotor.set(ControlMode.PercentOutput, 0);
	}
}
