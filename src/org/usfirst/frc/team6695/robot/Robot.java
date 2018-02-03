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

	private AlphaMDrive driveTrain;
	private Joystick joystick;
	XboxController xbox;

	TalonSRX rightBoxLiftMotor;
	TalonSRX leftBoxLiftMotor;
	TalonSRX closingBoxLiftMotor;

	Counter LiftLeftEncoder;
	Counter LiftRightEncoder;

	@Override
	public void robotInit() {
		CameraServer.getInstance().startAutomaticCapture();

		TalonSRX frontLeft = new TalonSRX(Config.DriveTrainFrontLeft);
		TalonSRX rearLeft = new TalonSRX(Config.DriveTrainRearLeft);
		TalonSRX frontRight = new TalonSRX(Config.DriveTrainFrontRight);
		TalonSRX rearRight = new TalonSRX(Config.DriveTrainRearRight);

		rightBoxLiftMotor = new TalonSRX(Config.LiftRightMotor);
		leftBoxLiftMotor = new TalonSRX(Config.LiftLeftMotor);
		closingBoxLiftMotor = new TalonSRX(Config.liftGrabberMotor);

		rightBoxLiftMotor.enableCurrentLimit(false);
		leftBoxLiftMotor.enableCurrentLimit(false);
		closingBoxLiftMotor.enableCurrentLimit(true);

		rightBoxLiftMotor.configContinuousCurrentLimit(100, 500);
		leftBoxLiftMotor.configContinuousCurrentLimit(100, 500);
		closingBoxLiftMotor.configContinuousCurrentLimit(2, 500);

		driveTrain = new AlphaMDrive(frontLeft, rearLeft, frontRight, rearRight, ControlMode.PercentOutput);
		driveTrain.setDeadband(.1);
		joystick = new Joystick(Config.JoystickChannel);
		xbox = new XboxController(Config.XBoxChannel);

		LiftLeftEncoder = new Counter(Config.LiftLeftEncoderPort);
		LiftRightEncoder = new Counter(Config.LiftRightEncoderPort);
	}

	@Override
	public void teleopPeriodic() {
		driveTrain.driveCartesianMichael(joystick.getY(), joystick.getX(), joystick.getZ(), -90, joystick.getThrottle(),
				joystick.getTrigger());
		boxLift(xbox.getYButton(), xbox.getAButton(), xbox.getBButton(), xbox.getXButton());
		// System.out.println(c.get());
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

	/**
	 * This is the box Lift Code
	 * 
	 * @param goUp
	 *            Button to go up. Boolean Value
	 * @param goDown
	 *            Button to go down. Boolean Value
	 * @param close
	 *            Button to close the Thing. Boolean Value
	 * @param open
	 *            Button to open the thing. Boolean Value
	 */
	public void boxLift(boolean goUp, boolean goDown, boolean close, boolean open) {
		if (goDown) {
			LiftLeftEncoder.setReverseDirection(true);
			LiftRightEncoder.setReverseDirection(true);
		}
		if (goUp) {
			LiftLeftEncoder.setReverseDirection(false);
			LiftRightEncoder.setReverseDirection(false);
		}

		if (goUp) {
			if ((LiftRightEncoder.get() + LiftLeftEncoder.get()) / 2 < Config.EncoderTopValue) {
				if ((LiftRightEncoder.get() - LiftLeftEncoder.get()) >= Config.EncoderRange) {
					rightBoxLiftMotor.set(ControlMode.PercentOutput, .5);
					leftBoxLiftMotor.set(ControlMode.PercentOutput, 1);
				} else if ((LiftLeftEncoder.get() - LiftRightEncoder.get()) >= Config.EncoderRange) {
					rightBoxLiftMotor.set(ControlMode.PercentOutput, 1);
					leftBoxLiftMotor.set(ControlMode.PercentOutput, .5);
				} else {
					rightBoxLiftMotor.set(ControlMode.PercentOutput, 1);
					leftBoxLiftMotor.set(ControlMode.PercentOutput, 1);
				}
			}

		} else if (goDown) {
			if (((LiftRightEncoder.get() + LiftLeftEncoder.get()) / 2) < 4) {
				if ((LiftRightEncoder.get() - LiftLeftEncoder.get()) >= Config.EncoderRange) {
					rightBoxLiftMotor.set(ControlMode.PercentOutput, -1);
					leftBoxLiftMotor.set(ControlMode.PercentOutput, -.5);
				} else if ((LiftLeftEncoder.get() - LiftRightEncoder.get()) >= Config.EncoderRange) {
					rightBoxLiftMotor.set(ControlMode.PercentOutput, -.5);
					leftBoxLiftMotor.set(ControlMode.PercentOutput, -1);
				} else {
					rightBoxLiftMotor.set(ControlMode.PercentOutput, -1);
					leftBoxLiftMotor.set(ControlMode.PercentOutput, -1);
				}
			}
		} else {
			rightBoxLiftMotor.set(ControlMode.PercentOutput, 0);
			leftBoxLiftMotor.set(ControlMode.PercentOutput, 0);
		}
		if (open) closingBoxLiftMotor.set(ControlMode.PercentOutput, 1);
		else if (close) closingBoxLiftMotor.set(ControlMode.PercentOutput, -1);
		else closingBoxLiftMotor.set(ControlMode.PercentOutput, 0);
	}
}
