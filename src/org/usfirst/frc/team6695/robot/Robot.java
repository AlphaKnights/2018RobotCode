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


	ModeSelector modeTypes;
	
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
		
		modeTypes = new ModeSelector(1, 2, 3);
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

	@Override
	public void autonomousInit() {

		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		autonomousPathfinding(gameData, modeTypes.getStartingMode());

	}

	public void autonomousPathfinding(String gameData, boolean[] getStartingMode) {

		if (getStartingMode[0] == false && getStartingMode[1] == false && getStartingMode[2] == false) {
			System.out.println("DISABLED_START with switch priority");

		} else if (getStartingMode[0] == false && getStartingMode[1] == false && getStartingMode[2] == true) {
			System.out.println("DISABLED_START with scale priority");

		} else if (getStartingMode[0] == false && getStartingMode[1] == true && getStartingMode[2] == false) {
			System.out.println("RIGHT_START with switch priority");

		} else if (getStartingMode[0] == false && getStartingMode[1] == true && getStartingMode[2] == true) {
			System.out.println("RIGHT_START with scale priority");

		} else if (getStartingMode[0] == true && getStartingMode[1] == false && getStartingMode[2] == false) {
			System.out.println("LEFT_START with switch priority");

		} else if (getStartingMode[0] == true && getStartingMode[1] == false && getStartingMode[2] == true) {
			System.out.println("LEFT_START with scale priority");

		} else if (getStartingMode[0] == true && getStartingMode[1] == true && getStartingMode[2] == false) {
			System.out.println("CENTER_START with switch priority");

			if (gameData.charAt(0) == 'L') {
				// Put code for CENTER_START with switch priority LEFT here
			} else if (gameData.charAt(0) == 'R') {
				// Put code for CENTER_START with switch priority RIGHT here
			} else {
				System.err.println("Couldn't determine LEFT or RIGHT switch priority");
			}
		} else if (getStartingMode[0] == true && getStartingMode[1] == true && getStartingMode[2] == true) {
			System.out.println("CENTER_START with scale priority");

			if (gameData.charAt(1) == 'L') {
				// Put code for CENTER_START with scale priority LEFT here
			} else if (gameData.charAt(1) == 'R') {
				// Put code for CENTER_START with scale priority RIGHT here
			} else {
				System.err.println("Couldn't determine LEFT or RIGHT scale priority");
			}
		} else {
			System.err.println("Couldn't determine pathfinding switch states...");
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
