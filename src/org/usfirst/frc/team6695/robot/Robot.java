package org.usfirst.frc.team6695.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends IterativeRobot {

	private AlphaMDrive driveTrain;
	private Joystick joystick;
	XboxController xbox;

	TalonSRX rightBoxLiftMotor;
	TalonSRX leftBoxLiftMotor;
	TalonSRX closingBoxLiftMotor;

	ModeSelector switches;
	Position fieldPosition;

	Counter LiftLeftEncoder;
	Counter LiftRightEncoder;

	Counter DTEncFR;
	Counter DTEncFL;
	Counter DTEncRL;
	Counter DTEncRR;

	Timer autotime;

	public enum Position {
		Left,
		Middle,
		Right;
	}

	AHRS navx;

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

		switches = new ModeSelector(10, 11, 12, 13, 14, 15, 16, 17);
		joystick = new Joystick(Config.JoystickChannel);
		xbox = new XboxController(Config.XBoxChannel);

		LiftLeftEncoder = new Counter(Config.LiftLeftEncoderPort);
		LiftRightEncoder = new Counter(Config.LiftRightEncoderPort);

		DTEncFR = new Counter(Config.DrivetrainEncoderFrontRight);
		DTEncFL = new Counter(Config.DrivetrainEncoderFrontLeft);
		DTEncRR = new Counter(Config.DrivetrainEncoderRearRight);
		DTEncRL = new Counter(Config.DrivetrainEncoderRearLeft);

		autotime = new Timer();

		try {
			navx = new AHRS(SPI.Port.kMXP);
		} catch (Exception e) {
			e.printStackTrace();
			System.err.println("YO YO YO! That Navx board is not working!!!");
		}
	}

	boolean teleOpCalled = false;

	@Override
	public void teleopInit() {
		teleOpCalled = true;
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
		autotime.reset();
		autotime.start();

		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		autonomousPathfinding(gameData, switches.getSwitches());
	}

	/** Go to the scale */
	boolean scalePriority = false;
	/** Go to the switch */
	boolean switchPriority = false;
	/** Cross the line */
	boolean autonomousStraight = false;

	public void autonomousPathfinding(String gameData, boolean[] input) {
		// 0 - left
		// 1 - middle
		// 2 - right
		// 3 - straight only
		// 4 - target scale
		// 5 - target switch
		// 6 - delay 2s
		// 7 - delay 5s

		// TODO: break on autonomous end
		if (input[0]) {
			fieldPosition = Position.Left;
		} else if (input[1]) {
			fieldPosition = Position.Middle;
		} else if (input[2]) {
			fieldPosition = Position.Right;
		} else {
			System.err.println("Couldn't determine fieldPosition");
		}

		if (input[3]) {
			autonomousStraight = true;
			System.out.println("Go straight");
		} else if (input[4]) {
			scalePriority = true;
			System.out.println("Target scale");
		} else if (input[5]) {
			switchPriority = true;
			System.out.println("Target switch");
		}

		if (input[6]) try {
			Thread.sleep(2000);
		} catch (InterruptedException ex) {
			Thread.currentThread().interrupt();
		}

		if (input[7]) try {
			Thread.sleep(5000);
		} catch (InterruptedException ex) {
			Thread.currentThread().interrupt();
		}

		// TODO: Verify the logic of gameData and if switchPriority or scalePriority
		// interferes with it
		if (fieldPosition == Position.Left) {

			if (autonomousStraight) {
				System.out.println("go straight");
			} else if (scalePriority) {
				if (gameData.charAt(1) == 'L') {
					// Put left auto code here with scale priority
				} else if (gameData.charAt(1) == 'R') {
					// Put autonomous straight code here
				} else {
					System.err.println("Couldn't determine gameData.charAt(1)");
				}
			} else if (switchPriority) {
				if (gameData.charAt(0) == 'L') {
					// Put left auto code here with switch priority
				} else if (gameData.charAt(0) == 'R') {
					// Put autonomous straight code here
				} else {
					System.err.println("Couldn't determine gameData.charAt(0)");
				}
			}

		} else if (fieldPosition == Position.Middle) {

			if (autonomousStraight) {
				System.out.println("Cross base line");
			} else if (scalePriority) {
				if (gameData.charAt(1) == 'L') {
					// Put left auto code here with scale priority
				} else if (gameData.charAt(1) == 'R') {
					// Put right auto code here with scale priority
				} else {
					System.err.println("Couldn't determine gameData.charAt(1)");
				}
			} else if (switchPriority) {
				if (gameData.charAt(0) == 'L') {
					// Put left auto code here with switch priority
				} else if (gameData.charAt(0) == 'R') {
					// Put right auto code here with switch priority
				} else {
					System.err.println("Couldn't determine gameData.charAt(0)");
				}
			}

		} else if (fieldPosition == Position.Right) {

			if (autonomousStraight) {
				System.out.println("Go straight");
			} else if (scalePriority) {
				if (gameData.charAt(1) == 'L') {
					// Put autonomous straight code here
				} else if (gameData.charAt(1) == 'R') {
					// Put right auto code here with scale priority
				} else {
					System.err.println("Couldn't determine gameData.charAt(1)");
				}
			} else if (switchPriority) {
				if (gameData.charAt(0) == 'L') {
					// Put autonomous straight code here
				} else if (gameData.charAt(0) == 'R') {
					// Put right auto code here with switch priority
				} else {
					System.err.println("Couldn't determine gameData.charAt(0)");
				}
			}

		}

		// TODO: It doesn't help us to do nothing when the switches are set incorrectly
		// so do the bare minimum in case of human error by the driveteam (go straight)

		// 5-4-5 feet to cross base line in middle positions
		// 7 feet to go straight
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

	@Override
	public void testInit() {
		DTEncFL.reset();
		DTEncFR.reset();
		DTEncRL.reset();
		DTEncRR.reset();
	}

	@Override
	public void testPeriodic() {
		// driveTrain.driveCartesianMichael(joystick.getY(), joystick.getX(),
		// joystick.getZ(), -90, joystick.getThrottle(),
		// joystick.getTrigger());
		//
		// System.out.println("FRONTLEFT: " + DTEncFL.get() + ", FRONTRIGHT: " +
		// DTEncFR.get() + ", REARLEFT: " + DTEncRL.get() + ", REARRIGHT: " +
		// DTEncRR.get());
		System.out.println();
		for (boolean c : switches.getSwitches()) {
			System.out.print(c + " ");
		}
		System.out.println();
	}

	// TODO: Implement PID Feedback & Control

	public void DriveX(double speed, double feet) {
		DTEncFL.reset();
		DTEncFR.reset();
		DTEncRL.reset();
		DTEncRR.reset();

		while (Math.abs(DTEncFL.get()) < Math.abs(feet * Config.encUnit)
				&& Math.abs(DTEncFR.get()) < Math.abs(feet * Config.encUnit)
				&& Math.abs(DTEncRR.get()) < Math.abs(feet * Config.encUnit)
				&& Math.abs(DTEncRL.get()) < Math.abs(feet * Config.encUnit) && !teleOpCalled && autotime.get() < 15) {
			driveTrain.driveLinearX(speed);
		}
	}

	public void DriveY(double speed, double feet) {
		DTEncFL.reset();
		DTEncFR.reset();
		DTEncRL.reset();
		DTEncRR.reset();

		while (Math.abs(DTEncFL.get()) < Math.abs(feet * Config.encUnit)
				&& Math.abs(DTEncFR.get()) < Math.abs(feet * Config.encUnit)
				&& Math.abs(DTEncRR.get()) < Math.abs(feet * Config.encUnit)
				&& Math.abs(DTEncRL.get()) < Math.abs(feet * Config.encUnit) && !teleOpCalled && autotime.get() < 15) {
			driveTrain.driveLinearY(speed);
		}
	}

	public void DriveRotational(double speed, double degrees) {
		DTEncFL.reset();
		DTEncFR.reset();
		DTEncRL.reset();
		DTEncRR.reset();

		while (Math.abs(DTEncFL.get()) < Math.abs(degrees * Config.degUnit)
				&& Math.abs(DTEncFR.get()) < Math.abs(degrees * Config.degUnit)
				&& Math.abs(DTEncRR.get()) < Math.abs(degrees * Config.degUnit)
				&& Math.abs(DTEncRL.get()) < Math.abs(degrees * Config.degUnit) && !teleOpCalled
				&& autotime.get() < 15) {
			driveTrain.driveRotational(speed);
		}
	}
}
