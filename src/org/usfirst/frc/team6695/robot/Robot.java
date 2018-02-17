package org.usfirst.frc.team6695.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;

public class Robot extends IterativeRobot {

	public AlphaMDrive driveTrain;
	Joystick joystick;
	XboxController xbox;

	TalonSRX boxLiftMotor;
	TalonSRX closingBoxLiftMotor;
	TalonSRX liftSpinMotor;

	TalonSRX forkLiftMotor;

	ModeSelector switches;
	Position fieldPosition;

	Counter boxLiftEncoder;

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

	TalonSRX frontLeft;
	TalonSRX rearLeft;
	TalonSRX frontRight;
	TalonSRX rearRight;

	Ultrasonic ultra1;
	
	NetworkTableEntry DeadBandEntry;

	public int liftEncoderDistance;

	public void tableSetup() {
		NetworkTableInstance nts = NetworkTableInstance.getDefault();
		NetworkTable table = nts.getTable("SmartDashboard");
		DeadBandEntry = table.getEntry("deadband");
		DeadBandEntry.setDouble(.1);
	}

	@Override
	public void robotInit() {
		tableSetup();

		CameraServer.getInstance().startAutomaticCapture();

		frontLeft = new TalonSRX(Config.DriveTrainFrontLeft);
		rearLeft = new TalonSRX(Config.DriveTrainRearLeft);
		frontRight = new TalonSRX(Config.DriveTrainFrontRight);
		rearRight = new TalonSRX(Config.DriveTrainRearRight);

		boxLiftMotor = new TalonSRX(Config.LiftMotor);
		closingBoxLiftMotor = new TalonSRX(Config.liftGrabberMotor);
		liftSpinMotor = new TalonSRX(Config.LiftSpinMotor);

		forkLiftMotor = new TalonSRX(Config.ForkLiftMotor);

		boxLiftMotor.enableCurrentLimit(true);
		closingBoxLiftMotor.enableCurrentLimit(true);
		liftSpinMotor.enableCurrentLimit(true);

		boxLiftMotor.configContinuousCurrentLimit(10, 500);
		closingBoxLiftMotor.configContinuousCurrentLimit(10, 500);
		liftSpinMotor.configContinuousCurrentLimit(24, 500);

		switches = new ModeSelector(10, 11, 12, 13, 14, 15, 16, 17);
		joystick = new Joystick(Config.JoystickChannel);
		xbox = new XboxController(Config.XBoxChannel);

		boxLiftEncoder = new Counter(Config.LiftEncoderPort);

		DTEncFR = new Counter(Config.DrivetrainEncoderFrontRight);
		DTEncFL = new Counter(Config.DrivetrainEncoderFrontLeft);
		DTEncRR = new Counter(Config.DrivetrainEncoderRearRight);
		DTEncRL = new Counter(Config.DrivetrainEncoderRearLeft);

		autotime = new Timer();

		ultra1 = new Ultrasonic(8,9);
		
		liftEncoderDistance = 0;
	}

	boolean teleOpCalled = false;

	@Override
	public void teleopInit() {
		driveTrain = new AlphaMDrive(frontLeft, rearLeft, frontRight, rearRight, ControlMode.PercentOutput);
		driveTrain.setDeadband(DeadBandEntry.getDouble(.1));

		teleOpCalled = true;

		liftEncoderDistance = 0;
	}

	@Override
	public void teleopPeriodic() {
		driveTrain.driveCartesianMichael(joystick.getY(), joystick.getX(), joystick.getZ(), 90, joystick.getThrottle(),
				joystick.getTrigger());
		
		boxLift(xbox.getYButton(), xbox.getAButton(), xbox.getBButton(), xbox.getXButton());
		boxLiftSpin(xbox.getStartButton(), xbox.getBackButton());
		System.out.println("Current: " + liftSpinMotor.getOutputCurrent());
		
		updateFromDashboard(joystick.getRawButton(11));
	}	
	
	@Override
	public void autonomousInit() {
		teleOpCalled = false;
		
		liftEncoderDistance = 0;

		autotime.reset();
		autotime.start();

		// String gameData = DriverStation.getInstance().getGameSpecificMessage();
		// autonomousPathfinding(gameData, switches.getSwitches());

		// TEST CODE
		DriveY(.5, 4);
		DriveReset(0.1);
		DriveX(.5, 7);
		DriveReset(0.1);
		DriveY(.5, 5);
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
		// 3 - straight
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
			fieldPosition = null;
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
				System.out.println("Go straight"); // Put left auto code here for straight
			} else if (scalePriority) {
				if (gameData.charAt(1) == 'L') {
					// Put left auto code here with scale priority
				} else if (gameData.charAt(1) == 'R') {
					// Put left auto code here for straight
				} else {
					System.err.println("Couldn't determine gameData.charAt(1)");
				}
			} else if (switchPriority) {
				if (gameData.charAt(0) == 'L') {
					// Put left auto code here with switch priority
				} else if (gameData.charAt(0) == 'R') {
					DriveX(.5, 10);
					// Put left auto code here for straight
				} else {
					System.err.println("Couldn't determine gameData.charAt(0)");
				}
			}

		} else if (fieldPosition == Position.Middle) {

			//TODO: Grabber and Slide
			
			if (gameData.charAt(0) == 'L') {
				DriveY(.5, 4);
				DriveReset(0.1);
				DriveX(-.5, 7);
				DriveReset(0.1);
				DriveY(.5, 5);
			} else if (gameData.charAt(0) == 'R') {
				DriveY(.5, 4);
				DriveReset(0.1);
				DriveX(.5, 7);
				DriveReset(0.1);
				DriveY(.5, 5);
			} else {
				System.err.println("Couldn't determine gameData.charAt(0)");
			}

		} else if (fieldPosition == Position.Right) {

			if (autonomousStraight) {
				System.out.println("Go straight"); // Go straight, through the middle, and stop
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
					DriveX(0.5, 10);
				} else if (gameData.charAt(0) == 'R') {
					// Put right auto code here with switch priority
				} else {
					System.err.println("Couldn't determine gameData.charAt(0)");
				}
			}

		} else {
			System.out.println("Go straight");
		}

		// TODO: It doesn't help us to do nothing when the switches are set incorrectly
		// so do the bare minimum in case of human error by the driveteam (go straight)

		// 5-4-5 feet to cross base line in middle positions
		// 7 feet to go straight
	}

	int previousBoxLiftEncoderValue;

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
		int deltaBoxLiftEncoderValue = Math.abs(boxLiftEncoder.get() - previousBoxLiftEncoderValue);

		if (goDown) liftEncoderDistance = liftEncoderDistance - deltaBoxLiftEncoderValue;
		else if (goUp) liftEncoderDistance = liftEncoderDistance + deltaBoxLiftEncoderValue;
		else liftEncoderDistance = liftEncoderDistance - deltaBoxLiftEncoderValue;

		if (goUp) {
			boxLiftMotor.set(ControlMode.PercentOutput, 1);
		} else if (goDown) {
			boxLiftMotor.set(ControlMode.PercentOutput, -.5);
		} else {
			boxLiftMotor.set(ControlMode.PercentOutput, 0);
		}

		if (open) closingBoxLiftMotor.set(ControlMode.PercentOutput, .5);
		else if (close) closingBoxLiftMotor.set(ControlMode.PercentOutput, -.5);
		else closingBoxLiftMotor.set(ControlMode.PercentOutput, 0);

		previousBoxLiftEncoderValue = boxLiftEncoder.get();
	}
	
	public void boxLiftSpin(boolean wind, boolean unwind) {
		if (wind) {
			liftSpinMotor.set(ControlMode.PercentOutput, 0.2);
		} else if (unwind) {
			liftSpinMotor.set(ControlMode.PercentOutput, -1);
		} else {
			liftSpinMotor.set(ControlMode.PercentOutput, 0);
		}
	}

	public void robotForkLift(boolean goUp, boolean goDown) {
		if (goUp) forkLiftMotor.set(ControlMode.PercentOutput, 1);
		else if (goDown) forkLiftMotor.set(ControlMode.PercentOutput, -1);
		else forkLiftMotor.set(ControlMode.PercentOutput, 0);
	}

	public void updateFromDashboard(boolean update) { // TODO
		if (!update) return;
		driveTrain.setDeadband(DeadBandEntry.getDouble(.1));
	}

	@Override
	public void testInit() {
		DTEncFL.reset();
		DTEncFR.reset();
		DTEncRL.reset();
		DTEncRR.reset();
		ultra1.setAutomaticMode(true);

	}

	@Override
	public void testPeriodic() {
//		driveTrain.driveCartesianMichael(joystick.getY(), joystick.getX(), joystick.getZ(), -90, joystick.getThrottle(),
//				joystick.getTrigger());
//
//		System.out.println("FRONTLEFT: " + DTEncFL.get() + ", FRONTRIGHT: " + DTEncFR.get() + ", REARLEFT: "
//				+ DTEncRL.get() + ", REARRIGHT: " + DTEncRR.get());
//		System.out.println();
//		for (boolean c : switches.getSwitches()) {
//			System.out.print(c + " ");
//		}
//		System.out.println();
		System.out.println(ultra1.getRangeInches());  
		
	}

	public void DriveX(double speed, double feet) {
		DTEncFL.reset();
		DTEncFR.reset();
		DTEncRL.reset();
		DTEncRR.reset();

		Timer driveTimer = new Timer();
		driveTimer.reset();
		driveTimer.start();

		while (Math.abs(DTEncFL.get()) < Math.abs(feet * Config.encUnit)
				&& Math.abs(DTEncFR.get()) < Math.abs(feet * Config.encUnit)
				&& Math.abs(DTEncRR.get()) < Math.abs(feet * Config.encUnit)
				&& Math.abs(DTEncRL.get()) < Math.abs(feet * Config.encUnit) && !teleOpCalled && autotime.get() < 15) {

			if (driveTimer.get() < 0.15) {
				driveLinearX(speed * driveTimer.get() * (1.0 / 0.15));
				System.out.println("Speed: " + (speed * driveTimer.get() * (1.0 / 0.15)));
			} else {
				driveLinearX(speed);
				System.out.println("Speed: " + speed);
			}
		}
		
		driveLinearX(0);

		driveTimer.stop();
		driveTimer.reset();

	}

	public void DriveY(double speed, double feet) {
		DTEncFL.reset();
		DTEncFR.reset();
		DTEncRL.reset();
		DTEncRR.reset();

		Timer driveTimer = new Timer();
		driveTimer.reset();
		driveTimer.start();

		while (Math.abs(DTEncFL.get()) < Math.abs(feet * Config.encUnit)
				&& Math.abs(DTEncFR.get()) < Math.abs(feet * Config.encUnit)
				&& Math.abs(DTEncRR.get()) < Math.abs(feet * Config.encUnit)
				&& Math.abs(DTEncRL.get()) < Math.abs(feet * Config.encUnit) && !teleOpCalled && autotime.get() < 15) {

			if (driveTimer.get() < 0.15) {
				driveLinearY(speed * driveTimer.get() * (1.0 / 0.15));
				System.out.println("Speed: " + (speed * driveTimer.get() * (1.0 / 0.15)));
			} else {
				driveLinearY(speed);
				System.out.println("Speed: " + speed);
			}
		}
		
		driveLinearY(0);

		driveTimer.stop();
		driveTimer.reset();
	}

	public void DriveRotational(double speed, double degrees) {
		DTEncFL.reset();
		DTEncFR.reset();
		DTEncRL.reset();
		DTEncRR.reset();

		Timer driveTimer = new Timer();
		driveTimer.reset();
		driveTimer.start();

		while (Math.abs(DTEncFL.get()) < Math.abs(degrees * Config.degUnit)
				&& Math.abs(DTEncFR.get()) < Math.abs(degrees * Config.degUnit)
				&& Math.abs(DTEncRR.get()) < Math.abs(degrees * Config.degUnit)
				&& Math.abs(DTEncRL.get()) < Math.abs(degrees * Config.degUnit) && !teleOpCalled
				&& autotime.get() < 15) {

			if (driveTimer.get() < 0.15) {
				driveRotational(speed * driveTimer.get() * (1.0 / 0.15));
			} else {
				driveRotational(speed);
			}
		}
		
		driveRotational(0);

		driveTimer.stop();
		driveTimer.reset();
	}
	
	public void DriveReset(double time) {
		Timer driveTimer = new Timer();
		driveTimer.reset();
		driveTimer.start();
		
		while (driveTimer.get() < time) driveLinearX(0);
			
		driveTimer.stop();
		driveTimer.reset();
	}

	public void driveLinearY(double speed) {
		ControlMode cm = ControlMode.PercentOutput;

		double[] wheelSpeeds = new double[4];
		wheelSpeeds[MotorType.kFrontLeft.value] = speed;
		wheelSpeeds[MotorType.kFrontRight.value] = -speed;
		wheelSpeeds[MotorType.kRearLeft.value] = speed;
		wheelSpeeds[MotorType.kRearRight.value] = -speed;

		normalize(wheelSpeeds);
		frontLeft.set(cm, wheelSpeeds[MotorType.kFrontLeft.value]);
		frontRight.set(cm, wheelSpeeds[MotorType.kFrontRight.value]);
		rearLeft.set(cm, wheelSpeeds[MotorType.kRearLeft.value]);
		rearRight.set(cm, wheelSpeeds[MotorType.kRearRight.value]);
	}

	public void driveLinearX(double speed) {
		ControlMode cm = ControlMode.PercentOutput;

		double[] wheelSpeeds = new double[4];
		wheelSpeeds[MotorType.kFrontLeft.value] = speed;
		wheelSpeeds[MotorType.kFrontRight.value] = speed;
		wheelSpeeds[MotorType.kRearLeft.value] = -speed;
		wheelSpeeds[MotorType.kRearRight.value] = -speed;

		normalize(wheelSpeeds);
		frontLeft.set(cm, wheelSpeeds[MotorType.kFrontLeft.value]);
		frontRight.set(cm, wheelSpeeds[MotorType.kFrontRight.value]);
		rearLeft.set(cm, wheelSpeeds[MotorType.kRearLeft.value]);
		rearRight.set(cm, wheelSpeeds[MotorType.kRearRight.value]);
	}

	public void driveRotational(double speed) {
		ControlMode cm = ControlMode.PercentOutput;

		double[] wheelSpeeds = new double[4];
		wheelSpeeds[MotorType.kFrontLeft.value] = speed;
		wheelSpeeds[MotorType.kFrontRight.value] = speed;
		wheelSpeeds[MotorType.kRearLeft.value] = speed;
		wheelSpeeds[MotorType.kRearRight.value] = speed;

		normalize(wheelSpeeds);
		frontLeft.set(cm, wheelSpeeds[MotorType.kFrontLeft.value]);
		frontRight.set(cm, wheelSpeeds[MotorType.kFrontRight.value]);
		rearLeft.set(cm, wheelSpeeds[MotorType.kRearLeft.value]);
		rearRight.set(cm, wheelSpeeds[MotorType.kRearRight.value]);
	}

	protected void normalize(double[] wheelSpeeds) {
		double maxMagnitude = Math.abs(wheelSpeeds[0]);
		for (int i = 1; i < wheelSpeeds.length; i++) {
			double temp = Math.abs(wheelSpeeds[i]);
			if (maxMagnitude < temp) {
				maxMagnitude = temp;
			}
		}
		if (maxMagnitude > 1.0) {
			for (int i = 0; i < wheelSpeeds.length; i++) {
				wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
			}
		}
	}
}
