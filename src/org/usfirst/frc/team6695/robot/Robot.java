package org.usfirst.frc.team6695.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogOutput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;

public class Robot extends IterativeRobot {
	/* Controllers */

	/** Joystick **/
	Joystick joystick;
	/** Xbox Controller **/
	XboxController xbox;

	/* Power Cube Subsystem */

	/** Linear Slide Motor **/
	TalonSRX boxLiftMotor;
	/** Grabber Motor **/
	TalonSRX closingBoxLiftMotor;
	/** Snowmobile Spin Motor **/
	TalonSRX liftSpinMotor;

	/** Linear Slide Encoder **/
	Counter boxLiftEncoder;
	
	/** Linear Slide Max Height Limit Switch **/
	DigitalInput boxLiftLimit;
	/** Linear Slide Min Height Limit Switch **/
	DigitalInput liftLowLimit;
	/** Grabber Outer Limit Switch **/
	DigitalInput boxGrabLimit;
	
	/** Linear Slide Max Height Limit Switch Default **/
	boolean boxLiftLimitDefault;
	/** Linear Slide Min Height Limit Switch Default **/
	boolean liftLowLimitDefault;
	/** Grabber Outer Limit Switch Default **/
	boolean boxGrabLimitDefault;

	/** Linear Slide Displacement Sensor **/
	Ultrasonic ultra1;

	/* Forklift Subsystem */

	/** Forklift Motor **/
	TalonSRX forkLiftMotor;

	/* Drivetrain Subsystem */

	/** Drivetrain **/
	AlphaMDrive driveTrain;

	/** Front Right Drivetrain Motor **/
	TalonSRX frontRight;
	/** Front Left Drivetrain Motor **/
	TalonSRX frontLeft;
	/** Rear Left Drivetrain Motor **/
	TalonSRX rearLeft;
	/** Rear Right Drivetrain Motor **/
	TalonSRX rearRight;

	/** Front Right Drivetrain Encoder **/
	Counter DTEncFR;
	/** Front Left Drivetrain Encoder **/
	Counter DTEncFL;
	/** Rear Left Drivetrain Encoder **/
	Counter DTEncRL;
	/** Rear Right Drivetrain Encoder **/
	Counter DTEncRR;

	/** Controller Deadzone (Configurable in Dashboard) **/
	NetworkTableEntry DeadBandEntry;

	/* Autonomous Subsystem */

	/** Possible Positions **/
	public enum Position {
		Left,
		Middle,
		Right;
	}

	/** Switch Data **/
	ModeSelector switches;
	/** Gyroscope **/
	ADXRS450_Gyro gyroscope;
	/** Position Data **/
	Position fieldPosition;
	/** Autonomous Time Elapsed **/
	Timer autotime;
	/** Autonomous Error LED **/
	Relay errorLED;

	/* Servos */
	
	/** Right Side Servo **/
	Servo rightServo;
	/** Left Side Servo **/
	Servo leftServo;

	/** Autonomous Finished **/
	boolean teleOpCalled;
	/** Autonomous Started **/
	boolean autoInitCalled = false;
	/** Scale Priority **/
	boolean scalePriority = false;
	/** Switch Priority **/
	boolean switchPriority = false;
	/** Autoline Priority **/
	boolean autonomousStraight = false;
	
	/* ---------------------------------------- */

	/**
	 * Setup configurable controller deadzone
	 */
	public void tableSetup() {
		NetworkTableInstance nts = NetworkTableInstance.getDefault();
		NetworkTable table = nts.getTable("SmartDashboard");
		DeadBandEntry = table.getEntry("deadband");
		DeadBandEntry.setDouble(.1);
	}

	/**
	 * Robot-wide initialization code is here.
	 *
	 * <p>
	 * Users should override this method for default Robot-wide initialization which will be called when
	 * the robot is first powered on. It will be called exactly one time.
	 *
	 * <p>
	 * Warning: the Driver Station "Robot Code" light and FMS "Robot Ready" indicators will be off until
	 * RobotInit() exits. Code in RobotInit() that waits for enable will cause the robot to never
	 * indicate that the code is ready, causing the robot to be bypassed in a match.
	 */
	@Override
	public void robotInit() {
		/** Controller Setup **/
		joystick = new Joystick(Config.JoystickChannel);
		xbox = new XboxController(Config.XBoxChannel);

		/** Drivetrain Subsystem Setup **/
		frontLeft = new TalonSRX(Config.DriveTrainFrontLeft);
		rearLeft = new TalonSRX(Config.DriveTrainRearLeft);
		frontRight = new TalonSRX(Config.DriveTrainFrontRight);
		rearRight = new TalonSRX(Config.DriveTrainRearRight);

		DTEncFR = new Counter(Config.DrivetrainEncoderFrontRight);
		DTEncFL = new Counter(Config.DrivetrainEncoderFrontLeft);
		DTEncRR = new Counter(Config.DrivetrainEncoderRearRight);
		DTEncRL = new Counter(Config.DrivetrainEncoderRearLeft);

		/** Power Cube Subsystem Setup **/
		boxLiftMotor = new TalonSRX(Config.LiftMotor);
		closingBoxLiftMotor = new TalonSRX(Config.liftGrabberMotor);
		liftSpinMotor = new TalonSRX(Config.LiftSpinMotor);

		boxLiftMotor.enableCurrentLimit(true);
		closingBoxLiftMotor.enableCurrentLimit(true);
		liftSpinMotor.enableCurrentLimit(true);

		boxLiftMotor.configContinuousCurrentLimit(10, 500);
		closingBoxLiftMotor.configContinuousCurrentLimit(5, 500);
		liftSpinMotor.configContinuousCurrentLimit(36, 500);
		
		boxLiftLimit = new DigitalInput(Config.LiftHiLimitPort);
		liftLowLimit = new DigitalInput(Config.LiftLoLimitPort);
		boxGrabLimit = new DigitalInput(Config.GrabHiLimitPort);
		
		boxLiftLimitDefault = boxLiftLimit.get();
		liftLowLimitDefault = liftLowLimit.get();
		boxGrabLimitDefault = boxGrabLimit.get();

		boxLiftEncoder = new Counter(Config.LiftEncoderPort);
//		ultra1 = new Ultrasonic(8, 9);

		/** Forklift Subsystem Setup **/
		forkLiftMotor = new TalonSRX(Config.ForkLiftMotor);
		rightServo = new Servo(0);
		leftServo = new Servo(1);

		/** Autonomous Subsystem Setup **/
		switches = new ModeSelector(18, 11, 12, 13, 14, 15, 16, 17);
		autotime = new Timer();

		gyroscope = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
		gyroscope.calibrate();
		
//		errorLED = new Relay(0, Direction.kForward);

		/** Dashboard Tasks **/
		tableSetup();
		CameraServer.getInstance().startAutomaticCapture();
		CameraServer.getInstance().startAutomaticCapture();

	}

	@Override
	public void robotPeriodic() {
//		if (switches.hasError()) errorLED.set(Relay.Value.kOn);
//		else errorLED.set(Relay.Value.kOff);
		
//		errorLED.set(Relay.Value.kOn);
	}

	/**
	 * Initialization code for teleop mode is here.
	 *
	 * <p>
	 * Users should override this method for initialization code which will be called each time the
	 * robot enters teleop mode.
	 */
	@Override
	public void teleopInit() {
		/** Force End Autonomous **/
		teleOpCalled = true;

		/** Setup Drivetrain **/
		driveTrain = new AlphaMDrive(frontLeft, rearLeft, frontRight, rearRight, ControlMode.PercentOutput);

		/** Setup Deadzone **/
		driveTrain.setDeadband(DeadBandEntry.getDouble(.1));
	}

	/**
	 * Periodic code for teleop mode is here.
	 */
	@Override
	public void teleopPeriodic() {
		/** Joystick-Controlled Driving **/
		driveTrain.driveCartesianMichael(joystick.getY(), joystick.getX(), joystick.getZ(), 90, joystick.getThrottle(),
				joystick.getTrigger());

		/** Xbox-Controlled Linear Sliding **/
		boxLift(xbox.getYButton(), xbox.getAButton());
		/** Xbox-Controlled Power Cube Grabbing **/
		boxGrab(xbox.getBButton(), xbox.getXButton());
		/** Xbox-Controlled Lift Spinning **/
		boxSpin(xbox.getStartButton(), xbox.getBackButton());

//		/** Joystick-Controlled Forklift **/
//		forklift(xbox.getBumper(Hand.kRight), xbox.getBumper(Hand.kLeft));
//		/** Joystick-Controlled Forklift Deploy **/
//		forkliftDeploy(joystick.getRawButton(2), joystick.getRawButton(9));

		/** Update Deadzone **/
		updateFromDashboard(joystick.getRawButton(11));

		System.out.println("Lift Limit Reached: " + boxLiftLimit.get());
//		SmartDashboard.putNumber("Slider Height in. ", ultra1.getRangeInches());
//		SmartDashboard.getNumber("Limit Height", sliderLimitHeight);
		
//		if (ultra1.getRangeInches() > sliderLimitHeight) {
//			SmartDashboard.putBoolean("Max Slider Height ", false);
//		} else {
//			SmartDashboard.putBoolean("Max Slider Height ", true);
//		}
	}

	/**
	 * Initialization code for autonomous mode is here.
	 *
	 * <p>
	 * Users should override this method for initialization code which will be called each time the
	 * robot enters autonomous mode.
	 */
	@Override
	public void autonomousInit() {
		/** Autonomous Start **/
		teleOpCalled = false;
		autoInitCalled = true;

		/** Start Autonomous Time-Elapsed **/
		autotime.reset();
		autotime.start();

		Timer temp = new Timer();
		Timer temp2 = new Timer();

		/** Switch and Scale Alliance Positions **/
		// String gameData = DriverStation.getInstance().getGameSpecificMessage();
		/** Execute Desired Autonomous Pathway **/
		// autonomousPathfinding(gameData, switches.getSwitches());

		// TEST CODE
		new Thread(() -> {
			temp.reset();
			temp.start();

			while (temp.get() < 2 && !Thread.interrupted())
				boxLift(true, false);
			boxLift(false, false);

			temp.stop();
			temp.reset();
		}).start();
		
		new Thread(() -> {
			temp2.reset();
			temp2.start();

			while (temp2.get() < 1 && !Thread.interrupted())
				boxSpin(true, false);
			boxSpin(false, false);

			temp2.stop();
			temp2.reset();
		}).start();
		
		DriveY(.5, 4);
		DriveReset(0.1);
		DriveX(.5, 6.25);
		DriveReset(0.1);
		DriveY(.5, 7);
		DriveReset(0.1);
		DriveToAngle(0);
		DriveReset(1.5);

		temp.reset();
		temp.start();

		while (temp.get() < 0.5)
			boxGrab(true, false);
		boxGrab(false, false);

		temp.stop();
		temp.reset();
	}

	/**
	 * Pathfinding code for autonomous mode is here.
	 */
	public void autonomousPathfinding(String gameData, boolean[] input) {
		// 0 - left
		// 1 - middle
		// 2 - right
		// 3 - straight
		// 4 - target scale
		// 5 - target switch
		// 6 - delay 2s
		// 7 - delay 5s

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
			System.out.println("Target straight");
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

		if (fieldPosition == Position.Left) {

			if (autonomousStraight) {
				System.out.println("Go straight, middle, and stop");
				// Put left auto code here for straight
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

			// TODO: Grabber and Slide
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
				System.out.println("Go straight"); // Go straight, middle, and stop
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
	}

	/**
	 * This is the Power Cube Lift code
	 * 
	 * @param goUp
	 *            Button to raise the linear slide
	 * @param goDown
	 *            Button to lower the linear slide
	 */
	public void boxLift(boolean goUp, boolean goDown) {
		if (goUp && boxLiftLimit.get()) boxLiftMotor.set(ControlMode.PercentOutput, 1);
		else if (goDown && liftLowLimit.get()) boxLiftMotor.set(ControlMode.PercentOutput, -.5);
		else boxLiftMotor.set(ControlMode.PercentOutput, 0);
	}

	/**
	 * This is the Power Cube Grab code
	 * 
	 * @param close
	 *            Button to close the grabber
	 * @param open
	 *            Button to open the grabber
	 */
	public void boxGrab(boolean close, boolean open) {
		if (open) closingBoxLiftMotor.set(ControlMode.PercentOutput, .5);
		else if (close && boxGrabLimit.get()) closingBoxLiftMotor.set(ControlMode.PercentOutput, -.5);
		else closingBoxLiftMotor.set(ControlMode.PercentOutput, 0);
	}

	/**
	 * This is the Power Cube Spin code
	 * 
	 * @param wind
	 *            Button to wind the grabber
	 * @param unwind
	 *            Button to unwind the grabber
	 */
	public void boxSpin(boolean wind, boolean unwind) {
		if (wind) liftSpinMotor.set(ControlMode.PercentOutput, 0.2);
		else if (unwind) liftSpinMotor.set(ControlMode.PercentOutput, -1);
		else liftSpinMotor.set(ControlMode.PercentOutput, 0);
	}

	/**
	 * This is the Forklift code
	 * 
	 * @param goUp
	 *            Button to raise the forklift
	 * @param goDown
	 *            Button to lower the forklift
	 */
	public void forklift(boolean goUp, boolean goDown) {
		if (goUp) forkLiftMotor.set(ControlMode.PercentOutput, 1);
		else if (goDown) forkLiftMotor.set(ControlMode.PercentOutput, -1);
		else forkLiftMotor.set(ControlMode.PercentOutput, 0);
	}

	/**
	 * This is the Forklift deploy code
	 * 
	 * @param deploy
	 *            Button to deploy the forks
	 */
	public void forkliftDeploy(boolean deploy, boolean areYouSure) {
		if (deploy && areYouSure) rightServo.set(0.5);
	}

	// TODO: Test & Document updateFromDashboard()
	public void updateFromDashboard(boolean update) {
		if (!update) return;
		driveTrain.setDeadband(DeadBandEntry.getDouble(.1));
	}

	/**
	 * Initialization code for test mode is here.
	 *
	 * <p>
	 * Users should override this method for initialization code which will be called each time the
	 * robot enters test mode.
	 */
	@Override
	public void testInit() {
		// DTEncFL.reset();
		// DTEncFR.reset();
		// DTEncRL.reset();
		// DTEncRR.reset();
		//
		// ultra1.setAutomaticMode(true);
		leftServo.set(0);
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		rightServo.set(0.5);
	}

	/**
	 * Periodic code for test mode is here.
	 */
	@Override
	public void testPeriodic() {
		// driveTrain.driveCartesianMichael(joystick.getY(), joystick.getX(), joystick.getZ(), -90,
		// joystick.getThrottle(),
		// joystick.getTrigger());
		//
		// System.out.println("FRONTLEFT: " + DTEncFL.get() + ", FRONTRIGHT: " + DTEncFR.get() + ",
		// REARLEFT: "
		// + DTEncRL.get() + ", REARRIGHT: " + DTEncRR.get());
		// System.out.println();
		// for (boolean c : switches.getSwitches()) {
		// System.out.print(c + " ");
		// }
		// System.out.println();
		// System.out.println(ultra1.getRangeInches());
	}

	/**
	 * This is the autonomous x-axis driving code
	 * 
	 * @param speed
	 *            Percent output for all motors where negative is the opposite direction of motion
	 * @param feet
	 *            The number of feet to travel in the given direction (floating-point value)
	 */
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

	/**
	 * This is the autonomous y-axis driving code
	 * 
	 * @param speed
	 *            Percent output for all motors where negative is the opposite direction of motion
	 * @param feet
	 *            The number of feet to travel in the given direction (floating-point value)
	 */
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

	/**
	 * This is the autonomous rotational driving code
	 * 
	 * @param speed
	 *            Percent output for all motors where negative is the opposite direction of motion
	 * @param degrees
	 *            The number of degrees to spin in the given direction (floating-point value)
	 */
	public void DriveRotational(double speed, double degrees) {
		DTEncFL.reset();
		DTEncFR.reset();
		DTEncRL.reset();
		DTEncRR.reset();

		Timer driveTimer = new Timer();
		driveTimer.reset();
		driveTimer.start();

		double initialDegrees = gyroscope.getAngle();
		if (degrees < 0) {
			while (gyroscope.getAngle() > (initialDegrees + degrees) && !teleOpCalled && autotime.get() < 15) {
				if (driveTimer.get() < 0.15) driveRotational(speed * driveTimer.get() * (1.0 / 0.15));
				else driveRotational(speed);
			}
		} else {
			while (gyroscope.getAngle() < (initialDegrees + degrees) && !teleOpCalled && autotime.get() < 15) {
				if (driveTimer.get() < 0.15) driveRotational(speed * driveTimer.get() * (1.0 / 0.15));
				else driveRotational(speed);
			}
		}

		driveRotational(0);

		driveTimer.stop();
		driveTimer.reset();
	}

	/**
	 * This is the autonomous driving reset code (sets the motor output to 0)
	 * 
	 * @param time
	 *            Number of seconds to let the robot cool off
	 */
	public void DriveReset(double time) {
		Timer driveTimer = new Timer();
		driveTimer.reset();
		driveTimer.start();

		while (driveTimer.get() < time)
			driveLinearX(0);

		driveTimer.stop();
		driveTimer.reset();
	}

	/**
	 * This is the autonomous x-axis helper code
	 * 
	 * @param speed
	 *            Percent output for all motors where negative is the opposite direction of motion
	 */
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

	/**
	 * This is the autonomous y-axis helper code
	 * 
	 * @param speed
	 *            Percent output for all motors where negative is the opposite direction of motion
	 */
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

	/**
	 * This is the autonomous rotational helper code
	 * 
	 * @param speed
	 *            Percent output for all motors where negative is the opposite direction of motion
	 */
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

	/**
	 * This is the wheel speed normalization code for maintaining direction at varying speeds
	 * 
	 * @param wheelSpeeds
	 *            Set of motor output values for a given tick
	 */
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
