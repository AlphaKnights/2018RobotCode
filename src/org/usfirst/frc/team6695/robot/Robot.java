package org.usfirst.frc.team6695.robot;

import java.util.Arrays;

import org.usfirst.frc.team6695.robot.DrivingData.DrivingDataType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
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
	/** Liniar Slide Mid Hight Limit Switch **/
	DigitalInput liftMidLimit;
	/** Grabber Outer Limit Switch **/
	DigitalInput boxGrabLimit;

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
	/** Autonomous Multithread Timer 1 **/
	Timer temp;
	/** Autonomous Mutlithread Timer 2 **/
	Timer temp2;
	// /** Autonomous Error LED **/
	// Relay errorLED;

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

		boxLiftMotor.configContinuousCurrentLimit(20, 500);
		closingBoxLiftMotor.configContinuousCurrentLimit(7, 500);
		liftSpinMotor.configContinuousCurrentLimit(15, 500);

		boxLiftLimit = new DigitalInput(Config.LiftHiLimitPort);
		liftMidLimit = new DigitalInput(Config.LiftMidLimitPort);
		liftLowLimit = new DigitalInput(Config.LiftLoLimitPort);
		boxGrabLimit = new DigitalInput(Config.GrabHiLimitPort);

		boxLiftEncoder = new Counter(Config.LiftEncoderPort);
		// ultra1 = new Ultrasonic(8, 9);

		/** Forklift Subsystem Setup **/
		forkLiftMotor = new TalonSRX(Config.ForkLiftMotor);
		rightServo = new Servo(0);
		leftServo = new Servo(1);

		/** Autonomous Subsystem Setup **/
		switches = new ModeSelector(Config.AutoSwitchLeftPosPort, Config.AutoSwitchMiddlePosPort,
				Config.AutoSwitchRightPosPort, Config.AutoSwitchStraightPort, Config.AutoSwitchScalePort,
				Config.AutoSwitchSwitchPort, Config.AutoSwitchDelay2sPort, Config.AutoSwitchDelay5sPort);

		autotime = new Timer();
		temp = new Timer();
		temp2 = new Timer();

		gyroscope = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
		gyroscope.calibrate();

		// errorLED = new Relay(0, Direction.kForward);

		/** Dashboard Tasks **/
		tableSetup();
		CameraServer.getInstance().startAutomaticCapture();
		CameraServer.getInstance().startAutomaticCapture();

	}

	@Override
	public void robotPeriodic() {
		// if (switches.hasError()) errorLED.set(Relay.Value.kOn);
		// else errorLED.set(Relay.Value.kOff);

		// errorLED.set(Relay.Value.kOn);
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

		// /** Joystick-Controlled Forklift **/
		// forklift(xbox.getBumper(Hand.kRight), xbox.getBumper(Hand.kLeft));
		// /** Joystick-Controlled Forklift Deploy **/
		// forkliftDeploy(joystick.getRawButton(2), joystick.getRawButton(9));

		/** Update Deadzone **/
		updateFromDashboard(joystick.getRawButton(11));

		System.out.println("Snowblower Current: " + liftSpinMotor.getOutputCurrent());
		// SmartDashboard.putNumber("Slider Height in. ", ultra1.getRangeInches());
		// SmartDashboard.getNumber("Limit Height", sliderLimitHeight);

		// if (ultra1.getRangeInches() > sliderLimitHeight) {
		// SmartDashboard.putBoolean("Max Slider Height ", false);
		// } else {
		// SmartDashboard.putBoolean("Max Slider Height ", true);
		// }
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

		/** Switch and Scale Alliance Positions **/
		String gameData = "";
        while (gameData.equals("") && autotime.get() < 3.0) {
            gameData = DriverStation.getInstance().getGameSpecificMessage();
        }
        
        System.out.println(gameData);
        
		/** Execute Desired Autonomous Pathway **/
		autonomousPathfinding(gameData, switches.getSwitches());

		// DrivingData driveData = new DrivingData(DrivingDataType.MiddleR);
		
		// int timeIndex = 0;
		// autotime.reset();
		// autotime.start();
		
		// while (!teleOpCalled) {
		// 	if (autotime.get() >= driveData.driveDataArray[timeIndex][0]) {
		// 		frontLeft.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][1]);
		// 		frontRight.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][2]);
		// 		rearLeft.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][3]);
		// 		rearRight.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][4]);
		// 		boxLiftMotor.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][5]);
		// 		closingBoxLiftMotor.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][6]);
		// 		liftSpinMotor.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][7]);
				
		// 		System.out.println("motor set at " + autotime.get());
		// 		System.out.println(timeIndex++);
		// 		if (timeIndex == driveData.driveDataArray.length) teleOpCalled = true;
		// 	}
		// 	if (autotime.get() >= 15.0) teleOpCalled = true;
		// }
		
		// TEST CODE
		// gyroscope.reset();
		//
		// new Thread(() -> {
		// temp.reset();
		// temp.start();
		//
		// while (temp.get() < 6 && !Thread.interrupted())
		// boxLift(true, false);
		// boxLift(false, false);
		//
		// temp.stop();
		// temp.reset();
		// }).start();
		//
		// // new Thread(() -> {
		// // temp2.reset();
		// // temp2.start();
		// //
		// // while (temp2.get() < 1 && !Thread.interrupted())
		// // boxSpin(true, false);
		// // boxSpin(false, false);
		// //
		// // temp2.stop();
		// // temp2.reset();
		// // }).start();
		//
		// DriveY(.5, 24 * 24 / 14.75);
		// DriveReset(1);
		// DriveToAngle(-0.2, 270);
		// DriveReset(0.8);
		//
		// temp.reset();
		// temp.start();
		//
		// while (temp.get() < 0.5)
		// boxGrab(true, false);
		// boxGrab(false, false);
		//
		// temp.stop();
		// temp.reset();
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
		
		input = switches.getSwitches();

		if (input[0]) {
			fieldPosition = Position.Left;
		} else if (input[1]) {
			fieldPosition = Position.Middle;
			System.out.println("Middle Autonomous");
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

            DriveY(0.5, 10);

			// if (autonomousStraight) {
			// 	System.out.println("Go straight, middle, and stop");
			// 	// Put left auto code here for straight
			// } else if (scalePriority) {
			// 	if (gameData.charAt(1) == 'L') {
			// 		// Put left auto code here with scale priority
			// 	} else if (gameData.charAt(1) == 'R') {
			// 		// Put left auto code here for straight
			// 	} else {
			// 		System.err.println("Couldn't determine gameData.charAt(1)");
			// 	}
			// } else if (switchPriority) {
			// 	if (gameData.charAt(0) == 'L') {
			// 		// Put left auto code here with switch priority
			// 	} else if (gameData.charAt(0) == 'R') {
			// 		DriveX(.5, 10);
			// 		// Put left auto code here for straight
			// 	} else {
			// 		System.err.println("Couldn't determine gameData.charAt(0)");
			// 	}
			// }

		} else if (fieldPosition == Position.Middle) {
			// TODO: Grabber and Slide
			if (gameData.charAt(0) == 'L' && (autonomousStraight || scalePriority || switchPriority)) {
				DrivingData driveData = new DrivingData(DrivingDataType.MiddleL);
		
                int timeIndex = 0;
                while (!teleOpCalled) {
                    if (autotime.get() >= driveData.driveDataArray[timeIndex][0]) {
                        frontLeft.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][1]);
                        frontRight.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][2]);
                        rearLeft.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][3]);
                        rearRight.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][4]);

                        if (driveData.driveDataArray[timeIndex][5] > 0 && (boxLiftLimit.get() || liftMidLimit.get())) {
                            boxLiftMotor.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][5]);
                        } else if (driveData.driveDataArray[timeIndex][5] < 0 && liftLowLimit.get()) {
                            boxLiftMotor.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][5]);
                        } else {
                            boxLiftMotor.set(ControlMode.PercentOutput, 0);
                        }

                        if (driveData.driveDataArray[timeIndex][6] < 0 && boxGrabLimit.get()) {
                            closingBoxLiftMotor.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][6]);
                        } else if (driveData.driveDataArray[timeIndex][6] < 0) {
                            closingBoxLiftMotor.set(ControlMode.PercentOutput, 0);
                        } else {
                            closingBoxLiftMotor.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][6]);
                        }
                        
//                        liftSpinMotor.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][7]);
                        
//                        if (autotime.get() < 1) {
//                        	boxLiftMotor.set(ControlMode.PercentOutput, 0.5);
//                        }
                        
                        System.out.println("motor set at " + autotime.get());
                        System.out.println(timeIndex++);
                        if (timeIndex == driveData.driveDataArray.length) teleOpCalled = true;
                    }
                    if (autotime.get() >= 15.0) teleOpCalled = true;
                }
			} else if (gameData.charAt(0) == 'R' && (autonomousStraight || scalePriority || switchPriority)) {
				DrivingData driveData = new DrivingData(DrivingDataType.MiddleR);
		
                int timeIndex = 0;
                while (!teleOpCalled) {
                    if (autotime.get() >= driveData.driveDataArray[timeIndex][0]) {
                        frontLeft.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][1]);
                        frontRight.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][2]);
                        rearLeft.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][3]);
                        rearRight.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][4]);

                        if (driveData.driveDataArray[timeIndex][5] > 0 && (boxLiftLimit.get() || liftMidLimit.get())) {
                            boxLiftMotor.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][5]);
                        } else if (driveData.driveDataArray[timeIndex][5] < 0 && liftLowLimit.get()) {
                            boxLiftMotor.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][5]);
                        } else {
                            boxLiftMotor.set(ControlMode.PercentOutput, 0);
                        }

                        if (driveData.driveDataArray[timeIndex][6] < 0 && boxGrabLimit.get()) {
                            closingBoxLiftMotor.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][6]);
                        } else if (driveData.driveDataArray[timeIndex][6] < 0) {
                            closingBoxLiftMotor.set(ControlMode.PercentOutput, 0);
                        } else {
                            closingBoxLiftMotor.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][6]);
                        }
                        
//                        liftSpinMotor.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][7]);
                        
//                        if (autotime.get() < 1) {
//                        	boxLiftMotor.set(ControlMode.PercentOutput, 0.5);
//                        }
                        
                        System.out.println("motor set at " + autotime.get());
                        System.out.println(timeIndex++);
                        if (timeIndex == driveData.driveDataArray.length) teleOpCalled = true;
                    }
                    if (autotime.get() >= 15.0) teleOpCalled = true;
                }
			} else {
				System.err.println("Couldn't determine gameData.charAt(0)");
			}

		} else if (fieldPosition == Position.Right) {
            DriveY(0.5, 10);
		}
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
		if (goUp) {
			if (boxLiftLimit.get() || liftMidLimit.get()) boxLiftMotor.set(ControlMode.PercentOutput, 1);
		} else if (goDown && liftLowLimit.get()) boxLiftMotor.set(ControlMode.PercentOutput, -.5);
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
		if (open) closingBoxLiftMotor.set(ControlMode.PercentOutput, .6);
		else if (close && boxGrabLimit.get()) closingBoxLiftMotor.set(ControlMode.PercentOutput, -.6);
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
		if (wind) liftSpinMotor.set(ControlMode.PercentOutput, 0.5);
		else if (unwind) liftSpinMotor.set(ControlMode.PercentOutput, -0.5);
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
	@Deprecated
	public void forklift(boolean goUp, boolean goDown) {
//		if (goUp) forkLiftMotor.set(ControlMode.PercentOutput, 1);
//		else if (goDown) forkLiftMotor.set(ControlMode.PercentOutput, -1);
//		else forkLiftMotor.set(ControlMode.PercentOutput, 0);
	}

	/**
	 * This is the Forklift deploy code
	 * 
	 * @param deploy
	 *            Button to deploy the forks
	 */
	@Deprecated
	public void forkliftDeploy(boolean deploy, boolean areYouSure) {
//		if (deploy && areYouSure) rightServo.set(0.5);
	}

	// TODO: Test & Document updateFromDashboard()
	public void updateFromDashboard(boolean update) {
		if (!update) return;
		driveTrain.setDeadband(DeadBandEntry.getDouble(.1));
	}

	public void DriveToAngle(double speed, double angle) {
		Timer driveTimer = new Timer();
		driveTimer.reset();
		driveTimer.start();

		if (speed < 0) {
			while (gyroscope.getAngle() % 360 > angle && !teleOpCalled && autotime.get() < 15) {
				if (driveTimer.get() < 0.15) driveRotational(speed * driveTimer.get() * (1.0 / 0.15));
				else driveRotational(speed);

				System.out.println("Gyro Normalized Angle (1): " + gyroscope.getAngle() % 360);
			}
		} else {
			while (gyroscope.getAngle() % 360 < angle && !teleOpCalled && autotime.get() < 15) {
				if (driveTimer.get() < 0.15) driveRotational(speed * driveTimer.get() * (1.0 / 0.15));
				else driveRotational(speed);

				System.out.println("Gyro Normalized Angle (2): " + gyroscope.getAngle() % 360);
			}
		}

		driveRotational(0);

		driveTimer.stop();
		driveTimer.reset();
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
		autotime.reset();
		autotime.start();
//		
//		DrivingData driveData = new DrivingData(DrivingDataType.TroyMiddle);
//		
//		int timeIndex = 0;
//		autotime.reset();
//		autotime.start();
//		
//		while (!teleOpCalled) {
//			if (autotime.get() >= driveData.driveDataArray[timeIndex][0]) {
//				frontLeft.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][1]);
//				frontRight.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][2]);
//				rearLeft.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][3]);
//				rearRight.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][4]);
//				boxLiftMotor.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][5]);
//				closingBoxLiftMotor.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][6]);
//				liftSpinMotor.set(ControlMode.PercentOutput, driveData.driveDataArray[timeIndex][7]);
//				
//				System.out.println("motor set at " + autotime.get());
//				System.out.println(timeIndex++);
//				if (timeIndex == driveData.driveDataArray.length) teleOpCalled = true;
//			}
//			if (autotime.get() >= 15.0) teleOpCalled = true;
//		}
	}

	/**
	 * Periodic code for test mode is here.
	 */
	@Override
	public void testPeriodic() {
		driveTrain = new AlphaMDrive(frontLeft, rearLeft, frontRight, rearRight, ControlMode.PercentOutput);
		driveTrain.setDeadband(DeadBandEntry.getDouble(.1));
		driveTrain.driveCartesianMichael(joystick.getY(), joystick.getX(), joystick.getZ(), 90, joystick.getThrottle(),
				joystick.getTrigger());

		/** Xbox-Controlled Linear Sliding **/
		boxLift(xbox.getYButton(), xbox.getAButton());
		/** Xbox-Controlled Power Cube Grabbing **/
		boxGrab(xbox.getBButton(), xbox.getXButton());
		/** Xbox-Controlled Lift Spinning **/
		boxSpin(xbox.getStartButton(), xbox.getBackButton());

		double[] autoRecord = { autotime.get(), frontLeft.getMotorOutputPercent(), frontRight.getMotorOutputPercent(),
				rearLeft.getMotorOutputPercent(), rearRight.getMotorOutputPercent(),
				boxLiftMotor.getMotorOutputPercent(), closingBoxLiftMotor.getMotorOutputPercent(),
				liftSpinMotor.getMotorOutputPercent() };
		
		// Time, Drivetrain FL, FR, RL, RR, Lift, Grabber, Spinner
		System.out.println(Arrays.toString(autoRecord));
		
//		System.out.println(Arrays.toString(switches.getSwitches()));
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
			} else {
				driveLinearX(speed);
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
			} else {
				driveLinearY(speed);
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
