package org.usfirst.frc.team6695.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	public AlphaMDrive driveTrain;
	Joystick joystick;
	XboxController xbox;

	TalonSRX boxLiftLeftMotor;
	TalonSRX boxLiftRightMotor;
	TalonSRX closingBoxLiftMotor;

	TalonSRX forkLiftMotor;

	ModeSelector switches;
	Position fieldPosition;

	Counter boxLiftEncoder;

	Counter DTEncFR;
	Counter DTEncFL;
	Counter DTEncRL;
	Counter DTEncRR;

	private static final double Kp = 0.3;
	private static final double Ki = 0.0;
	private static final double Kd = 0.0;

	PIDController DTFRPID;
	PIDController DTFLPID;
	PIDController DTRLPID;
	PIDController DTRRPID;

	Timer autotime;

	public enum Position {
		Left,
		Middle,
		Right;
	}

	AHRS navx;

	WPI_TalonSRX frontLeft;
	WPI_TalonSRX rearLeft;
	WPI_TalonSRX frontRight;
	WPI_TalonSRX rearRight;

	NetworkTableEntry DeadBandEntry;

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

		frontLeft = new WPI_TalonSRX(Config.DriveTrainFrontLeft);
		rearLeft = new WPI_TalonSRX(Config.DriveTrainRearLeft);
		frontRight = new WPI_TalonSRX(Config.DriveTrainFrontRight);
		rearRight = new WPI_TalonSRX(Config.DriveTrainRearRight);

		boxLiftLeftMotor = new TalonSRX(Config.LiftLeftMotor);
		boxLiftRightMotor = new TalonSRX(Config.LiftRightMotor);
		closingBoxLiftMotor = new TalonSRX(Config.liftGrabberMotor);

		forkLiftMotor = new TalonSRX(Config.ForkLiftMotor);

		boxLiftLeftMotor.enableCurrentLimit(false);
		boxLiftRightMotor.enableCurrentLimit(false);
		closingBoxLiftMotor.enableCurrentLimit(true);

		boxLiftLeftMotor.configContinuousCurrentLimit(100, 500);
		boxLiftRightMotor.configContinuousCurrentLimit(100, 500);
		closingBoxLiftMotor.configContinuousCurrentLimit(2, 500);

		switches = new ModeSelector(10, 11, 12, 13, 14, 15, 16, 17);
		joystick = new Joystick(Config.JoystickChannel);
		xbox = new XboxController(Config.XBoxChannel);

		boxLiftEncoder = new Counter(Config.LiftEncoderPort);

		DTEncFR = new Counter(Config.DrivetrainEncoderFrontRight);
		DTEncFL = new Counter(Config.DrivetrainEncoderFrontLeft);
		DTEncRR = new Counter(Config.DrivetrainEncoderRearRight);
		DTEncRL = new Counter(Config.DrivetrainEncoderRearLeft);

		// kp, ki, kd, source, output
		DTFRPID = new PIDController(Kp, Ki, Kd, DTEncFR, frontRight);
		DTFLPID = new PIDController(Kp, Ki, Kd, DTEncFL, frontLeft);
		DTRRPID = new PIDController(Kp, Ki, Kd, DTEncRL, rearLeft);
		DTRLPID = new PIDController(Kp, Ki, Kd, DTEncRR, rearRight);
		
//		SmartDashboard.putData("Front Right Wheel Speed PID", DTFRPID);
//		SmartDashboard.putData("Front Left Wheel Speed PID", DTFLPID);
//		SmartDashboard.putData("Rear Right Wheel Speed PID", DTRRPID);
//		SmartDashboard.putData("Rear Left Wheel Speed PID", DTFRPID);

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
		driveTrain = new AlphaMDrive(frontLeft, rearLeft, frontRight, rearRight, ControlMode.PercentOutput);
		driveTrain.setDeadband(DeadBandEntry.getDouble(.1));
		DTFRPID.disable();
		DTFLPID.disable();
		DTRRPID.disable();
		DTRLPID.disable();
		teleOpCalled = true;
	}

	@Override
	public void teleopPeriodic() {
		driveTrain.driveCartesianMichael(joystick.getY(), joystick.getX(), joystick.getZ(), 90, joystick.getThrottle(),
				joystick.getTrigger());
		boxLift(xbox.getYButton(), xbox.getAButton(), xbox.getBButton(), xbox.getXButton());
		// System.out.println(c.get());
	}

	@Override
	public void autonomousInit() {
		frontLeft.setSafetyEnabled(false);
		frontRight.setSafetyEnabled(false);
		rearLeft.setSafetyEnabled(false);
		rearRight.setSafetyEnabled(false);
		
		autotime.reset();
		autotime.start();

		// String gameData = DriverStation.getInstance().getGameSpecificMessage();
		// autonomousPathfinding(gameData, switches.getSwitches());

		// TEST CODE
		DriveY(.5, 3);
		DriveX(.5, 4);
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

			if (autonomousStraight) {
				System.out.println("Cross base line"); // Will never actually cross base line in middle position
			} else if (scalePriority) {
				if (gameData.charAt(1) == 'L') {
					// Put middle auto code here with left scale priority
				} else if (gameData.charAt(1) == 'R') {
					// Put middle auto code here with right scale priority
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
		if (goDown) boxLiftEncoder.setReverseDirection(true);
		if (goUp) boxLiftEncoder.setReverseDirection(false);

		if (goUp) if (boxLiftEncoder.get() < Config.EncoderTopValue) {
			boxLiftLeftMotor.set(ControlMode.PercentOutput, 1);
			boxLiftRightMotor.set(ControlMode.PercentOutput, 1);
		} else if (goDown) if (boxLiftEncoder.get() < 10) {
			boxLiftLeftMotor.set(ControlMode.PercentOutput, -1);
			boxLiftRightMotor.set(ControlMode.PercentOutput, -1);
		} else {
			boxLiftLeftMotor.set(ControlMode.PercentOutput, 0);
			boxLiftRightMotor.set(ControlMode.PercentOutput, 0);
		}

		if (open) closingBoxLiftMotor.set(ControlMode.PercentOutput, 1);
		else if (close) closingBoxLiftMotor.set(ControlMode.PercentOutput, -1);
		else closingBoxLiftMotor.set(ControlMode.PercentOutput, 0);
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
	}

	@Override
	public void testPeriodic() {
		driveTrain.driveCartesianMichael(joystick.getY(), joystick.getX(), joystick.getZ(), -90, joystick.getThrottle(),
				joystick.getTrigger());

		System.out.println("FRONTLEFT: " + DTEncFL.get() + ", FRONTRIGHT: " + DTEncFR.get() + ", REARLEFT: "
				+ DTEncRL.get() + ", REARRIGHT: " + DTEncRR.get());
		System.out.println();
		for (boolean c : switches.getSwitches()) {
			System.out.print(c + " ");
		}
		System.out.println();
	}

	public void DriveX(double speed, double feet) {
		DTEncFL.reset();
		DTEncFR.reset();
		DTEncRL.reset();
		DTEncRR.reset();
		
		DTFLPID.setF(speed);
		DTFRPID.setF(speed);
		DTRLPID.setF(speed);
		DTRRPID.setF(speed);
		
		Timer driveTimer = new Timer();
		driveTimer.reset();
		driveTimer.start();
		
		while (Math.abs(DTEncFL.get()) < Math.abs(feet * Config.encUnit)
				&& Math.abs(DTEncFR.get()) < Math.abs(feet * Config.encUnit)
				&& Math.abs(DTEncRR.get()) < Math.abs(feet * Config.encUnit)
				&& Math.abs(DTEncRL.get()) < Math.abs(feet * Config.encUnit) && !teleOpCalled && autotime.get() < 15) {
			
			if (driveTimer.get() < 0.15) {
				driveLinearX(speed * driveTimer.get() * (1.0 / 0.15), DTFLPID, DTFRPID, DTRLPID, DTRRPID);
			} else {
				driveLinearX(speed, DTFLPID, DTFRPID, DTRLPID, DTRRPID);
			}
			
			driveLinearX(speed, DTFLPID, DTFRPID, DTRLPID, DTRRPID);
		}
		
		driveTimer.stop();
		driveTimer.reset();
		
		DTFLPID.setF(0);
		DTFRPID.setF(0);
		DTRLPID.setF(0);
		DTRRPID.setF(0);
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
				driveLinearY(speed * driveTimer.get() * (1.0 / 0.15), DTFLPID, DTFRPID, DTRLPID, DTRRPID);
			} else {
				driveLinearY(speed, DTFLPID, DTFRPID, DTRLPID, DTRRPID);
			}
		}
		
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
				driveRotational(speed * driveTimer.get() * (1.0 / 0.15), DTFLPID, DTFRPID, DTRLPID, DTRRPID);
			} else {
				driveRotational(speed, DTFLPID, DTFRPID, DTRLPID, DTRRPID);
			}
		}
		
		driveTimer.stop();
		driveTimer.reset();
	}

	public void driveLinearY(double speed, PIDController frontLeftPID, PIDController frontRightPID,
			PIDController rearLeftPID, PIDController rearRightPID) {
		ControlMode cm = ControlMode.PercentOutput;

		double[] wheelSpeeds = new double[4];
		wheelSpeeds[MotorType.kFrontLeft.value] = speed;
		wheelSpeeds[MotorType.kFrontRight.value] = -speed;
		wheelSpeeds[MotorType.kRearLeft.value] = speed;
		wheelSpeeds[MotorType.kRearRight.value] = -speed;

		normalize(wheelSpeeds);
//		frontLeftPID.setSetpoint(wheelSpeeds[MotorType.kFrontLeft.value]);
//		frontRightPID.setSetpoint(wheelSpeeds[MotorType.kFrontRight.value]);
//		rearLeftPID.setSetpoint(wheelSpeeds[MotorType.kRearLeft.value]);
//		rearRightPID.setSetpoint(wheelSpeeds[MotorType.kRearRight.value]);
		 frontLeft.set(cm, wheelSpeeds[MotorType.kFrontLeft.value]);
		 frontRight.set(cm, wheelSpeeds[MotorType.kFrontRight.value]);
		 rearLeft.set(cm, wheelSpeeds[MotorType.kRearLeft.value]);
		 rearRight.set(cm, wheelSpeeds[MotorType.kRearRight.value]);
	}

	public void driveLinearX(double speed, PIDController frontLeftPID, PIDController frontRightPID,
			PIDController rearLeftPID, PIDController rearRightPID) {
		ControlMode cm = ControlMode.PercentOutput;

		double[] wheelSpeeds = new double[4];
		wheelSpeeds[MotorType.kFrontLeft.value] = speed;
		wheelSpeeds[MotorType.kFrontRight.value] = speed;
		wheelSpeeds[MotorType.kRearLeft.value] = -speed;
		wheelSpeeds[MotorType.kRearRight.value] = -speed;

		normalize(wheelSpeeds);
//		frontLeftPID.setSetpoint(wheelSpeeds[MotorType.kFrontLeft.value]);
//		frontRightPID.setSetpoint(wheelSpeeds[MotorType.kFrontRight.value]);
//		rearLeftPID.setSetpoint(wheelSpeeds[MotorType.kRearLeft.value]);
//		rearRightPID.setSetpoint(wheelSpeeds[MotorType.kRearRight.value]);
		 frontLeft.set(cm, wheelSpeeds[MotorType.kFrontLeft.value]);
		 frontRight.set(cm, wheelSpeeds[MotorType.kFrontRight.value]);
		 rearLeft.set(cm, wheelSpeeds[MotorType.kRearLeft.value]);
		 rearRight.set(cm, wheelSpeeds[MotorType.kRearRight.value]);
	}

	public void driveRotational(double speed, PIDController frontLeftPID, PIDController frontRightPID,
			PIDController rearLeftPID, PIDController rearRightPID) {
		ControlMode cm = ControlMode.PercentOutput;
		
		double[] wheelSpeeds = new double[4];
		wheelSpeeds[MotorType.kFrontLeft.value] = speed;
		wheelSpeeds[MotorType.kFrontRight.value] = speed;
		wheelSpeeds[MotorType.kRearLeft.value] = speed;
		wheelSpeeds[MotorType.kRearRight.value] = speed;

		normalize(wheelSpeeds);
//		frontLeftPID.setSetpoint(wheelSpeeds[MotorType.kFrontLeft.value]);
//		frontRightPID.setSetpoint(wheelSpeeds[MotorType.kFrontRight.value]);
//		rearLeftPID.setSetpoint(wheelSpeeds[MotorType.kRearLeft.value]);
//		rearRightPID.setSetpoint(wheelSpeeds[MotorType.kRearRight.value]);
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
