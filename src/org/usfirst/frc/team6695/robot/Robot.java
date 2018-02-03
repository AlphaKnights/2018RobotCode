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

	@Override
	public void autonomousInit() {

		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		autonomousPathfinding(gameData, null);

	}
	
	public void autonomousPathfinding(String gameData, boolean[] getStartingMode) {
		
		if(getStartingMode[0] == false && getStartingMode[1] == false && getStartingMode[2] == false) { // DISABLED_START with switch priority
			
		} else if(getStartingMode[0] == false && getStartingMode[1] == false && getStartingMode[2] == true) { // DISABLED_START with scale priority
			
		} else if(getStartingMode[0] == false && getStartingMode[1] == true && getStartingMode[2] == false) { // RIGHT_START with switch priority
			
		} else if(getStartingMode[0] == false && getStartingMode[1] == true && getStartingMode[2] == true) { // RIGHT_START with scale priority
			
		} else if(getStartingMode[0] == true && getStartingMode[1] == false && getStartingMode[2] == false) { // LEFT_START with switch priority
			
		} else if(getStartingMode[0] == true && getStartingMode[1] == false && getStartingMode[2] == true) { // LEFT_START with scale priority
			
		} else if(getStartingMode[0] == true && getStartingMode[1] == true && getStartingMode[2] == false) { // CENTER_START with switch priority
			
		} else if(getStartingMode[0] == true && getStartingMode[1] == true && getStartingMode[2] == true) { // CENTER_START with scale priority
			
		} else {
			// Throw errors here
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
