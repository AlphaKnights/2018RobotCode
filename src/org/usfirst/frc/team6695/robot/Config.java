package org.usfirst.frc.team6695.robot;

public final class Config {

	/* Talon SRX */
	static final int DriveTrainFrontLeft = 1;
	static final int DriveTrainRearLeft = 2;
	static final int DriveTrainFrontRight = 4;
	static final int DriveTrainRearRight = 3;
	static final int LiftRightMotor = 5;
	static final int LiftLeftMotor = 6;
	static final int liftGrabberMotor = 7;

	static final int JoystickChannel = 0;
	static final int XBoxChannel = 1;
	
	static final int EncoderTopValue = 1000;
	static final int LiftLeftEncoderPort = 0; //TODO: PLACEHOLDER
	static final int LiftRightEncoderPort = 0; //TODO: PLACEHOLDER
	static final int EncoderRange = 20;

	// 1 foot
	static final double encUnit = 200 / (Math.PI / 2);

	// 360 degrees
	static final double degUnit = 100;

	static final int DrivetrainEncoderFrontRight = 0; //TODO: PLACEHOLDER
	static final int DrivetrainEncoderFrontLeft = 0; //TODO: PLACEHOLDER
	static final int DrivetrainEncoderRearRight = 0; //TODO: PLACEHOLDER
	static final int DrivetrainEncoderRearLeft = 0; //TODO: PLACEHOLDER
}
