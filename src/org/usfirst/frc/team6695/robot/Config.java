package org.usfirst.frc.team6695.robot;

public final class Config {

	/* Talon SRX */
	static final int DriveTrainFrontLeft = 1;
	static final int DriveTrainRearLeft = 2;
	static final int DriveTrainFrontRight = 4;
	static final int DriveTrainRearRight = 3;
	static final int LiftMotor = 5;
	static final int liftGrabberMotor = 7;
	static final int ForkLiftMotor = 9;

	static final int JoystickChannel = 0;
	static final int XBoxChannel = 1;

	static final int EncoderTopValue = 10000;
	static final int LiftEncoderPort = 1; // TODO: PLACEHOLDER
	static final int EncoderRange = 200;

	// 1 foot
	static final double encUnit = 200 / (Math.PI / 2);

	// 360 degrees
	static final double degUnit = 100;

	static final int DrivetrainEncoderFrontRight = 3; // TODO: PLACEHOLDER
	static final int DrivetrainEncoderFrontLeft = 4; // TODO: PLACEHOLDER
	static final int DrivetrainEncoderRearRight = 5; // TODO: PLACEHOLDER
	static final int DrivetrainEncoderRearLeft = 6; // TODO: PLACEHOLDER

}
