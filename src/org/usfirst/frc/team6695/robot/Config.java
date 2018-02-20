package org.usfirst.frc.team6695.robot;

public final class Config {

	/* Talon SRX */
	static final int DriveTrainFrontLeft = 1;
	static final int DriveTrainRearLeft = 2;
	static final int DriveTrainFrontRight = 4;
	static final int DriveTrainRearRight = 3;
	static final int LiftMotor = 5;
	static final int LiftSpinMotor = 8;
	static final int liftGrabberMotor = 7;
	static final int ForkLiftMotor = 9;

	static final int JoystickChannel = 0;
	static final int XBoxChannel = 1;

	static final int LiftHiLimitPort = 9;
	static final int LiftMidLimitPort = 5;
	static final int LiftLoLimitPort = 7;
	static final int GrabHiLimitPort = 8;

	static final int EncoderTopValue = 10000;
	static final int LiftEncoderPort = 4;
	static final int EncoderRange = 200;

	// static final int AutoSwitchLeftPosPort = 10;
	// static final int AutoSwitchMiddlePosPort = 11;
	// static final int AutoSwitchRightPosPort = 12;
	// static final int AutoSwitchStraightPort = 13;
	// static final int AutoSwitchScalePort = 20;
	// static final int AutoSwitchSwitchPort = 22;
	// static final int AutoSwitchDelay2sPort = 18;
	// static final int AutoSwitchDelay5sPort = 19;
	static final int AutoSwitchLeftPosPort = 10;
	static final int AutoSwitchMiddlePosPort = 11;
	static final int AutoSwitchRightPosPort = 12;
	static final int AutoSwitchStraightPort = 13;
	static final int AutoSwitchScalePort = 14;
	static final int AutoSwitchSwitchPort = 15;
	static final int AutoSwitchDelay2sPort = 16;
	static final int AutoSwitchDelay5sPort = 17;

	// 1 foot
	static final double encUnit = (529 + 541 + 578 + 655) / (11 * 4);

	// 360 degrees
	static final double degUnit = 100;

	static final int DrivetrainEncoderFrontRight = 1;
	static final int DrivetrainEncoderFrontLeft = 0;
	static final int DrivetrainEncoderRearRight = 3;
	static final int DrivetrainEncoderRearLeft = 2;
}
