package org.usfirst.frc.team6695.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.PIDOutput;

public final class RateControlledMotor implements PIDOutput {

	private final TalonSRX talon;
	
	public RateControlledMotor(TalonSRX motorController) {
		talon = motorController;
	}
	
	public void pidWrite(double output) {
		double rateOutput = talon.getMotorOutputPercent() + output;
		rateOutput = Math.min(1.0, rateOutput);
		rateOutput = Math.max(-1.0, rateOutput);
		talon.set(talon.getControlMode(), rateOutput);
	}
	
}
