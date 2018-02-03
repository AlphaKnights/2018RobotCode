package org.usfirst.frc.team6695.robot;

import edu.wpi.first.wpilibj.DigitalInput;

public class ModeSelector {
	/** Input for left switch */
	DigitalInput switch1; // Controls starting position
	/** Input for center switch */
	DigitalInput switch2; // Controls starting position
	/** Input for right switch */
	DigitalInput switch3; // Controls switch or scale

	public ModeSelector(int left, int right, int center) {
		switch1 = new DigitalInput(left);
		switch2 = new DigitalInput(right);
		switch3 = new DigitalInput(center);
	}

	public boolean[] getStartingMode() {
		boolean switchMode[] = { switch1.get(), switch2.get(), switch3.get() };
		return switchMode;
	}

}