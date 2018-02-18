package org.usfirst.frc.team6695.robot;

import edu.wpi.first.wpilibj.DigitalInput;

public class ModeSelector {
	/** Input for switch1 */
	DigitalInput switch1;
	/** Input for switch2 */
	DigitalInput switch2;
	/** Input for switch3 */
	DigitalInput switch3;
	/** Input for switch4 */
	DigitalInput switch4;
	/** Input for switch5 */
	DigitalInput switch5;
	/** Input for switch6 */
	DigitalInput switch6;
	/** Input for switch7 */
	DigitalInput switch7;
	/** Input for switch8 */
	DigitalInput switch8;

	/**
	 * 
	 * @param _1
	 *            DIO Port for Switch 1
	 * @param _2
	 *            DIO Port for Switch 2
	 * @param _3
	 *            DIO Port for Switch 3
	 * @param _4
	 *            DIO Port for Switch 4
	 * @param _5
	 *            DIO Port for Switch 5
	 * @param _6
	 *            DIO Port for Switch 6
	 * @param _7
	 *            DIO Port for Switch 7
	 * @param _8
	 *            DIO Port for Switch 8
	 */
	public ModeSelector(int _1, int _2, int _3, int _4, int _5, int _6, int _7, int _8) {
		switch1 = new DigitalInput(_1);
		switch2 = new DigitalInput(_2);
		switch3 = new DigitalInput(_3);
		switch4 = new DigitalInput(_4);
		switch5 = new DigitalInput(_5);
		switch6 = new DigitalInput(_6);
		switch7 = new DigitalInput(_7);
		switch8 = new DigitalInput(_8);
	}

	public boolean[] getSwitches() {
		boolean switchMode[] = { switch1.get(), switch2.get(), switch3.get(), switch4.get(), switch5.get(),
				switch6.get(), switch7.get(), switch8.get() };

		return switchMode;
	}

	public boolean hasError() { // TODO Make Has Error Code and test it
		// test for bottem row errors
		if ((switch1.get() && switch2.get()) || (switch2.get() && switch3.get()) || (switch1.get() && switch3.get()))
			return true;
		// test for second row errors
		if ((switch4.get() && switch5.get()) || (switch5.get() && switch6.get()) || (switch4.get() && switch6.get()))
			return true;
		return false;
	}

}