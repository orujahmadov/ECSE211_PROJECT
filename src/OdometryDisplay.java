/*
 * OdometryDisplay.java
 */
import lejos.nxt.LCD;

/**
 * This class displays odometer values on display
 * @author Oruj Ahmadov, Sangmoon Hwang
 * 
 */
public class OdometryDisplay extends Thread {
	
	/** The Constant DISPLAY_PERIOD. */
	private static final long DISPLAY_PERIOD = 250;
	
	/** The odometer. */
	private final Odometer odometer;

	// constructor
	/**
	 * Instantiates a new odometry display.
	 *
	 * @param odometer the odometer
	 */
	public OdometryDisplay(Odometer odometer) {
		this.odometer = odometer;
	}

	// run method (required for Thread)
	/**
	 * Run.
	 */
	@Override
	public void run() {
		long displayStart, displayEnd;
		double[] position = new double[3];

		// clear the display once
		LCD.clearDisplay();

		while (true) {
			displayStart = System.currentTimeMillis();

			// clear the lines for displaying odometry information
			//Test
//			LCD.drawString("A:              ", 0, 0);
//			LCD.drawString("M:              ", 0, 1);
			//ENd
			LCD.drawString("X:              ", 0, 0);
			LCD.drawString("Y:              ", 0, 1);
			LCD.drawString("T:              ", 0, 2);
//			LCD.drawString("Dt:              ", 0, 3); // Testing
			
			//Test
//			LCD.drawInt(OdometryCorrection.aval, 3, 0);
//			LCD.drawInt(OdometryCorrection.mval, 3, 1);
			//End
			
			// get the odometry information
			odometer.getPosition(position);
//			LCD.drawInt((int) (OdometryCorrection.ltt - OdometryCorrection.rtt), 3, 3); // testing
			// display odometry information
			for (int i = 0; i < 3; i++) {
				LCD.drawString(formattedDoubleToString(position[i], 2), 3, i);
			}

			// throttle the OdometryDisplay
			displayEnd = System.currentTimeMillis();
			if (displayEnd - displayStart < DISPLAY_PERIOD) {
				try {
					Thread.sleep(DISPLAY_PERIOD - (displayEnd - displayStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that OdometryDisplay will be interrupted
					// by another thread
				}
			}
		}
	}
	
	/**
	 * Formatted double to string.
	 *
	 * @param x the x
	 * @param places the places
	 * @return the string
	 */
	private static String formattedDoubleToString(double x, int places) {
		String result = "";
		String stack = "";
		long t;

		// put in a minus sign as needed
		if (x < 0.0)
			result += "-";

		// put in a leading 0
		if (-1.0 < x && x < 1.0)
			result += "0";
		else {
			t = (long) x;
			if (t < 0)
				t = -t;

			while (t > 0) {
				stack = Long.toString(t % 10) + stack;
				t /= 10;
			}

			result += stack;
		}

		// put the decimal, if needed
		if (places > 0) {
			result += ".";

			// put the appropriate number of decimals
			for (int i = 0; i < places; i++) {
				x = Math.abs(x);
				x = x - Math.floor(x);
				x *= 10.0;
				result += Long.toString((long) x);
			}
		}

		return result;
	}

}
