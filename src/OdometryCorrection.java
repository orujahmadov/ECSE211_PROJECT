/*
 * OdometryCorrection.java
 */

import java.util.Arrays;
import java.util.LinkedList;

import lejos.nxt.ColorSensor;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;

/**
 * This is the A version of odometer correction
 * @version A 2.0
 * @author sangmoon hwang, oruj Ahmadov
 *
 */
public class OdometryCorrection extends Thread {

	//Testing purposes
	public static int aval,mval;
	public static long ltt, rtt;
	//End

	public static boolean is_Back = false;
	static double r_lightSensorValue;
	static double l_lightSensorValue;

	private static final long CORRECTION_PERIOD = 50; // 10 ms
	private static final double DIFFERENCE_BETWEEN_CENTER_AND_SENSOR = 4;
	private static final double WIDTH_BETWEEN_SENSORS = 13;
	private static final double WIDTH = 13.0923;
	private final double WHEEL_RADIUS = 1.621;
	private static Odometer odometer;
	private boolean fl = false, fr = false, seen_L = false, seen_R = false;



	private double LIGHT_THRESHOLD = 0; //515
	int ang = 0; //0:n, 1:e, 2:s, 3:w
	private static int forward_Speed = 400; // degree per minute
	private int diff_Bandwith = 20;
	private int diff_Line_Tile = 100;
	long l = 0, r = 0;
	private Object obj = new Object();
	// constructor
	/**
	 * constructor
	 * @param odometer odometer to use
	 */
	public OdometryCorrection(Odometer odometer) {
		this.odometer = odometer;
		OdometryCorrection.r_lightSensorValue = 0;
		OdometryCorrection.l_lightSensorValue = 0;
	}

	// run method (required for Thread)
	public void run() {
		ColorSensor left_Lightsensor = new ColorSensor(SensorPort.S1);
		ColorSensor right_Lightsensor = new ColorSensor(SensorPort.S4);
		left_Lightsensor.setFloodlight(true);
		right_Lightsensor.setFloodlight(true);

		//median filtered
		LIGHT_THRESHOLD = (filteredLightVal(right_Lightsensor) + filteredLightVal(left_Lightsensor))/2 - 65;
		// end of initialization for the light sensor using median filter

		long correctionStart, correctionEnd;   


		boolean s1 = true, s2 = false, s3 = false;

		while (true) {

			//Testing
			//          aval = filteredLightVal_Avg(left_Lightsensor);
			//          mval = filteredLightVal(left_Lightsensor);
			//ENd
			correctionStart = System.currentTimeMillis();
			if(odometer.getX()<45 && odometer.getY()<45 ){//&& !SearchAndRescue.isDone){
			}
			else{
				// put your correction code here
				OdometryCorrection.r_lightSensorValue = filteredLightVal(right_Lightsensor)/*right_Lightsensor.getNormalizedLightValue()*/;
				OdometryCorrection.l_lightSensorValue = filteredLightVal(left_Lightsensor)/*left_Lightsensor.getNormalizedLightValue()*/;
				//correctPosition();
				if(l_lightSensorValue <= LIGHT_THRESHOLD && Motor.A.getSpeed() == forward_Speed && !seen_L){
					if(fl){
						synchronized(obj){
						l = System.currentTimeMillis();
						}
						//                  ltt = l; //Test
						fl = false;
						seen_L = true;
						Sound.beep();
					}
					fl = true;
				}

				if(r_lightSensorValue <= LIGHT_THRESHOLD && Motor.A.getSpeed() == forward_Speed && !seen_R){
					if(fr){
						synchronized(obj){
						r = System.currentTimeMillis();
						}
						//                  rtt = r; //Test
						fr = false;
						seen_R = true;
						Sound.beep();
					}
					fr = true;
				}

				if(l!=0 && r!=0 && Math.abs(l-r) < 150){
					correctPosition();
					//              correct_Odo_Ang(l,r);
					//              correct_All_Ang(l,r);
					//
					correct_Physical_Ang(l, r);
					correctAng();
					r = 0;
					l = 0;
					seen_R = false;
					seen_L = false;
				}
				else if (l != 0 && r!=0){
					correctPosition();

					correct_Physical_Ang(l,r);
					correctAng();
					//                              correct_Odo_Ang(l,r);
					//                              correct_All_Ang(l,r);
					r = 0;
					l = 0;
					seen_R = false;
					seen_L = false;
				}
				else if (Math.abs(l-r) > 2800 && seen_R && seen_L){ //flush
					r = 0;
					l = 0;
					seen_R = false;
					seen_L = false;
				}
			}
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD
							- (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometry correction will be
					// interrupted by another thread
				}
			}
		}
	}


	//  }
	/**
	 * it corrects only the robot's physical heading
	 * @param l time when left sensor detects a line
	 * @param r time when right sensor detects a line
	 */
	private void correct_Physical_Ang(long l, long r){
//		Sound.beep();
		synchronized(obj){
		if(l>r){
			Motor.A.setSpeed(200);
			for(int i = 0; i < (l-r); i++){}
			Motor.A.setSpeed(forward_Speed);
		}
		else{
			Motor.B.setSpeed(200);
			for(int i = 0; i < (r-l); i++){}
			Motor.B.setSpeed(forward_Speed);
		}
		}
	}

	/**
	 * it corrects both the odometer theta and the robot's physical heading
	 * @param l time when left sensor detects a line
	 * @param r time when right sensor detects a line
	 */
	private void correct_All_Ang(long l, long r) {
		correctAng();
		if(l > r){
			switch(ang){
			case 0: //north
				double n0 = odometer.getTheta();
				double dist0 = 2 * Math.PI * WHEEL_RADIUS * (forward_Speed/360) * ((l-r) / 60000); //ms to min
				double offset0 = Math.asin(dist0/WIDTH_BETWEEN_SENSORS);
				turnBy(offset0);
				break;
			case 1: //east
				double e = odometer.getTheta();
				double n1 = odometer.getTheta();
				double dist1 = 2 * Math.PI * WHEEL_RADIUS * (forward_Speed/360) * ((l-r) / 60000); //ms to min
				double offset1 = Math.asin(dist1/WIDTH_BETWEEN_SENSORS);
				turnBy(offset1);
				break;
			case 2: //south
				double s = odometer.getTheta();
				double n2 = odometer.getTheta();
				double dist2 = 2 * Math.PI * WHEEL_RADIUS * (forward_Speed/360) * ((l-r) / 60000); //ms to min
				double offset2 = Math.asin(dist2/WIDTH_BETWEEN_SENSORS);
				turnBy(offset2);
				break;
			case 3: //west
				double w = odometer.getTheta();
				double n3 = odometer.getTheta();
				double dist3 = 2 * Math.PI * WHEEL_RADIUS * (forward_Speed/360) * ((l-r) / 60000); //ms to min
				double offset3 = Math.asin(dist3/WIDTH_BETWEEN_SENSORS);
				turnBy(offset3);
				break;
			}
		}
		else{
			switch(ang){
			case 0: //north
				double dist0 = 2 * Math.PI * WHEEL_RADIUS * (forward_Speed/360) * (-(l-r) / 60000); //ms to min
				double offset0 = Math.asin(dist0/WIDTH_BETWEEN_SENSORS);
				turnBy(-offset0);
				break;
			case 1: //east
				double dist1 = 2 * Math.PI * WHEEL_RADIUS * (forward_Speed/360) * (-(l-r) / 60000); //ms to min
				double offset1 = Math.asin(dist1/WIDTH_BETWEEN_SENSORS);
				turnBy(-offset1);
				break;
			case 2: //south
				double dist2 = 2 * Math.PI * WHEEL_RADIUS * (forward_Speed/360) * (-(l-r) / 60000); //ms to min
				double offset2 = Math.asin(dist2/WIDTH_BETWEEN_SENSORS);
				turnBy(-offset2);
				break;
			case 3: //west
				double dist3 = 2 * Math.PI * WHEEL_RADIUS * (forward_Speed/360) * (-(l-r) / 60000); //ms to min
				double offset3 = Math.asin(dist3/WIDTH_BETWEEN_SENSORS);
				turnBy(-offset3);
				break;
			}
		}
		correctAng();
	}

	private void turnBy(double d) {
		Sound.beep();
		d = -Math.toDegrees(d);
		Motor.A.rotate(convertAngle(WHEEL_RADIUS, WIDTH, d), true);
		Motor.B.rotate(-convertAngle(WHEEL_RADIUS, WIDTH, d), true);
	}

	/**
	 * it corrects only the odometer theta
	 * @param l time when left sensor detects a line
	 * @param r time when right sensor detects a line
	 */
	private void correct_Odo_Ang(long l, long r) {

		if(l > r){
			switch(ang){
			case 0: //north
				double n0 = odometer.getTheta();
				double dist0 = 2 * Math.PI * WHEEL_RADIUS * (forward_Speed/360) * ((l-r) / 60000); //ms to min
				double offset0 = Math.asin(dist0/WIDTH_BETWEEN_SENSORS);
				odometer.setTheta(offset0 + n0);
				break;
			case 1: //east
				double e = odometer.getTheta();
				double n1 = odometer.getTheta();
				double dist1 = 2 * Math.PI * WHEEL_RADIUS * (forward_Speed/360) * ((l-r) / 60000); //ms to min
				double offset1 = Math.asin(dist1/WIDTH_BETWEEN_SENSORS);
				odometer.setTheta(offset1 + n1);
				break;
			case 2: //south
				double s = odometer.getTheta();
				double n2 = odometer.getTheta();
				double dist2 = 2 * Math.PI * WHEEL_RADIUS * (forward_Speed/360) * ((l-r) / 60000); //ms to min
				double offset2 = Math.asin(dist2/WIDTH_BETWEEN_SENSORS);
				odometer.setTheta(offset2 + n2);
				break;
			case 3: //west
				double w = odometer.getTheta();
				double n3 = odometer.getTheta();
				double dist3 = 2 * Math.PI * WHEEL_RADIUS * (forward_Speed/360) * ((l-r) / 60000); //ms to min
				double offset3 = Math.asin(dist3/WIDTH_BETWEEN_SENSORS);
				odometer.setTheta(offset3 + n3);
				break;
			}
		}
		else{
			switch(ang){
			case 0: //north
				double n0 = odometer.getTheta();
				double dist0 = 2 * Math.PI * WHEEL_RADIUS * (forward_Speed/360) * (-(l-r) / 60000); //ms to min
				double offset0 = Math.asin(dist0/WIDTH_BETWEEN_SENSORS);
				odometer.setTheta(-offset0 + n0);
				break;
			case 1: //east
				double e = odometer.getTheta();
				double n1 = odometer.getTheta();
				double dist1 = 2 * Math.PI * WHEEL_RADIUS * (forward_Speed/360) * (-(l-r) / 60000); //ms to min
				double offset1 = Math.asin(dist1/WIDTH_BETWEEN_SENSORS);
				odometer.setTheta(-offset1 + n1);
				break;
			case 2: //south
				double s = odometer.getTheta();
				double n2 = odometer.getTheta();
				double dist2 = 2 * Math.PI * WHEEL_RADIUS * (forward_Speed/360) * (-(l-r) / 60000); //ms to min
				double offset2 = Math.asin(dist2/WIDTH_BETWEEN_SENSORS);
				odometer.setTheta(-offset2 + n2);
				break;
			case 3: //west
				double w = odometer.getTheta();
				double n3 = odometer.getTheta();
				double dist3 = 2 * Math.PI * WHEEL_RADIUS * (forward_Speed/360) * (-(l-r) / 60000); //ms to min
				double offset3 = Math.asin(dist3/WIDTH_BETWEEN_SENSORS);
				odometer.setTheta(-offset3 + n3);
				break;
			}
		}
	}

	/**
	 * corrects the x and y positions according to the odometer readings
	 */
	private void correctPosition() {
		double theta = odometer.getTheta();
		if (theta < 0){
			theta = Math.toRadians(360) + theta;
		}
		if (theta >= Math.toRadians(360)){
			theta -= Math.toRadians(360);
		}
		// If Y has to be changed
		if (theta > Math.toRadians(315) || theta < Math.toRadians(45) ||
				(Math.toRadians(135) < theta && theta < Math.toRadians(225)) ) {
			double y = odometer.getY();
			y = gridLineDetector(y);
			// if we're going north, then add the correction. Else, subtract.
			if (theta > Math.toRadians(315) || theta < Math.toRadians(45)) {
				ang = 0; //will correct the heading to north
				odometer.setY(y);// + DIFFERENCE_BETWEEN_CENTER_AND_SENSOR);
			}
			else {
				ang = 2; //will correct the heading to south
				odometer.setY(y);// - DIFFERENCE_BETWEEN_CENTER_AND_SENSOR);
			}
		}
		else { // If X has to be changed
			double x = odometer.getX();
			x = gridLineDetector(x);
			// If we're going east, then add. Else, subtract.
			if (theta > Math.toRadians(45) && (Math.toRadians(135) > theta)){
				ang = 1;
				odometer.setX(x);// + DIFFERENCE_BETWEEN_CENTER_AND_SENSOR);
			}
			else {
				ang = 3;
				odometer.setX(x);// - DIFFERENCE_BETWEEN_CENTER_AND_SENSOR);
			}
		}
		//      try {
		//          Thread.sleep(100); // Make sure that you don't read two lines too quickly
		//      } catch (InterruptedException e) {
		//          // TODO Auto-generated catch block
		//      }
	}


	/**
	 * corrects approximately the angle of the robot according to the odometer readings
	 */
	private void correctAng() {
		if(Math.abs(odometer.getTheta()) > Math.toRadians(360)){
			odometer.setTheta(odometer.getTheta()%Math.toRadians(360));
		}
		switch(ang){
		case 0:
			odometer.setTheta(Math.toRadians(0));
			break;
		case 1:
			if(odometer.getTheta() < 0)
				odometer.setTheta(-Math.toRadians(270));
			else
				odometer.setTheta(Math.toRadians(90));
			break;
		case 2:
			if(odometer.getTheta() < 0)
				odometer.setTheta(-Math.toRadians(180));
			else
				odometer.setTheta(Math.toRadians(180));
			break;
		case 3:
			if(odometer.getTheta() < 0)
				odometer.setTheta(-Math.toRadians(90));
			else
				odometer.setTheta(Math.toRadians(270));
			break;
		}
	}

	/**
	 * median filter to remove possible noise in the color sensor readings
	 * @param cs colorsensor to be filtered
	 * @return filtered reading
	 */
	private int filteredLightVal(ColorSensor cs){
		int i []= new int[5];
		for (int j = 0; j < 5; j++){
			i[j] = cs.getNormalizedLightValue();
		}
		Arrays.sort(i);
		return i[2];
	}


	/**
	 * average filter to remove possible noise in the color sensor readings
	 * @param ls left sensor
	 * @param rs right sensor
	 * @return filtered reading
	 */
	private int filteredLightVal_Avg(ColorSensor cs){
		int b = 0;

		for(int i = 0 ; i < 5; i++){
			b += cs.getNormalizedLightValue();
		}

		return b/3;
	}

	// This helps us to get the correct value of the odometer from the line
	/**
	 * wrap up the odometer readings into a formalized line positions.
	 * @param readValue current odometer position
	 * @return actual position
	 */
	public static double gridLineDetector (double readValue) {

		//      int rounded;
		//      if ((readValue % 1) < 0.5 ) { // Rounding down
		//          rounded = (int) readValue;
		//      }
		//      else { // Rounding up
		//          rounded = (int) readValue + 1;
		//      }
		//      int leftOver = (rounded-15) % 30;
		//      int quotient = (rounded-15) / 30;
		//      int gap = 0;
		//      if (leftOver > 15)//8
		//      {
		//          gap = 30;
		//      }
		//      Sound.beep(); //@TEST
		////        if(is_Back){
		////            return (double) (quotient * 30 + gap - 30); // +15
		////        }
		////        else{
		//          return (double) (quotient * 30 + gap); // +15
		////        }
		double[] yGridCoordinatesUp = new double[8];
		double[] yGridCoordinatesDown = new double[8];
		double[] xGridCoordinatesLeft = new double[8];
		double[] xGridCoordinatesRight = new double[8];

		for(int w=0; w<8; w++){
			yGridCoordinatesUp[w]=30*w + DIFFERENCE_BETWEEN_CENTER_AND_SENSOR;
			yGridCoordinatesDown[w]=30*w - DIFFERENCE_BETWEEN_CENTER_AND_SENSOR;
			xGridCoordinatesRight[w]=30*w + DIFFERENCE_BETWEEN_CENTER_AND_SENSOR;
			xGridCoordinatesLeft[w]=30*w - DIFFERENCE_BETWEEN_CENTER_AND_SENSOR;
		}
		for (int i = 0; i < 8; i++) {
			if (goingStraight()) {
				if ((Math.abs(odometer.getX() - xGridCoordinatesRight[i]) < 11)
						&& (Math.abs(odometer.getTheta() - 1.55) < 0.1)) {
					// only correct odometer if robot is not turning

					// beep to know correction has been made

					readValue = xGridCoordinatesRight[i];

				} else if ((Math.abs(odometer.getX()
						- xGridCoordinatesLeft[i]) < 11)
						&& (Math.abs(odometer.getTheta() - 4.65) < 0.1)) {


					readValue = xGridCoordinatesLeft[i];

				} else if ((Math.abs(odometer.getY()
						- yGridCoordinatesDown[i]) < 11)
						&& (Math.abs(odometer.getTheta() - 3.10) < 0.1)) {


					readValue = yGridCoordinatesDown[i];

				} else if ((Math.abs(odometer.getY()
						- yGridCoordinatesUp[i]) < 11)
						&& (Math.abs(odometer.getTheta() - 0.00) < 0.1)) {


					readValue = yGridCoordinatesUp[i];

				}
			}
		}



		return readValue;
	}
	/**
	 * converts to the distance in cm
	 * @param radius wheel radius
	 * @param distance desired distance
	 * @return converted distance in cm
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * convert to the angle in degrees
	 * @param radius wheel radius
	 * @param width wheel base
	 * @param angle angle to turn
	 * @return converted angle in degrees
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	private static boolean goingStraight(){
		return (Motor.A.getSpeed() == forward_Speed && Motor.B.getSpeed() == forward_Speed);
	}

}
