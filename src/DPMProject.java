import java.util.ArrayList;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;

/**
 * This class contains main method
 * @author Oruj Ahmadov, Sangmoon Hwang
 * 
 */
public class DPMProject {

	/**
	 * The main method.
	 *
	 * @param args the arguments
	 */
	public static void main(String[] args) {
		int buttonChoice;
		final NXTRegulatedMotor pickUpMotor = Motor.C;
		// setup the odometer, display, and ultrasonic and light sensors
		Odometer odo = new Odometer();
		OdometryCorrection odoCorrection = new OdometryCorrection(odo);
		OdometryDisplay display = new OdometryDisplay(odo);
		OdometryCorrection oc = new OdometryCorrection(odo);
		UltrasonicSensor usFront = new UltrasonicSensor(SensorPort.S2);
		UltrasonicSensor usBack = new UltrasonicSensor(SensorPort.S3);
		Navigation nav = new Navigation(odo);
		Localization ori = new Localization(odo, nav,usFront, usBack);
		SearchAndRescue searchAndRescue = new SearchAndRescue(odo, nav, usFront, usBack, ori);
		int counter=0;
		ArrayList inputs = new ArrayList();
		
	
	 

		
			// clear the display
			LCD.clear();

			// ask the user whether the motors should drive in a square or float
			LCD.drawString("Enter map number", 0, 0);
	

	
		
		

		

			buttonChoice = Button.waitForAnyPress();
		 while (buttonChoice != Button.ID_ESCAPE){

		if (buttonChoice == Button.ID_LEFT) {
			counter++;
			
			LCD.drawInt(counter,0,1);

			buttonChoice = Button.waitForAnyPress();

//			searchAndRescue.startMission();


		} else if (buttonChoice == Button.ID_RIGHT) {
		    inputs.add(counter);
			counter=0;
			LCD.clear();
			LCD.drawString("Dropoff X,Y", 0, 0);
			LCD.drawInt(counter,0,1);
			buttonChoice = Button.waitForAnyPress();
			
			

		}
		
		else if (buttonChoice == Button.ID_ENTER) {
		 
			
			for (NXTRegulatedMotor motor : new NXTRegulatedMotor[] { Motor.A,
                    Motor.B, Motor.C }) {
                motor.forward();
                motor.flt();
 
            }
			try{
			odo.start();
			display.start();
            searchAndRescue.startMission((Integer)inputs.get(0),(Integer)inputs.get(1), (Integer)inputs.get(2));
			} catch(IllegalStateException ex){
				
			}
		  
		   } 
			
			
			

		}
		 
	

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}

	/**
	 * converts to the distance in cm.
	 *
	 * @param radius wheel radius
	 * @param distance desired distance
	 * @return converted distance in cm
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * convert to the angle in degrees.
	 *
	 * @param radius wheel radius
	 * @param width wheel base
	 * @param angle angle to turn
	 * @return converted angle in degrees
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
}
