import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
  
public class Navigation extends Thread {
  
    private final Odometer odometer;
    UltrasonicSensor us = new UltrasonicSensor(SensorPort.S1);
  
    private static final int FORWARD_SPEED = 400;
    private static final int ROTATE_SPEED = 200;
    private static final int MAX_ACCELERATION = 2000;
    private final double wheelRadius = 1.621;
    private final double wheelBase = 13.1701;
      
    public double destinationX;
    public double destinationY;
 
  
    private final NXTRegulatedMotor leftMotor = Motor.A, rightMotor = Motor.B;
  
    public Navigation(Odometer odom) {
        this.odometer = odom;
        leftMotor.setAcceleration(MAX_ACCELERATION);
        rightMotor.setAcceleration(MAX_ACCELERATION);
  
    }
  
    @Override
    public void run() {
  
    }
  
    public void travelTo(double x, double y) {
         
        double startX = odometer.getX();
        double startY = odometer.getY();
   
        destinationX = x;
        destinationY = y;
  
        // until checkpoints are reached keep calculating theta and moving
        // motors forward
  
  
            double theta;
  
            double deltaX = destinationX - (odometer.getX());
            double deltaY = destinationY - (odometer.getY());
  
            theta = (Math.atan2(deltaX, deltaY) * (180 / Math.PI));
            // convert theta to be 0-360 degrees increasing clockwise
            if (theta < 0) {
                theta = theta + 360;
            }
  
            turnTo(theta);
 
              
            double distance = Math.pow(Math.pow(deltaX, 2)+Math.pow(deltaY, 2), 0.5);
             
             leftMotor.setSpeed(FORWARD_SPEED);
             rightMotor.setSpeed(FORWARD_SPEED);
             
             while((Math.pow(Math.pow((odometer.getX() - startX), 2)+Math.pow((odometer.getY() - startY), 2), 0.5)<distance)){
               
    
            	 leftMotor.setSpeed(FORWARD_SPEED);
                 rightMotor.setSpeed(FORWARD_SPEED);
                 leftMotor.forward();
                 rightMotor.forward();
                 if(Math.abs(Math.toDegrees(odometer.getTheta()))-theta>5){
                	 turnTo(theta);
                 }
                 leftMotor.setSpeed(FORWARD_SPEED);
                 rightMotor.setSpeed(FORWARD_SPEED);
                 leftMotor.forward();
                 rightMotor.forward();
                  
             }
 
 
             leftMotor.setSpeed(0);
             rightMotor.setSpeed(0);
             
 
  
              
  
          
  
        
  
    }
  
    /**
     * turn to the given orientation
     * @param heading
     */
    public void turnTo(double heading) {
        double degree;
   
        degree = heading - getOdometerTheta();
        // find minimum turning angle
        if (degree > 180) {
            degree = degree - 360;
        }
        leftMotor.setSpeed(ROTATE_SPEED);
        rightMotor.setSpeed(ROTATE_SPEED);
        Motor.A.rotate(convertAngle(wheelRadius, wheelBase, degree),true);
        Motor.B.rotate(-convertAngle(wheelRadius, wheelBase, degree),false);
        // keep rotation until correct heading is reached by checking error
        // between current theta and desired heading
//        while (Math.abs(degree) > 3) {
// 
//            degree = heading - getOdometerTheta();
//            // find minimum turning angle
//            if (degree > 180) {
//                degree = degree - 360;
//            }
// 
//            if (degree < 0.0) {
//                setSpeeds(-ROTATE_SPEED, ROTATE_SPEED);
//            } else if (degree > 180) {
//                setSpeeds(-ROTATE_SPEED, ROTATE_SPEED);
//            } else if (degree < -180) {
//                setSpeeds(ROTATE_SPEED, -ROTATE_SPEED);
//            } else {
//                setSpeeds(ROTATE_SPEED, -ROTATE_SPEED);
//            }
//        }
//        setSpeeds(0, 0);
   
        
   
    }
 
    public double getOdometerTheta() {
        if (odometer.getTheta() * (180 / Math.PI) > 360) {
            return (odometer.getTheta() * (180 / Math.PI)) - 360.0;
        } else if (odometer.getTheta() * (180 / Math.PI) > 720) {
            return (odometer.getTheta() * (180 / Math.PI)) - 720.0;
        } else {
            return (odometer.getTheta() * (180 / Math.PI));
        }
    }
  
 
  
    private static int convertDistance(double radius, double distance) {
        return (int) ((180.0 * distance) / (Math.PI * radius));
    }
  
    private static int convertAngle(double radius, double width, double angle) {
        return convertDistance(radius, Math.PI * width * angle / 360.0);
    }
      
 
      
    //moves robot for distance indicated
    public void move(double distance){
          
        leftMotor.setSpeed(FORWARD_SPEED);
        rightMotor.setSpeed(FORWARD_SPEED);
          
        leftMotor.rotate(convertDistance(wheelRadius, distance), true);
        rightMotor.rotate(convertDistance(wheelRadius, distance), false);
    }
          
          
      
  
}