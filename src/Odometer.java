import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
 
/*
 * Odometer.java
 */
 
public class Odometer extends Thread {
    // robot position
    private double x, y, theta;
    private int tachoCountL;
    private int tachoCountR;
 
    // odometer update period, in ms
    private static final long ODOMETER_PERIOD = 25;
 
    // lock object for mutual exclusion
    private final Object lock;
    private final NXTRegulatedMotor leftMotor = Motor.A, rightMotor = Motor.B;
 
    // hard code wheel base and wheel radius
    private final double wheelRadius = 1.621;
    private final double wheelBase = 13.1701;
 
    // default constructor
    public Odometer() {
        // initial position is origin, pointing along y axis with theta as 0
        x = 0.0;
        y = 0.0;
        theta = 0.0;
        lock = new Object();
 
        // initial tachometer counts should be 0
        leftMotor.resetTachoCount();
        rightMotor.resetTachoCount();
        leftMotor.flt();
        rightMotor.flt();
        // set last tachometer readings as the initial tachometer readings
        tachoCountL = leftMotor.getTachoCount();
        tachoCountR = rightMotor.getTachoCount();
    }
 
    // run method (required for Thread)
    @Override
    public void run() {
        long updateStart, updateEnd;
 
        while (true) {
            updateStart = System.currentTimeMillis();
            // change in tachometer readings is current tacho count minus last
            int deltaTachoCountL = leftMotor.getTachoCount() - tachoCountL;
            int deltaTachoCountR = rightMotor.getTachoCount() - tachoCountR;
            // set last tachometer readings
            tachoCountR = rightMotor.getTachoCount();
            tachoCountL = leftMotor.getTachoCount();
            double deltaDLeft = ((wheelRadius * (Math.PI) * deltaTachoCountL) / 180);
            double deltaDRight = ((wheelRadius * (Math.PI) * deltaTachoCountR) / 180);
            double deltaD = (deltaDRight + deltaDLeft) / 2;
            double deltaTheta = (deltaDLeft - deltaDRight) / wheelBase;
 
            // set x and y only in synchronized block
            synchronized (lock) {
                this.theta = this.theta + deltaTheta;
                this.x = x + (deltaD * Math.sin(this.theta));
                this.y = y + (deltaD * Math.cos(this.theta));
            }
 
            updateEnd = System.currentTimeMillis();
            if (updateEnd - updateStart < ODOMETER_PERIOD) {
                try {
                    Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
                } catch (InterruptedException e) {
                    // there is nothing to be done here because it is not
                    // expected that the odometer will be interrupted by
                    // another thread
                }
            }
        }
    }
 
    // accessors
    public void getPosition(double[] position, boolean[] update) {
        // ensure that the values don't change while the odometer is running
        synchronized (lock) {
            if (update[0])
                position[0] = x;
            if (update[1])
                position[1] = y;
            if (update[2])
                position[2] = theta;
        }
    }
 
    public double getX() {
        double result;
 
        synchronized (lock) {
            result = x;
        }
 
        return result;
    }
 
    public double getY() {
        double result;
 
        synchronized (lock) {
            result = y;
        }
 
        return result;
    }
 
    public double getTheta() {
        double result;
 
        synchronized (lock) {
            result = theta;
        }
 
        return result;
    }
 
    // mutators
    public void setPosition(double[] position, boolean[] update) {
        // ensure that the values don't change while the odometer is running
        synchronized (lock) {
            if (update[0])
                x = position[0];
            if (update[1])
                y = position[1];
            if (update[2])
                theta = position[2];
        }
    }
 
    public void setX(double x) {
        synchronized (lock) {
            this.x = x;
        }
    }
 
    public void setY(double y) {
        synchronized (lock) {
            this.y = y;
        }
    }
 
    public void setTheta(double theta) {
        synchronized (lock) {
            this.theta = theta;
        }
    }
 
    public void getPosition(double[] pos) {
        synchronized (lock) {
            pos[0] = x;
            pos[1] = y;
            pos[2] = theta;
        }
    }
}