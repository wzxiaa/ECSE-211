//package ca.mcgill.ecse211.Lab5;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.hardware.Button;
import lejos.hardware.Sound;

/**
 */
public class LightLocalizer {
	// constants
	private final double f = 14;	// distance between light sensor and wheels
	public static int forwardSpeed = 150;
	private static final int blackValue = 150;
	
	Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	final TextLCD display = LocalEV3.get().getTextLCD();
	
	// color sensor setup
	private static final Port lightPort = LocalEV3.get().getPort("S2");
	SensorModes lightSensor = new EV3ColorSensor(lightPort);
	SampleProvider lightValue = lightSensor.getMode("Red");			// provides samples from this instance
	float[] lightData = new float[lightValue.sampleSize()];			// buffer in which data are returned
	
	/**
	 * Constructor
	 * 
	 * @param leftMotor - robot's left motor
	 * @param rightMotor - robot's right motor
	 * @param odometer - robot's odometer
	 */
	public LightLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, 
			Odometer odometer) {
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	}
	
	/**
	 * Performs position localization.
	 */
	public void doLoc(){
		
	    int buttonChoice;
			buttonChoice = Button.readButtons();
		// set speed and acceleration
		double x, y;
		
		this.leftMotor.setSpeed(forwardSpeed);
		this.rightMotor.setSpeed(forwardSpeed);
		this.leftMotor.setAcceleration(500);
		this.rightMotor.setAcceleration(500);
		
		// move forward until a line is detected
		this.leftMotor.forward();
		this.rightMotor.forward();
		
		// get light value
	    lightValue.fetchSample(lightData, 0);
	    float reading = lightData[0] * 1000;		// scale up for more accuracy
	    display.drawString("Color: " + reading, 0, 4);
	    
		while (reading >= blackValue) {
			// update light value
		    lightValue.fetchSample(lightData, 0);
		    reading = lightData[0] * 1000;		// scale up for more accuracy
		    display.drawString("                  ", 0, 4);
		    display.drawString("Color: " + reading, 0, 4);
		}
		
		y = odometer.getY();
		
		this.leftMotor.setSpeed(0);
		this.rightMotor.setSpeed(0);
		
		Sound.beep();
		
		this.leftMotor.setSpeed(forwardSpeed);
		this.rightMotor.setSpeed(forwardSpeed);
		
		//move back
		this.leftMotor.rotate(convertDistance(Lab5.wheelRadius, -y), true);
		this.rightMotor.rotate(convertDistance(Lab5.wheelRadius, -y), false);
		
		// turn 90deg clockwise
		this.leftMotor.rotate(convertAngle(Lab5.wheelRadius, Lab5.axleWidth, 90), true);
		this.rightMotor.rotate(-convertAngle(Lab5.wheelRadius, Lab5.axleWidth, 90), false);
		
		this.leftMotor.forward();
		this.rightMotor.forward();
		
		// move forward until a line is detected, stop
		// get light value
	    lightValue.fetchSample(lightData, 0);
	    reading = lightData[0] * 1000;		// scale up for more accuracy
	    display.drawString("Color: " + reading, 0, 4);		
		while (reading >= blackValue) {
			// update light value
		    lightValue.fetchSample(lightData, 0);
		    reading = lightData[0] * 1000;		// scale up for more accuracy
		    display.drawString("Color: " + reading, 0, 4);
		}

		x = odometer.getX();
		this.leftMotor.setSpeed(0);
		this.rightMotor.setSpeed(0);
		Sound.beep();
		
		this.leftMotor.setSpeed(forwardSpeed);
		this.rightMotor.setSpeed(forwardSpeed);
		//move back
		this.leftMotor.rotate(convertDistance(Lab5.wheelRadius, -f), true);
		this.rightMotor.rotate(convertDistance(Lab5.wheelRadius, -f), false);
		
		this.leftMotor.setSpeed(0);
		this.rightMotor.setSpeed(0);
		
		this.leftMotor.setSpeed(forwardSpeed);
		this.rightMotor.setSpeed(forwardSpeed);
		
		//turn anticlockwise
		this.leftMotor.rotate(-convertAngle(Lab5.wheelRadius, Lab5.axleWidth, 90), true);
		this.rightMotor.rotate(convertAngle(Lab5.wheelRadius, Lab5.axleWidth, 90), false);
	
		this.leftMotor.setSpeed(0);
		this.rightMotor.setSpeed(0);
		
		this.leftMotor.setSpeed(forwardSpeed);
		this.rightMotor.setSpeed(forwardSpeed);
		
		this.leftMotor.rotate(convertDistance(Lab5.wheelRadius, y-f), true);
		this.rightMotor.rotate(convertDistance(Lab5.wheelRadius, y-f), false);

		// done => zero odometer values
		this.odometer.setX(30.48);
		this.odometer.setY(30.48);
		this.odometer.setTheta(0);

		if (buttonChoice == Button.ID_ESCAPE) {
			System.exit(0);
		}
	}
	
	/*
	public void doLoc2() {
		
		int counter = 0;
		double angleX = 0;
		double angleY = 0;
		
		leftMotor.setSpeed(50);
		rightMotor.setSpeed(50);
		
		leftMotor.rotate(convertAngle(Lab5.wheelRadius, Lab5.axleWidth, 360), true);
		rightMotor.rotate(-convertAngle(Lab5.wheelRadius, Lab5.axleWidth, 360), true);
		
		
		while(leftMotor.isMoving() && rightMotor.isMoving()) {
		    lightValue.fetchSample(lightData, 0);
		    float reading = lightData[0] * 1000;			
			if(reading < blackValue) {
				
			Sound.beep();
			if(counter == 0) {
				angleY = odometer.getTheta();
				counter++;
			} else if (counter == 1) {
				angleX = odometer.getTheta();
				counter++;
			} else if (counter == 2) {
				angleY = odometer.getTheta() - angleY;
				counter++;
			} else if (counter == 3 ) {
				angleX = odometer.getTheta() - angleX;
			}
			}
		}
		
		double y = -18 * Math.cos(angleX / 2);
		double x = -18 * Math.cos(angleY / 2);
		
		leftMotor.rotate(convertDistance(Lab5.axleWidth, y), true);
		rightMotor.rotate(convertDistance(Lab5.axleWidth, y), false);
		leftMotor.rotate(convertAngle(Lab5.axleWidth, Lab5.wheelRadius, 90), true);
		rightMotor.rotate(convertAngle(Lab5.axleWidth, Lab5.wheelRadius, 90), false);
		leftMotor.rotate(convertDistance(Lab5.axleWidth, x), true);
		rightMotor.rotate(convertDistance(Lab5.axleWidth, x), false);
		leftMotor.rotate(convertAngle(Lab5.axleWidth, Lab5.wheelRadius, 90), true);
		rightMotor.rotate(convertAngle(Lab5.axleWidth, Lab5.wheelRadius, 90), false);
		
	}
	*\
	
	
	
	
	
	/**
	 * Converts travel distance to number of degrees of rotation of robot's wheel.
	 * 
	 * @param radius - radius of robot's wheel (cm)
	 * @param distance - distance needed to be traveled (cm)
	 * @return number of degrees of wheel rotation (degrees)
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	/**
	 * Converts robot rotation to number of degrees each wheel must turn (in opposite directions).
	 * 
	 * @param radius - robot wheel radius (cm)
	 * @param width - robot axle width (cm)
	 * @param angle - amount of robot rotation (radians)
	 * @return number of degrees of wheel rotation (degrees)
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
