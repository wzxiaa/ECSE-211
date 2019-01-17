
import lejos.hardware.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;


public class Lab5 {
	// setting up ports for motors
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	// setting up track length and wheel radius for the robot
	public static final double axleWidth = 11.8, wheelRadius = 2.2;
	public static final double TILE_SIZE = 31.48;
	// setting up waupoints used for the search. It corresponds to {[LLx, LLy],[URx, URy]}
	public static int waypoints[][] = new int[][]{{1, 1}, {5,5}};
	// specifying the color of the ring to be detected
	public static final ColorDetector.Color targetColor = ColorDetector.Color.Orange; 
	// setting up Ultrasonic port
	public static final Port usPort = LocalEV3.get().getPort("S1");	
	// an ultrasonic poller instance to be used for the tasks
	public static UltrasonicPoller usPoller;
	public static final TextLCD lcd = LocalEV3.get().getTextLCD();
	

	public static void main(String[] args) {
		// ultrasonic Sensor sensor setup
		@SuppressWarnings("resource")							    	// do not bother to close this resource
		SensorModes usSensor = new EV3UltrasonicSensor(usPort);
		SampleProvider usValue = usSensor.getMode("Distance");			// provides samples from this instance
		float[] usData = new float[usValue.sampleSize()];				// buffer in which data are returned
		
		Odometer odometer = new Odometer(leftMotor, rightMotor);		// setting up odometer
		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, lcd);		// setting up odometer display

		int buttonChoice;
		
		// setting up display
		do {
			lcd.clear();
			lcd.drawString("Start Falling Edge", 0, 0);
			lcd.drawString("Press Left Button", 0, 1);
			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT);
	    	    
		odometer.start();			// start the odometer thread
		odometryDisplay.start();		//start the odometer display thread
		
		//colorDetector.performColorDetection();
	   /*
	    USLocalizer usLoc = new USLocalizer(leftMotor, rightMotor, odometer);		// setting up USLocalizer
	    usPoller = new UltrasonicPoller(usValue, usData, usLoc);		// setting up UltrasonicPoller
		usPoller.start();		//start the UltrasonicPoller thread
		
		usLoc.fallingEdge();		//start ultrasonic localization
		
		do {
			lcd.clear();
			lcd.drawString("Start Light Localization", 0, 0);
			lcd.drawString("Press Left Button", 0, 1);
			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT);
		
		LightLocalizer lightLoc = new LightLocalizer(leftMotor, rightMotor, odometer);
		lightLoc.doLoc();		// start light localization

		do {
			lcd.clear();
			lcd.drawString("Start Search", 0, 0);
			lcd.drawString("Press Left Button", 0, 1);
			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT);
		
		*/
		ColorDetector colorDetector = new ColorDetector(odometer, targetColor);
		
		Navigation nav = new Navigation(odometer, leftMotor, rightMotor, colorDetector);
		usPoller = new UltrasonicPoller(usValue, usData, nav);
		usPoller.start();
	    if (buttonChoice == Button.ID_LEFT) {
				nav.run();
	    }

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);	
	}
}
