// Lab2.java
package ca.mcgill.ecse211.lab3;


import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab3 {

  // Motor Objects, and Robot related parameters
  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  
  public static final double WHEEL_RAD = 2.2;
  public static final double TRACK = 16.7;
  
  public static final int bandCenter = 5;
  public static final int bandWidth = 5;
  
  private static final int motorLow = 100; // Speed of slower rotating wheel (deg/sec)
  private static final int motorHigh = 200; // Speed of the faster rotating wheel (deg/seec)
  
  //waypoints to be used
  private static int waypoints[][] = new int[][] { { 0, 2 }, { 1, 1 }, { 2, 2 }, { 2, 1 }, { 1, 0 } };
  private static int waypoints2[][] = new int[][] { { 1, 1 }, { 0, 2 }, { 2, 2 }, { 2, 1 }, { 1, 0 } };
  private static int waypoints3[][] = new int[][] { { 1, 0 }, { 2, 1 }, { 2, 2 }, { 0, 2 }, { 1, 1 } };
  private static int waypoints4[][] = new int[][] { { 0, 1 }, { 1, 2 }, { 1, 0 }, { 2, 1 }, { 2, 2 } };
  
  public static void main(String[] args) throws OdometerExceptions {

    int buttonChoice;

    // Odometer related objects
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); 
    Display odometryDisplay = new Display(lcd); // No need to change
	
    do {
      // clear the display
      lcd.clear();

      // ask the user whether the robot should avoid or not
      lcd.drawString("< Left | Right >", 0, 0);
      lcd.drawString("       |        ", 0, 1);
      lcd.drawString(" Avoid | Drive  ", 0, 2);
      lcd.drawString(" Block |		", 0, 3);
      lcd.drawString("       |		", 0, 4);
      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
      while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
		
      //start navigation, odometer and odometerDisplay thread
      if (buttonChoice == Button.ID_RIGHT) {
    	  Thread odoThread = new Thread(odometer);
    	  odoThread.start();
    	  Thread odoDisplayThread = new Thread(odometryDisplay);
		  odoDisplayThread.start();

      	  (new Thread() {
      		  public void run() {
      			  Navigation nav = new Navigation(leftMotor, rightMotor, WHEEL_RAD, TRACK);
      			  for (int i = 0; i < 5; i++) {
      				  nav.travelTo(waypoints2[i][0], waypoints3[i][1]);
      			  }	
      		  }
      	  }).start(); 
      } 
      
      //start avoidance, odometer and odometerDisplay thread
      else if (buttonChoice == Button.ID_LEFT) {
    	  
    	  Thread odoThread = new Thread(odometer);
    	  odoThread.start();
    	  
    	  Thread odoDisplayThread = new Thread(odometryDisplay);
		  odoDisplayThread.start();
		  
		  Avoidance avoid = new Avoidance(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		  Thread avoidance = new Thread(avoid);
		  avoidance.start();
      }
    }
      
    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
}
