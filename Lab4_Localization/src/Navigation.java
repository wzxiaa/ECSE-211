//package ca.mcgill.ecse211.lab4;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Navigation implements UltrasonicController {
	
	private static ColorDetector colorDetector;
	private static Odometer odometer;
	public static int distance;
	private static EV3LargeRegulatedMotor leftMotor;
	private static EV3LargeRegulatedMotor rightMotor;
	private Object lock;
	
	private static final int FORWARD_SPEED = 150;
	private static final int LOW_SPEED = 50;
	private static final int ROTATE_SPEED = 100;
	
	public static double[][] points;
	
	private static double prevAngle = 0;
	
	public static int iterator = 0;
	
	private int filterControl = 0;
	private static final int FILTER_OUT = 40;
	
	final TextLCD display = LocalEV3.get().getTextLCD();
	
	public static boolean foundTargetedRing = false;
	
	private static boolean reachedTargetPoint = false;

	// Define motor specifications for the navigation object
	public Navigation(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, ColorDetector colorDetector ) {
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.colorDetector = colorDetector;
		this.lock = new Object();
	}
	
	//This method causes the robot to travel to the absolute field location (x, y), specified in tile points
	public static void travelTo(double x, double y) {
		double dFromX, dFromY, theta, resizeX, resizeY;
		
		//isNavigating = true; // set navigating to true
		resizeX = Lab5.TILE_SIZE * x; // scale the X value from the target tile x value into cm
		resizeY = Lab5.TILE_SIZE * y; // scale the Y value from the target tile y value into cm

		double[] odometerData = odometer.getXYT();

		dFromX = resizeX - odometerData[0]; // distance from targeted x coordinate to the current x coordinate

		dFromY = resizeY - odometerData[1]; // distance from targeted y coordinate to the current y coordinate

		theta = Math.atan2(dFromX, dFromY); // theta returned in radians

		if (theta < 0) {
			theta = 2 * Math.PI + theta; // we want to keep the radians as positive values for simplicity
		}
		
		turnTo(theta); // make robot turn to the required angle

		double dFromTarget = Math.hypot(Math.abs(dFromX), Math.abs(dFromY)); // find how far the robot is from its
																				// target
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(Lab5.wheelRadius, dFromTarget+5), true);
		rightMotor.rotate(convertDistance(Lab5.wheelRadius, dFromTarget+5), true);

		
		while(isNavigating()) {
			if(distance<= 10) {	//the robot is too close to the wall 

				leftMotor.setSpeed(LOW_SPEED);
				rightMotor.setSpeed(LOW_SPEED);
				
				//colorDetector.performColorDetection();
				
				leftMotor.rotate(convertDistance(Lab5.wheelRadius, 4), true);	//travel a certain distance
				rightMotor.rotate(convertDistance(Lab5.wheelRadius, 4), false); //travel a certain distance
				
				leftMotor.rotate(convertDistance(Lab5.wheelRadius, -1), true);	//travel a certain distance
				rightMotor.rotate(convertDistance(Lab5.wheelRadius, -1), false); //travel a certain distance
				
				colorDetector.performColorDetection();
				
				leftMotor.rotate(convertAngle(Lab5.wheelRadius, Lab5.axleWidth, 90), true);  //turn 90 degree right when facing obstacle
				rightMotor.rotate(-convertAngle(Lab5.wheelRadius, Lab5.axleWidth, 90), false);
				
				leftMotor.setSpeed(FORWARD_SPEED);
				rightMotor.setSpeed(FORWARD_SPEED);
					
				leftMotor.rotate(convertDistance(Lab5.wheelRadius, 15), true);	//travel a certain distance
				rightMotor.rotate(convertDistance(Lab5.wheelRadius, 15), false); //travel a certain distance
				
				leftMotor.rotate(-convertAngle(Lab5.wheelRadius, Lab5.axleWidth, 90), true);  //turn 90 degree right when facing obstacle
				rightMotor.rotate(convertAngle(Lab5.wheelRadius, Lab5.axleWidth, 90), false);
				
				leftMotor.rotate(convertDistance(Lab5.wheelRadius, 37), true);	//travel a certain distance
				rightMotor.rotate(convertDistance(Lab5.wheelRadius, 37), false); //travel a certain distance
				
				leftMotor.rotate(-convertAngle(Lab5.wheelRadius, Lab5.axleWidth, 90), true);  //turn 90 degree right when facing obstacle
				rightMotor.rotate(convertAngle(Lab5.wheelRadius, Lab5.axleWidth, 90), false);
				
				leftMotor.rotate(convertDistance(Lab5.wheelRadius, 15), true);	//travel a certain distance
				rightMotor.rotate(convertDistance(Lab5.wheelRadius, 15), false); //travel a certain distance
				
				leftMotor.rotate(convertAngle(Lab5.wheelRadius, Lab5.axleWidth, 90), true);  //turn 90 degree right when facing obstacle
				rightMotor.rotate(-convertAngle(Lab5.wheelRadius, Lab5.axleWidth, 90), false);
				
				iterator--;	
				if(foundTargetedRing) {
					reachedTargetPoint = false;
				}	
			} else {
				reachedTargetPoint = true;
			}
			
		}
	}
	
	//This method causes the robot to turn (on point) to the absolute heading theta
	public static void turnTo(double theta) {
		//isNavigating = true; // set navigation to true
		double[] odometerData = odometer.getXYT(); // get data from odometer
		double delTheta = odometerData[2]; // get the theta value of odometer
		double turningAngle = theta - delTheta; // calculate the absolute turning angle based on the measured theta
												// based on coordinates and the current theta value of the robot
		if (turningAngle < 0) { // again, if the turning angle is negative, we want it to obtain its positive
								// equivalent
			turningAngle = 2 * (Math.PI) - Math.abs(turningAngle);
		}
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		if (turningAngle > Math.PI) { // turn left if the turning angle ranges from (181-360) degrees to perform					// smallest turn
			turningAngle = (2 * Math.PI) - turningAngle;
			leftMotor.rotate(-convertAngle(Lab5.wheelRadius, Lab5.axleWidth, Math.toDegrees(turningAngle)), true);
			rightMotor.rotate(convertAngle(Lab5.wheelRadius, Lab5.axleWidth, Math.toDegrees(turningAngle)), false);
		} else { // turn right if the turning angle ranges from (0-180) degrees to perform
					// smallest turn
			leftMotor.rotate(convertAngle(Lab5.wheelRadius, Lab5.axleWidth, Math.toDegrees(turningAngle)), true); // convert
																											// turning
																											// angle
																											// into
																											// degrees
			rightMotor.rotate(-convertAngle(Lab5.wheelRadius, Lab5.axleWidth, Math.toDegrees(turningAngle)), false); // convert
																											// turnin																							// angle
		}
	}
	
	public void run() {
		leftMotor.stop();
		rightMotor.stop();
		
		leftMotor.setAcceleration(300);
		rightMotor.setAcceleration(300);
		
		points = setUpSearchMap(Lab5.waypoints[0][0], Lab5.waypoints[0][1], Lab5.waypoints[1][0], Lab5.waypoints[1][1]);
		
		try {
			Thread.sleep(1500);
		} catch (Exception e) {
			//do nothing
		}
		travelTo(Lab5.waypoints[0][0], Lab5.waypoints[0][1]);		//travel to (LLx, LLy) from (1,1)
		// start search the map for target ring until the targeted ring is found
		while(iterator < points.length && !foundTargetedRing) {
			if(foundTargetedRing) {
				break;
			}
			travelTo(points[iterator][0], points[iterator][1]);
			iterator++;
		}
		while(!reachedTargetPoint) {
			travelTo(Lab5.waypoints[1][0], Lab5.waypoints[1][1]);
		}
	}

	//This method returns true if another thread has called travelTo() or turnTo() and the method has yet to return; false otherwise
	//static boolean navigating = false;

	private static boolean isNavigating() {
		if((leftMotor.isMoving() || rightMotor.isMoving()))
			return true;
		else 
			return false;
	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	  public static double[][] setUpSearchMap(int LLx, int LLy, int URx, int URy ) {
	      int size = (URx - LLx + 1) * 2 -1;
	      double[][] map = new double[size][2];
	      map[size-1][0] = URx;
	      map[size-1][1] = URy;
	      for (int i=0,j=0,k=1; i<size && i+1<size; i+=2){
	          map[i][0] = (LLx+j);
	          j++;
	          map[i+1][0] = (LLx + j);
	          if(k%2 == 0) {
	              map[i][1] = LLy;
	              map[i+1][1] = LLy;
	          } else {
	              map[i][1] = URy;
	              map[i+1][1] = URy;
	          }
	          k++;
	      }
	      return map;
	  }

	@Override
	public void processUSData(int distance) {
		// rudimentary filter - toss out invalid samples corresponding to null signal 
	    if (distance >= 255 && filterControl < FILTER_OUT) {
	      // bad value: do not set the distance var, do increment the filter value
	      this.filterControl++;
	    } else if (distance >= 255) {
	      // We have repeated large values, so there must actually be nothing
	      // there: leave the distance alone
	      this.distance = distance;
	    } else {
	      // distance went below 255: reset filter and leave
	      // distance alone.
	      this.filterControl = 0;
	      this.distance = distance;
	    }
	    display.drawString("                    ", 0, 6);
	    display.drawString("US: " + this.distance, 0, 6);
	}

	@Override
	public int readUSDistance() {
		// TODO Auto-generated method stub
		return this.distance;
	}
}

	
