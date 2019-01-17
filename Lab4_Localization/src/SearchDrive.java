/*
 * SquareDriver.java
 */

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class is used to drive the robot on the demo floor.
 */
public class SearchDrive implements UltrasonicController {
  private static final int FORWARD_SPEED = 250;
  private static final int ROTATE_SPEED = 150;
  private static final double TILE_SIZE = 30.48;
  
  private static EV3LargeRegulatedMotor leftMotor = Lab5.leftMotor;
  private static EV3LargeRegulatedMotor rightMotor = Lab5.rightMotor;

  
	private int distance;
  
	private int filterControl = 0;
	private static final int FILTER_OUT = 35;
	private int d = 40;	// drop-off point
	private int k = 1;	// error margin
	
	TextLCD lcd = LocalEV3.get().getTextLCD();
	
	private static int[][] points = {};
	

  /**
   * This method is meant to drive the robot in a square of size 2x2 Tiles. It is to run in parallel
   * with the odometer and Odometer correcton classes allow testing their functionality.
   *
   */
  
  public static void search() {
	  
    // reset the motors
	  leftMotor.setAcceleration(3000);
	  rightMotor.setAcceleration(3000);
	  
	  leftMotor.setSpeed(100);
	  rightMotor.setSpeed(100);
	  
	  points = setUpSearchMap(Lab5.waypoints[0][0],Lab5.waypoints[0][1],Lab5.waypoints[1][0],Lab5.waypoints[1][1]);
	  
	  
	  
	  
   }


  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  public void run() {
	  	//correct the heading to point forward
		leftMotor.rotate(-convertAngle(Lab5.wheelRadius, Lab5.axleWidth, 45), true);
		rightMotor.rotate(convertAngle(Lab5.wheelRadius, Lab5.axleWidth, 45), false);
		
		
		
  }
  
  public static int[][] setUpSearchMap(int LLx, int LLy, int URx, int URy ) {
      int size = (URx - LLx + 1) * 2 -1;
      int[][] map = new int[size][2];
      map[size-1][0] = URx;
      map[size-1][1] = URy;
      for (int i=0,j=0,k=1; i<size && i+1<size; i+=2){
          map[i][0] = LLx+j;
          j++;
          map[i+1][0] = LLx + j;
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
  
  
  
  
  
  
  
  
	
	/**
	 * Filters out invalid ultrasonic (US) samples, prints US reading to screen.
	 * 
	 * @param distance - the reading of the ultrasonic sensor.
	 */
	@Override
  public void processUSData (int distance) {
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
	    
	    lcd.drawString("                    ", 0, 6);
	    lcd.drawString("US: " + this.distance, 0, 6);
	}
  
  
	/**
	 * @return ultrasonic sensor reading
	 */
	@Override
	public int readUSDistance() {
	  return this.distance;
	}
  
  
  
  /**
   * This method allows the conversion of a distance to the total rotation of each wheel need to
   * cover that distance.
   * 
   * @param radius
   * @param distance
   * @return
   */
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
}
