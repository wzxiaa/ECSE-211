package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 200;
  private static final int FILTER_OUT = 20;

  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl;

  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0;

    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {

    // rudimentary filter - toss out invalid samples corresponding to null
    // signal.
    // (n.b. this was not included in the Bang-bang controller, but easily
    // could have).
    //
    if (distance >= 255 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
    } else if (distance >= 255) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }

    // TODO: process a movement based on the us distance passed in (P style)
    
    this.distance = distance;
    
    //difference between the distance from the wall and the bandCenter set in the main class
    int error = bandCenter - distance;
    
    //set deltaSpeed to be proportional to the error
    int deltaSpeed =  (int)(15*Math.abs(error));
    
    //set the maximum speed allowed for deltaSpeed
    if (deltaSpeed > 400) {
    	deltaSpeed = 400;
    }
    
    //If the error is not significant enough, move straight forward
    if(Math.abs(error) <= bandWidth) {
   	 	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
   	 	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    }
    
    //Too far away from the wall, turn left, decrease left motor speed by an amount deltaSpeed proportional to the error, increase right motor speed
    else if(error < 0) {
		WallFollowingLab.rightMotor.setSpeed(deltaSpeed);
		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - deltaSpeed);
		
    }
    
    //Too far away from the wall, turn left, decrease left motor speed by an amount deltaSpeed proportional to the error, increase right motor speed
    else if(error > 0){
		WallFollowingLab.leftMotor.setSpeed(deltaSpeed);
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED - deltaSpeed);
    }
  }


  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
