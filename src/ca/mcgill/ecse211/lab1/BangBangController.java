package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

  private final int bandCenter;
  private final int bandwidth;
  private final int motorLow;
  private final int motorHigh;
  private int distance;

  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {
    this.distance = distance;
    // TODO: process a movement based on the us distance passed in (BANG-BANG style)
    
    //Amount by which the motors decrease their speed
    int deltaSpeed = 190;
    
    //difference between the distance from the wall and the bandCenter set in the main class
    int error = bandCenter - distance;
    
    //If the error is not significant enough, move straight forward
    if(Math.abs(error) <= bandwidth ) {
    	 WallFollowingLab.leftMotor.setSpeed(motorHigh);
    	 WallFollowingLab.rightMotor.setSpeed(motorHigh);
    } 
    
    //Too far away from the wall, turn left, decrease left motor speed by a set amount deltaSpeed, increase right motor speed
    else if(error < 0) {
   	 	WallFollowingLab.leftMotor.setSpeed(motorHigh - deltaSpeed);
   	 	WallFollowingLab.rightMotor.setSpeed(motorHigh * 2);
    }
    //Too close to the wall, turn right, decrease right motor speed by a set amount deltaSpeed, increase left motor speed
    else if(error > 0) {
   	 	WallFollowingLab.rightMotor.setSpeed(motorHigh - deltaSpeed);
   	 	WallFollowingLab.leftMotor.setSpeed(motorHigh * 2);
    }
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
}
