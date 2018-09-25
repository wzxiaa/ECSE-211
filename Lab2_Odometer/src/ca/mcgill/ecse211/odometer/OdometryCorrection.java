/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import java.awt.Color;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private Odometer odometer;
  private static final int blackValue = 100;
  private static final double TILE_SIZE = 30.48;
  
  //define port for light sensor and color sensor
  private static final Port lightSensorPort = LocalEV3.get().getPort("S1");
  private static final EV3ColorSensor colorSensor = new EV3ColorSensor(lightSensorPort);
  private static final SampleProvider lightSample = colorSensor.getRedMode();
  
  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection() throws OdometerExceptions {

    this.odometer = Odometer.getOdometer();

  }

  /**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;
    
    //initialize counter for number of lines passed in X and Y directions
    int countX = 0;
    int countY = 0;
    
    //make the light sensor on
    colorSensor.setFloodlight(true);
    
    //set up sampler
    int sampleSize = lightSample.sampleSize();
    float[] sample = new float[sampleSize];
    
    while (true) {
      correctionStart = System.currentTimeMillis();

      lightSample.fetchSample(sample,0);
     
	if (sample[0] < 0.3) {
      double data[] = null;
 
      try {
    	data = Odometer.getOdometer().getXYT();
    	
      } catch (OdometerExceptions e1) {
		e1.printStackTrace();
      }
      
      // Beep when black line passed
      Sound.beep();			
      
      //moving upward
      if (data[2] > 345 || data[2] < 15) {
    	  odometer.setY(countY*TILE_SIZE);
    	  countY++;
      }
      
      //moving right
      else if(data[2] > 75 && data[2] < 105) {
    	  odometer.setX(countX*TILE_SIZE);
    	  countX++;
      }
      //moving downward
      else if(data[2] > 165 && data[2] < 195) {
    	  countY--;
    		  odometer.setY(countY*TILE_SIZE);
      }
      //moving left
      else if(data[2] > 255 && data[2] < 285) {
    	  countX--;
    	  odometer.setX(countX*TILE_SIZE);
      }
      
      // Pause thread so that it does not read the same black line twice
      try {
    	  Thread.sleep(500);
      } catch (InterruptedException e) {
    	  e.printStackTrace();
      }
	}

      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
  }
}
