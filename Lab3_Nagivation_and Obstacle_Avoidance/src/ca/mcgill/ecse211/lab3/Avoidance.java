package ca.mcgill.ecse211.lab3;
import lejos.hardware.sensor.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;

import lejos.hardware.Button;

public class Avoidance implements Runnable {

	//set fields
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private float[] usData;
	private SampleProvider usDistance ;
	private final double TRACK;
	private final double WHEEL_RAD;
	public static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	double currentT, currentY, currentX;
	double dx, dy, dt;
	double distanceToTravel;
	int iterator = 0;
	private Odometer odometer;
	private OdometerData odoData;
	private double[][]  wayPoints = new double[][]{{1*30.48,1*30.48}, // Set the array points
		{0*30.48,2*30.48},
		{2*30.48,2*30.48},
		{2*30.48,1*30.48},
		{1*30.48,0*30.48}};

		//constructor
		public Avoidance(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			final double TRACK, final double WHEEL_RAD) throws OdometerExceptions { 
			this.odometer = Odometer.getOdometer();
			this.leftMotor = leftMotor;
			this.rightMotor = rightMotor;
			odoData = OdometerData.getOdometerData();
			odoData.setXYT(0 , 0 , 0);
			this.TRACK = TRACK;
			this.WHEEL_RAD = WHEEL_RAD;
			SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
			usDistance = usSensor.getMode("Distance"); // usDistance provides samples from this instance
			this.usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are returned
		}

		
		public void run() {
			for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
				motor.stop();
				motor.setAcceleration(300);  // reduced the acceleration to make it smooth
			}
			// wait 2 seconds
			try {
				Thread.sleep(2000);
			} catch (InterruptedException e) {
				//nothing to be done
			}
			//iterate through all the points 
			while(iterator < wayPoints.length) {
				travelTo(wayPoints[iterator][0], wayPoints[iterator][1]);
				iterator++;
			}
		}

		//Navigate to the (x,y) 
		void travelTo(double x, double y) {
			currentX = odometer.getXYT()[0]; //get position x
			currentY = odometer.getXYT()[1]; //get position y
			currentT = odometer.getXYT()[2]; 

			dx = x- currentX;	//get the difference in x 
			dy = y - currentY;  //get the differences in y
			distanceToTravel = Math.sqrt(dx*dx+dy*dy);	//calculate the distance to be traveled
			if(dy>=0) {
				dt=Math.atan(dx/dy);	//calculate the angle to turn
			}
			else if(dy<=0&&dx>=0) {
				dt=Math.atan(dx/dy)+Math.PI;
			}
			else {
				dt=Math.atan(dx/dy)-Math.PI;
			}

			double differenceInTheta = (dt*180/Math.PI-currentT);  //robot has to turn "differenceInTheta",
			
			turnTo(differenceInTheta); //turn the robot in desired angle

			// drive forward required distance
			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);
			leftMotor.rotate(convertDistance(WHEEL_RAD, distanceToTravel), true);
			rightMotor.rotate(convertDistance(WHEEL_RAD, distanceToTravel), true);

			//avoiding the obstacles
			while(isNavigating()) {
				usDistance.fetchSample(usData,0);
				float distance = usData[0]*100;	
				if(distance<= 15) {	//the robot is too close to the wall 
					if(odometer.getXYT()[0]<2.4*30.48&&odometer.getXYT()[0]>1.3*30.48&&odometer.getXYT()[1]<2.5*30.48&&odometer.getXYT()[1]>1.6*30.48){ //decide whether the robot is turning left or right
						leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), true);  //turn 90 degree when facing obstacle
						rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), false);
						leftMotor.rotate(convertDistance(WHEEL_RAD, 30), true);	//travel a certain distance
						rightMotor.rotate(convertDistance(WHEEL_RAD, 30), false); //travel a certain distance
						leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), true);	//turn 90 degree again 
						rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), false);//turn 90 degree again
						leftMotor.rotate(convertDistance(WHEEL_RAD, 40), true); //travel forward along the obstacle
						rightMotor.rotate(convertDistance(WHEEL_RAD, 40), false); //travel forward along the obstacle
					}
					else {
					leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), true);  	//turn 90 degree when facing obstacle
					rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), false); 	//turn 90 degree when facing obstacle
					leftMotor.rotate(convertDistance(WHEEL_RAD, 30), true); 	//travel a certain distance
					rightMotor.rotate(convertDistance(WHEEL_RAD, 30), false);	//travel a certain distance
					leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), true);  	//turn 90 degree again
					rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), false);  	//turn 90 degree again
					leftMotor.rotate(convertDistance(WHEEL_RAD, 40), true);		//travel forward along the obstacle
					rightMotor.rotate(convertDistance(WHEEL_RAD, 40), false);	//travel forward along the obstacle
					}
					iterator--;
				}
			}
		}

		//method for turning the robot to desired angle towards waypoints
		void turnTo(double theta) {
			//find the minimum angle
			if(theta>180) { 
				theta=360-theta;
				leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);
				leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), true);	
				rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), false);
			}
			else if(theta<-180) {
				theta=360+theta;
				leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);
				leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
				rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);
			}
			else {
				leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);
				leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
				rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);	
			}
		}

		//determine if the robot is travelling
		boolean isNavigating() {
			if((leftMotor.isMoving() || rightMotor.isMoving()))
				return true;
			else 
				return false;

		}
		
		//convert the distance 
		private static int convertDistance(double radius, double distance) {
			return (int) ((180.0 * distance) / (Math.PI * radius));
		}
		//convert the angle
		private static int convertAngle(double radius, double width, double angle) {
			return convertDistance(radius, Math.PI * width * angle / 360.0);
		}
}