
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class ColorDetector {
	
	  private static final Port lightPort = LocalEV3.get().getPort("S3");
	  private SensorModes lightSensor = new EV3ColorSensor(lightPort);
	  private SampleProvider colorSample = lightSensor.getMode("RGB");			// provides samples from this instance
	  private float[] colorvalues = new float[colorSample.sampleSize()];			// buffer in which data are returned
	  private static final TextLCD lcd = LocalEV3.get().getTextLCD();

	  double thetaX1, thetaX2, thetaY1, thetaY2;
	  
	  private Odometer odometer;
	  
	  private static Color currentColor;
	  private static Color targetColor;
	  
	  private static boolean isDetected;
	  //private Navigation nav;
	  
	  //private SampleProvider colorSample;
	  private double previousBrightness;
	  private double currentBrightness;
	  //private float[] colorvalues;
	 
	  public static enum Color {
		  Orange, Blue, Yellow, Green, Other;
	  }
	 
	  //Navigation nav
	  public ColorDetector(Odometer odo, Color inputColor) {
		  this.odometer = odo;
		  this.targetColor = inputColor;
		  this.isDetected = false;
	  }
	
	  public void performColorDetection() {
		  while (!isDetected){
			ColorDisplay();
		  }	
		  if (currentColor == targetColor) {
			  Sound.twoBeeps();
		      lcd.drawString("Color: " + currentColor,0, 5);
		      Navigation.foundTargetedRing = true;
		  } else {
			  Sound.beep();
			  lcd.drawString("Color: " + currentColor,0, 5);
		  }
		   isDetected = false;
	  }
	  
	  private void ColorDisplay() {
		  // Trigger correction (When do I have information to correct?)
	      colorSample.fetchSample(colorvalues, 0);
	      currentBrightness = (colorvalues[0] + colorvalues[1] + colorvalues[2])*100;
	      
	      if(previousBrightness == -1) { //no set brightness
	    	  previousBrightness = currentBrightness;
	      }
	      
	      lcd.drawString("0: " +colorvalues[0]*100,0, 1);
	      lcd.drawString("1: " +colorvalues[1]*100,0, 2);
	      lcd.drawString("2: " +colorvalues[2]*100,0, 3);
	      lcd.drawString("Co: " +currentBrightness,0, 4);
	      lcd.drawString("Color: " + currentColor,0, 5);
	     
	      double R = colorvalues[0]*100;
	      double G = colorvalues[1]*100;
	      double B = colorvalues[2]*100;
	      
	      currentColor = getColor(R, G, B);

	      
	      /*
	      double Rn = R/Math.sqrt(Math.pow(R,2) + Math.pow(G,2) + Math.pow(B,2));
	      double Gn = G/Math.sqrt(Math.pow(R,2) + Math.pow(G,2) + Math.pow(B,2));
	      double Bn = B/Math.sqrt(Math.pow(R,2) + Math.pow(G,2) + Math.pow(B,2));
	      
	      if (currentBrightness > 20 && currentBrightness <= 22) {//blue
	    	  clearDisplay();
	    	  lcd.drawString("OBJECT DETECTED", 0, 5);
	    	  lcd.drawString("BLUE RING", 0, 6);
	    	  currentColor = Color.Blue;
	    	  isDetected = true;
	      }
	      if (currentBrightness > 22 && currentBrightness <= 30) {//green
	    	  clearDisplay();
	    	  lcd.drawString("OBJECT DETECTED", 0, 5);
	    	  lcd.drawString("GREEN RING", 0, 6);
	    	  currentColor = Color.Green;
	    	  isDetected = true;
	      }
	      if (currentBrightness > 31) {//yellow
	    	  clearDisplay();
	    	  lcd.drawString("OBJECT DETECTED", 0, 5);
	    	  lcd.drawString("YELLOW RING", 0, 6);
	    	  currentColor = Color.Yellow;
	    	  isDetected = true;
	      }
	      if (currentBrightness > 11 && currentBrightness <= 17) {//orange
	    	  clearDisplay();
	    	  lcd.drawString("OBJECT DETECTED", 0, 5);
	    	  lcd.drawString("ORANGE RING", 0, 6);
	    	  currentColor = Color.Orange;
	    	  isDetected = true;
	      }
	      if (currentBrightness <= 10) {
	    	  //lcd.drawString("NOT DETECTED", 0, 6);
	    	  clearDisplay();
	      }*/

	      int buttonChoice;
			buttonChoice = Button.readButtons();
			if (buttonChoice == Button.ID_ESCAPE) {
				System.exit(0);
			}
			
	  }
	  
	  //green: Rmin:1.4 Rmax:3.7	Gmin:5.1 Gmax:13	Bmin:0.8 Bmax:1.5
	  //Blue: Rmin:0.5 Rmax:3.9	Gmin:5.0 Gmax:13  Bmin: 2.7 Bmax:7
	  //Orange: Rmin:5.3 Rmax:14 Gmin:0.8 Gmax:5  Bmin:0.3 Bmax:0.7
	  //Yellow: Rmin:5 Rmax:19  Gmin:3.5 Gmax:12  Bmin:0.8  Bmax:2
	  
	  public static Color getColor(double r, double g, double b) {
		  if((r>=5 && r<=19) && (g>=3.5 && g<=12) && (b>=0.8 && b<=2)) {
			  currentColor = Color.Yellow;
			  lcd.drawString("YELLOW RING", 0, 6);
	    	  isDetected = true;
		  } else if ((r >= 0.5 && r <= 3.9) && (g >= 5 && g <= 13) && (b >=2.7 && b <= 7)) {
			  currentColor = Color.Blue;
	    	  lcd.drawString("BLUE RING", 0, 6);
	    	  isDetected = true;
		  } else if ((r >= 1.4 && r <= 8) && (g >= 5.1 && g <= 16) && (b >= 0.8 && b <= 1.6)) {
			  currentColor = Color.Green;
	    	  lcd.drawString("GREEN RING", 0, 6);
	    	  isDetected = true;
		  } else if ((r >= 5.3 && r <= 14) && (g >= 0.8 && g <= 5) && (b >= 0.3 && b<= 0.9)) {
			  currentColor = Color.Orange;
			  lcd.drawString("ORANGE RING", 0, 6);
			  isDetected = true;
		  } else {
			  lcd.drawString("Not Detected", 0, 6);
			  currentColor = Color.Other;
		  }
		  return currentColor;
	  }
	  
		public static Color getColor() {
			if (currentColor != null) return currentColor;
			else return Color.Other;
		}
	  
	  public void clearDisplay() {
		  lcd.clear(5);
		  lcd.clear(6);
	  }
}