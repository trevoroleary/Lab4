package ca.mcgill.ecse211.lab4;

/**
 * This program allows the robot to navigate around a coordinate system
 * 
 * @author Trevor Oleary 
 * @author Winnie Nyakundi
 * @version 1.0
 * @since 2018-02-03
 */

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab4 {

  // Motor Objects, and Robot related parameters
  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  private static final Port usPort = LocalEV3.get().getPort("S1");
  private static final Port ltPort = LocalEV3.get().getPort("S2");
  
  public static final double WHEEL_RAD = 2.15;
  public static final double TRACK = 10.2;

  public static void main(String[] args) throws OdometerExceptions {
	
	  
	  
	SensorModes usSensor = new EV3UltrasonicSensor(usPort);
	SampleProvider usDistance = usSensor.getMode("Distance");
	float[] usData = new float[usDistance.sampleSize()];
	
	SensorModes cSensor = new EV3ColorSensor(ltPort);
	SampleProvider cColor = cSensor.getMode("Red");
	float[] cData = new float[cColor.sampleSize()];
	
	
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); 
    
    // Initializing the navigation object and giving it all the required arguments
    final Localizer localizer = new Localizer(leftMotor, rightMotor, WHEEL_RAD, WHEEL_RAD, TRACK, odometer, usDistance, usData, cColor, cData);

    Display odometryDisplay = new Display(lcd); // No need to change
    
    do {
      // clear the display
      lcd.clear();

      lcd.drawString("   Press  Any   ", 0, 1);
      lcd.drawString("     Button     ", 0, 2);

      Button.waitForAnyPress();    
      
      Thread odoThread = new Thread(odometer);
      odoThread.start();
      Thread odoDisplayThread = new Thread(odometryDisplay);
      odoDisplayThread.start();

      /*
       * Place the coordinates you want to travel here
       */
      new Thread() {
        public void run() {
        	//localizer.printColor();
        	
        	//localizer.printDistance();
        	
        	double[][] scanData = localizer.getScan();
        
        	
        	for(int i = 0; i< 360; i++) {
        		System.out.println(scanData[i][1]);
        	}
        	
        	System.out.println("NEXT SECTION");
        	int[] edges = localizer.getEdges();
        	System.out.println("Falling: " + edges[0] + ", Rising: " + edges[1]);
        	
        	localizer.setHeading(edges);
        	
        	
       
        	
        	//localizer.setHeading();
        	
//        	
//        	for(int i = 0; i< 360; i++) {
//        		System.out.println(scanData[i][1]);
//        	}
        	
//			localizer.travelTo(1,1);
//			localizer.travelTo(0, 2);
//			localizer.travelTo(2,2);
//			localizer.travelTo(2,1);
//			localizer.travelTo(1, 0);
        }
      }.start();
    }
    
    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
}
