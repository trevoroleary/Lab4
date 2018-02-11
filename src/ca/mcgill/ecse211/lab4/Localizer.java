/**
 * This class is an object
 * It is called whenever the main robot needs to get to a specific coordinate
 * 
 * 
 * @author Winnie Nyakundi & Trevor O'Leary
 * @Version 2.0
 * @since 2018-02-03
 */
package ca.mcgill.ecse211.lab4;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerData;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

/**
 * This class is used to drive the robot on the demo floor.
 */
public class Localizer {
	
	private static final int FORWARD_SPEED = 100;
	private static final int ROTATE_SPEED = 25;
	private static final int ACCEL = 500;
	private static final int BUFFER = 10;

	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private double leftRadius, rightRadius, track;
	private Odometer odo;
	private double TILE_SIZE = 30.48;
	private boolean Navigating;
    SampleProvider usDistance, cColor;
    float[] usData, cData;
    double[][] scanData;

  public Localizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      double leftRadius, double rightRadius, double track, Odometer odo, SampleProvider usDistance, float[] usData, SampleProvider cColor, float[] cData) throws OdometerExceptions {
	  
	  this.leftRadius = leftRadius;
	  this.rightRadius = rightRadius;
	  this.leftMotor = leftMotor;
	  this.rightMotor = rightMotor;
	  this.leftRadius = leftRadius;
	  this.rightRadius = rightRadius;
	  this.track = track;
	  this.odo = odo; 
	  this.usDistance = usDistance;
	  this.usData = usData;
	  this.cColor = cColor;
	  this.cData = cData;
	  leftMotor.setAcceleration(ACCEL);
	  rightMotor.setAcceleration(ACCEL);
	  
	  this.scanData = new double[360][2];
	  
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
  
  
  /**
   * This method makes the robot travel to the grid coordinate x, y
   * The coordinates do not need to take into account the size of a tile
   *
   *@param x the x coordinate of the waypoint
   *@param y the y coordinate of the waypoint
   */
  public void travelTo(double x, double y){
	  
	  Navigating = true;
	  
	  int distance;
	  double[] location =  odo.getXYT();
	  x = x*TILE_SIZE;
	  y = y*TILE_SIZE;
	  double dx = x - location[0];
	  double dy = y - location[1];
	  
	  //Calculating the change in angle required to get to our required angle
	  double dTheta = (Math.atan2(dx,dy)*180/3.1415) - location[2];
	  
	  //If the magnitude change is greater than 180 a correction is made
	  if(dTheta > 180) {
		  dTheta = dTheta - 360;
	  }
	  else if(dTheta < -180){
		  dTheta = dTheta + 360;
	  }
	  
	  //Length of hypoteneus used for distance of travel 
	  double hypo = Math.sqrt(Math.pow((x - location[0]),2) + Math.pow((y - location[1]), 2));
	  
	  turnTo(dTheta);
	  
	  leftMotor.setSpeed(FORWARD_SPEED);
	  rightMotor.setSpeed(FORWARD_SPEED);
	  
	  
	  leftMotor.rotate(convertDistance(leftRadius, hypo), true);
	  rightMotor.rotate(convertDistance(rightRadius, hypo), true);
	  
	  
	  /*
	   * while the robot is navigating it is constantly looking for obsicles
	   * or if its wheels stop moving
	   */
	  while(Navigating) {
		  
		  usDistance.fetchSample(usData, 0); // acquire data
	      distance = (int) (usData[0] * 100.0);
		  if(distance < 15) {
			  turnAway(location, x, y);
		  }
		  
		  if(!leftMotor.isMoving() && !rightMotor.isMoving()) {
			  Navigating = false;
		  }
	  }
  }
  
  /**
   * This method figures out the correct way to turn away depending on its location and heading
   * 
   * @param location gives this method the necessary location information
   * @param x & y allows this method to recall travelTo when its done
   */
  public void turnAway(double[] location, double x, double y) {
	  
	  Navigating = false;
	  
	  
	  /*
	   * Treats the robots heading as a vector in the Cartesian plane
	   * it makes an equation and computes the y intercept to determine the best way to turn
	   */
	  double Yint = (location[1]-1) - 1/(Math.tan(location[2]))*(location[0] - 1);
	  if((Yint > 0 && (location[2] < 180) )|| (Yint < 0 && location[2] > 180)) {
		  
			leftMotor.stop(true);
		  	rightMotor.stop(false);
			leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
		  	rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
		  	leftMotor.rotate(720, true);
			rightMotor.rotate(720, false);
			leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
		  	rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
		  	leftMotor.rotate(720, true);
			rightMotor.rotate(720, false);
			
	  }
	  else {
		  
			leftMotor.stop(true);
		  	rightMotor.stop(false);
			leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
		  	rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
		  	leftMotor.rotate(720, true);
			rightMotor.rotate(720, false);
			leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
		  	rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
		  	leftMotor.rotate(720, true);
			rightMotor.rotate(720, false);
			
	  }
		
		
		travelTo(x/TILE_SIZE,y/TILE_SIZE);
  }
  
  /**
   *This method turns the robot to the correct heading direction
   *
   *@param theta, in degrees tell the robot to turn that much
   */
  public void turnTo(double theta) {
	  
	  //EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      //double leftRadius, double rightRadius, double track
	  
	  leftMotor.setSpeed(ROTATE_SPEED);
	  rightMotor.setSpeed(ROTATE_SPEED);

	  leftMotor.rotate(convertAngle(leftRadius, track, theta), true);
	  rightMotor.rotate(-convertAngle(rightRadius, track, theta), false);
  }
  
  /**
   * This method is used to find out if the robot is navigating
   * 
   * @return boolean as to weather or not the robot is navigating
   */
  public boolean isNavigating() {
	  return Navigating;
  }

  
  public void printDistance() {
	  
	  double distance;
		Navigating = true;
		
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		
		leftMotor.rotate(convertAngle(leftRadius, track, 360.0), true);
	    rightMotor.rotate(-convertAngle(rightRadius, track, 360.0), true);
			
	  while(Navigating) {
			
		usDistance.fetchSample(usData, 0); // acquire data
	    distance = (usData[0] * 100.0);
		System.out.println(distance);
		
		if(!leftMotor.isMoving() && !rightMotor.isMoving()) {
			Navigating = false;
		}
	  }
  }
 
  public double[][] getScan() {
		double[] position;
		double heading;
		
		//boolean rotating = true;
		double distance;
		int i = 0;
		
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		
		leftMotor.rotate(convertAngle(leftRadius, track, 361.0), true);
	    rightMotor.rotate(-convertAngle(rightRadius, track, 361.0), true);
	    
		while(i != 359) {
			
			position = odo.getXYT();
			heading = position[2];
			i = (int) heading;
			
			usDistance.fetchSample(usData, 0); // acquire data
		    distance = (usData[0] * 100.0);
		    
			scanData[i][0] = heading;
			scanData[i][1] = distance;
		}
		return scanData;
	}

  
  public int[] getEdges() {
	  int[] edges = new int[2];
	  int averageBack = 0;
	  int averageForward = 0;
	  
	  edges[0] = 0;
	  edges[1] = 0;
	  
	  
	  
	  for(int i = 0; i < BUFFER; i++) {
		  if(scanData[i][1]>250) {
			  scanData[i][1] = 250;
		  }
		  if(scanData[i+BUFFER][1]>250) {
			  scanData[i+BUFFER][1] = 250;
		  }
		  averageBack = averageBack + ((int) scanData[i][1])/BUFFER;
	  	  averageForward = averageForward + ((int) scanData[i+BUFFER][1])/BUFFER;
	  	
		  if((averageBack - averageForward) > 150) {
			  edges[0] = BUFFER;
		  }
		  if((averageBack - averageForward) < - 150) {
			  edges[1] = BUFFER;
		  }
	  }
	  
	  
	  for(int i = BUFFER; i < (360 - BUFFER); i++) {
		  
		  if(scanData[i][1]>250) {
			  scanData[i][1] = 250;
		  }
		  if(scanData[i+BUFFER][1]>250) {
			  scanData[i+BUFFER][1] = 250;
		  }
		  if(scanData[i-BUFFER][1]>250) {
			  scanData[i-BUFFER][1] = 250;
		  }
		  
		  averageBack = averageBack + (( ((int) scanData[i][1]) - ((int) scanData[i-BUFFER][1]) )/BUFFER);
		  
		  //System.out.println(" " + averageBack + " ");
		  
		  averageForward = averageForward + ((((int) scanData[i+BUFFER][1]) - ((int)scanData[i][1]))/BUFFER);
		  
		  if((averageBack - averageForward) > 150) {
			  //falling
			  edges[0] = i;
		  }
		  if((averageBack - averageForward) < - 150) {
			  //rising
			  edges[1] = i;
		  }
	  }
	  return edges;
  }
  
  public void setHeading(int[] edges) {
	  
	  double thetaFall = scanData[edges[0]][0];
	  double thetaRise = scanData[edges[1]][0];
	  double dTheta;
	  
	  if(thetaFall < thetaRise) {
		  dTheta = 45 - (thetaFall + thetaRise)/2;
	  }
	  else {
		  dTheta = 225 - (thetaFall + thetaRise)/2;
	  }
	  
	  leftMotor.setSpeed(ROTATE_SPEED);
	  rightMotor.setSpeed(ROTATE_SPEED);
		
	  leftMotor.rotate(convertAngle(leftRadius, track, dTheta), true);
	  rightMotor.rotate(-convertAngle(rightRadius, track, dTheta), true);
	    
	  Sound.beep();
	}

  public void printColor() {	
		float color;
		Navigating = true;
		
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		
		leftMotor.rotate(720, true);
		rightMotor.rotate(720, true);
			
	  while(Navigating) {
			
		cColor.fetchSample(cData, 0); // acquire data
	    color = cData[0];
		System.out.println(color);
		
		if(!leftMotor.isMoving() && !rightMotor.isMoving()) {
			Navigating = false;
		}
	  }

	  }

}