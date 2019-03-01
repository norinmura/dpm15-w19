/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;


import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;


/**
 * This class calculate the current location with correction using color sensor and sends it to OdometerData with the angle of the car. 
 * @author Rintaro Nomura
 * @author Laurie Zaccarin
 */

public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private Odometer odometer;

  // Used for correction
  private int counter = 0;
  private double sensorValueOld = 0;

  // Constants
  private final double TILE_SIZE = 30.48;

  // Set of instances needed to use the sensor value
  private static final Port colorSensor = LocalEV3.get().getPort("S1");				// Loading port
  private static final SensorModes sensorSampler = new EV3ColorSensor(colorSensor);	// Hook sampler
  private static final SampleProvider sensorValue = sensorSampler.getMode("Red");	// Set up mode: Red - Measures the intensity of a reflected red light
  private float[] sensorData = new float[sensorSampler.sampleSize()];				// To store data in


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
   * 
   * This method runs the odometer with an applied correction.
   * In the correction version, first the value of the color sensor is collected, and the differential filter is set.
   * The correction uses the strategy of identifying how far the robot has travel according to how many lines
   * it has crossed. The distance travel is then set according to the trajectory completed.
   * These distance values are updated to the odometer.
   */
  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;

    while (true) {
      correctionStart = System.currentTimeMillis();

      // Obtain the value from the color sensor
      sensorValue.fetchSample(sensorData, 0);
      double value = sensorData[0];

      // setting up differential filter
      double dydx = value - this.sensorValueOld;
      this.sensorValueOld = value;

      if (dydx < -0.050) {
        Sound.beep();

        double[] odoData = odometer.getXYT();
        double distance_x = odoData[0];
        double distance_y = odoData[1];
        
        if (this.counter == 0) {  // first time detecting black line (count == 0): reset Y
          odometer.setY(0);
        }
        else if (this.counter == 3){ // first time turning right (count == 3): reset X
          odometer.setX(0);
        }
        else {
          if (this.counter < 3) {	// first move, moving forward, distance traveled (y-direction) is: y-offset + how many tiles it crossed. 
            distance_y = (this.counter * this.TILE_SIZE);
          }
          else if (this.counter < 6) {	// second move, first turn, distance traveled (x-direction) is: x-offset + how many tiles it crosses
            distance_x = ((this.counter - 3) * this.TILE_SIZE);
          }
          else if (this.counter < 9) {	// third move, second turn, distance traveled (y-direction) is: y-offset + 2tiles - count*tiles
            distance_y = (2 * this.TILE_SIZE - ((this.counter - 6) * this.TILE_SIZE));
          }
          else if (this.counter < 11){	// last move, third turn, distance traveled (x-direction) is: x-offset + 2tiles - count*tiles
            distance_x = (2 * this.TILE_SIZE - ((this.counter - 9) * this.TILE_SIZE));
          }        
          
          // Update the odometer
          odometer.setX(distance_x);
          odometer.setY(distance_y);
        }

        // increase count
        this.counter++;
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
