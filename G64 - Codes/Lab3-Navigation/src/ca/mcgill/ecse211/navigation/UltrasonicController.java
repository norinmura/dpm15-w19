/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.navigation;


import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;

/**
 * This class implements the function to avoid the obstacles using ultrasonic sensor. It
 * continuously obtains the sensor reading and stores the value into odometerdata.
 * 
 * @author Rintaro Nomura
 * @author Laurie Zaccarin
 */

public class UltrasonicController implements Runnable {
  private Odometer odometer;
  private Navigation navigation;

  private final double OBSTACLE_SIZE = 30.48;
  private final int UPPER_LIMIT = 70;
  private static final int FILTER_OUT = 20;

  private int filterControl;
  private int distance;

  // Set of instances needed to use the sensor value
  @SuppressWarnings("resource") // Because we don't bother to close this resource
  private static final Port usSensor = LocalEV3.get().getPort("S2"); // Loading port
  private static final SensorModes sensorSampler = new EV3UltrasonicSensor(usSensor); // Hook
                                                                                      // sampler
  private static final SampleProvider sensorValue = sensorSampler.getMode("Distance");// create
                                                                                      // sample
                                                                                      // provider
                                                                                      // instance,
                                                                                      // setting up
                                                                                      // mode
  private float[] sensorData = new float[sensorSampler.sampleSize()]; // To store data in

  private static EV3MediumRegulatedMotor sensorMotor =
      new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));


  /**
   * This is the constructor. An existing instance of the odometer is used. This is to ensure thread
   * safety.
   * 
   * @throws OdometerExceptions
   */
  public UltrasonicController() throws OdometerExceptions, NavigationExceptions {
    this.odometer = Odometer.getOdometer();
    this.navigation = Navigation.getNavigation();
  }


  // run method (required for Thread)
  public void run() {
    try {
      Thread.sleep(300);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }

    // Letting medium motor to move the US sensor back and forth
    (new Thread() { // using thread to avoid interrupting other functions
      public void run() {
        sensorMotor.setSpeed(120);
        sensorMotor.rotate(-15);
        while (true) {
          sensorMotor.rotate(30);
          sensorMotor.rotate(-30);
        }
      }
    }).start();

    while (true) {
      sensorValue.fetchSample(sensorData, 0); // acquire data
      this.distance = (int) (sensorData[0] * 100.0); // extract from buffer, cast to int

      // Sensor value filter
      if (distance >= UPPER_LIMIT && filterControl < FILTER_OUT) {
        // bad value, do not set the distance var, however do increment the filter value
        this.filterControl++;
      } else if (distance >= UPPER_LIMIT) {
        // We have repeated large values, so there must actually be nothing
        // there: leave the distance alone
        odometer.setDistance(distance); // Set the distance variable in OdoData
      } else {
        // distance went below 255: reset filter and leave distance alone.
        filterControl = 0;
        odometer.setDistance(distance); // Set the distance variable in OdoData
      }

      try {
        Thread.sleep(50);
      } catch (Exception e) {

      }

    }

  }
}
