package ca.mcgill.ecse211.finalproject;

import java.util.Arrays;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.robotics.SampleProvider;

public class WeightCan implements Runnable {

  private UnregulatedMotor weightMotor;
  private SampleProvider us; // The sample provider for the ultrasonic sensor
  private float[] usData; // The data buffer for the ultrasonic sensor reading

  private static final int ANGLE = 80;
  private static final int MIN_POWER = 50; // The power to lift a light can but not the heavy can TODO
  private static final int MAX_POWER = 100;
  private static final int WAIT_TIME = 5000; // 5 seconds TODO
  private static final int NEAR_SENSOR = 5;
  private static final int OUT_OF_BOUND = 20000;
  
  int lastTachoCount = 0;
  boolean heavy = false;

  public WeightCan(UnregulatedMotor weightMotor, SampleProvider us, float[] usData) {
    this.weightMotor = weightMotor;
    this.us = us;
    this.usData = usData;
  }

  public void run() {
    heavy = false;
    lastTachoCount = weightMotor.getTachoCount();
    claw_close(MIN_POWER);
    try {
      Thread.sleep(WAIT_TIME);
    } catch (Exception e) {
    }
    if (filter() < NEAR_SENSOR || filter() > OUT_OF_BOUND) {
      heavy = true;
      claw_close(MAX_POWER);
    } else {
      heavy = false;
    }
  }

  void claw_close(int power) {
    weightMotor.setPower(power);
    while (Math.abs(weightMotor.getTachoCount() - lastTachoCount) < ANGLE) {
      weightMotor.forward();
    }
  }

  void claw_open() {
    weightMotor.setPower(MAX_POWER);
    lastTachoCount = weightMotor.getTachoCount();
    while (Math.abs(weightMotor.getTachoCount() - lastTachoCount) < ANGLE) {
      weightMotor.backward();
    }
  }

  /**
   * The median filter of the distance detected to ignore the noice
   * 
   * @return
   */
  double filter() { // TODO debug
    double[] arr = new double[5]; // store readings
    for (int i = 0; i < 5; i++) { // take 5 readings
      us.fetchSample(usData, 0); // store reading in buffer
      arr[i] = usData[0] * 100.0; // signal amplification
    }
    Arrays.sort(arr); // sort readings
    return arr[2]; // take median value
  }

}
