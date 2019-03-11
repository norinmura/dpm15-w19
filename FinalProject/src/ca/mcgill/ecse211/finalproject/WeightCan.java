package ca.mcgill.ecse211.finalproject;

import lejos.hardware.motor.UnregulatedMotor;

public class WeightCan implements Runnable {

  private UnregulatedMotor weightMotor;
  private ColorClassification colorclassification;

  private static final int ANGLE = 80;
  private static final int MIN_POWER = 50; // The power to lift a light can but not the heavy can TODO
  private static final int MAX_POWER = 100;
  private static final int WAIT_TIME = 50; // TODO
  private static final int NEAR_SENSOR = 5;
  private static final int OUT_OF_BOUND = 20000;
  
  int lastTachoCount = 0;
  boolean heavy = false;

  public WeightCan(UnregulatedMotor weightMotor, ColorClassification colorclassification) {
    this.weightMotor = weightMotor;
    this.colorclassification = colorclassification;
  }

  public void run() {
    heavy = false;
    lastTachoCount = weightMotor.getTachoCount();
    claw_close(MIN_POWER);
    try {
      Thread.sleep(WAIT_TIME);
    } catch (Exception e) {
    }
    if (colorclassification.median_filter() < NEAR_SENSOR || colorclassification.median_filter() > OUT_OF_BOUND) {
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
    weightMotor.stop();
  }

  void claw_open() {
    weightMotor.setPower(MAX_POWER);
    lastTachoCount = weightMotor.getTachoCount();
    while (Math.abs(weightMotor.getTachoCount() - lastTachoCount) < ANGLE) {
      weightMotor.backward();
    }
    weightMotor.stop();
  }

}
