package ca.mcgill.ecse211.finalproject;

import lejos.hardware.motor.UnregulatedMotor;

/**
 * This is the class that takes care of lifting and detecting the weight of a can. A can can have
 * two different weight: light or heavy.
 * 
 * @author Floria Peng
 */
public class WeightCan implements Runnable {

  /* Private Fields */
  /**
   * Field from UnregulatedMotor to have access to the setPowerMethod form Basic Motor
   */
  private UnregulatedMotor weightMotor;
  /**
   * Field from ColorClassification used to call the method that gets the reading from Ultrasonic
   * Sensor
   */
  private ColorClassification colorclassification;
  /* Constants */
  /**
   * Angle to lift
   */
  private static final int ANGLE = 80;
  /**
   * The power to lift a light can but not the heavy can
   */
  private static final int MIN_POWER = 15;
  /**
   * The power needed to lift a heavy can
   */
  private static final int MAX_POWER = 35;
  /**
   * Time to wait before we request the ultrasonic sensor
   */
  private static final int WAIT_TIME = 500;
  // to say whether or not it still detects a can.
  /**
   * Time out for claw to move
   */
  private static final int TIMER = 60;
  /**
   * The comparison interval
   */
  private static final int INTERVAL = 80;

  /* Fields */
  /**
   * Initial tachoCount
   */
  int lastTachoCount = 0;
  /**
   * Label whether or not the can is heavy
   */
  boolean heavy = false;

  /**
   * This is the default constructor. It initializes:
   * 
   * @param weightMotor and
   * @param colorclassification
   */
  public WeightCan(UnregulatedMotor weightMotor, ColorClassification colorclassification) {
    this.weightMotor = weightMotor;
    this.colorclassification = colorclassification;
  }

  /**
   * This method tries to lift a can to ANGLE degree. If the can is light, once the lifting is
   * complete, the motor will get the current tacho count. If the tacho count is large, it is light
   * can, otherwise, it is a heavy can.
   */
  public void run() {
    heavy = false; // Initialize heavy to false
    weightMotor.resetTachoCount();
    claw_close(MIN_POWER); // try to lift the can
    try {
      Thread.sleep(WAIT_TIME);
    } catch (Exception e) {
    }
    while (true) {
      if (weightMotor.getTachoCount() < INTERVAL) {
        heavy = true;
        claw_open();
        try {
          Thread.sleep(WAIT_TIME);
        } catch (Exception e) {
        }
        claw_close(MAX_POWER);
        break;
      } else {
        heavy = false;
        break;
      }
    }
  }

  /**
   * This method set the power to the unregulated motor and tries to lift the can (till the angle
   * value).
   * 
   * @param power: value between 0 and 100 that represent the percentage of how much power of the
   *        motor you use
   */
  void claw_close(int power) {
    weightMotor.resetTachoCount();
    weightMotor.setPower(power);
    int i = 0;
    while (weightMotor.getTachoCount() < ANGLE) {
      weightMotor.forward();
      i++;
      try {
        Thread.sleep(50);
      } catch (Exception e) {
      }
      if (i > TIMER) { // A small timer counter
        i = 0;
        break;
      }
    }
    weightMotor.stop();
  }

  /**
   * Drop the can back on the floor and open the claw
   */
  void claw_open() {
    weightMotor.setPower(MAX_POWER);
    int i = 0;
    while (weightMotor.getTachoCount() > 0) {
      weightMotor.backward();
      i++;
      try {
        Thread.sleep(50);
      } catch (Exception e) {
      }
      if (i > TIMER) { // A small timer counter
        i = 0;
        break;
      }
    }
    weightMotor.stop();
    weightMotor.resetTachoCount();
  }

}
