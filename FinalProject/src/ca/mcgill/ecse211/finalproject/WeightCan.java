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
   * The power to lift a light can but not the heavy can
   */
  private static final int MIN_POWER = 20;
  /**
   * The power to lift a heavy can
   */
  private static final int MAX_POWER = 35;
  /**
   * Time to wait before we request the ultrasonic sensor
   */
  private static final int WAIT_TIME = 1000;
  // to say whether or not it still detects a can.
  /**
   * Time out for claw to close
   */
  private static final int TIMER_CLOSE = 2500;
  /**
   * Time out for claw to open
   */
  private static final int TIMER_OPEN = 2000;
  /**
   * The comparison interval
   */
  private static final int INTERVAL = 83;

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
    claw_open();
    weightMotor.resetTachoCount();
    claw_close(MIN_POWER); // try to lift the can
    try {
      Thread.sleep(WAIT_TIME);
    } catch (InterruptedException e1) {
      e1.printStackTrace();
    }
    if (weightMotor.getTachoCount() < INTERVAL) {
      claw_open();
      claw_close(MAX_POWER);
      heavy = true;
    } else {
      heavy = false;
    }
    
    weightMotor.setPower(MAX_POWER);
  }

  /**
   * This method set the power to the unregulated motor and tries to lift the can (till the angle
   * value).
   * 
   * @param power - value between 0 and 100 that represent the percentage of how much power of the
   *        motor you use
   */
  void claw_close(int power) {
    weightMotor.resetTachoCount();
    weightMotor.setPower(power);

    double beginTime = System.currentTimeMillis();
    double endTime = System.currentTimeMillis();
    while (true) {
      weightMotor.forward();
      endTime = System.currentTimeMillis();
      System.out.println(endTime - beginTime);
      if (endTime - beginTime > TIMER_CLOSE) {
        break;
      }
    }
    weightMotor.setPower(0);
  }

  /**
   * Drop the can back on the floor and open the claw
   */
  void claw_open() { // reset the claw to a starting point

    double beginTime = System.currentTimeMillis();
    double endTime = System.currentTimeMillis();
    while (true) {
      weightMotor.setPower(10);
      weightMotor.backward();
      endTime = System.currentTimeMillis();
      if (endTime - beginTime > TIMER_OPEN) {
        break;
      } else {

      }
    }
    weightMotor.setPower(0);
  }

}
