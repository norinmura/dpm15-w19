package ca.mcgill.ecse211.finalproject;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.UnregulatedMotor;

/**
 * This is the class that takes care of lifting and detecting the weight of a can. A can can have
 * two different weight: light or heavy.
 * 
 * @author Floria Peng
 *
 */
public class WeightCan implements Runnable {

  /* Private Fields */
  private UnregulatedMotor weightMotor; // use UnregulatedMotor to have access to the setPowerMethod
                                        // for (Basic Motor)
  private ColorClassification colorclassification; // used to call the method that gets the reading
                                                   // from Ultrasonic Sensor
  /* Constants */
  private static final int ANGLE = 50; // angle to lift
  private static final int MIN_POWER = 10; // The power to lift a light can but not the heavy can
                                           // TODO
  private static final int MAX_POWER = 30; // The power needed to lift a heavy can
  private static final int WAIT_TIME = 1000; // Time to wait before we request the ultrasonic sensor
                                           // to say whether or not it still detects a can. TODO
  private static final int NEAR_SENSOR = 5; // threshold value to check that determines whether or
                                            // not the can is light
  private static final int OUT_OF_BOUND = 20000;

  /* Fields */
  int lastTachoCount = 0; // initial tachoCount
  boolean heavy = false; // label whether or not the can is heavy

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
   * complete, the sensor will detect a distance superior to NEAR_SENSOR. On the other hand, if the
   * can is heavy, the sensor will detect a distance less than NEAR_SENSOR.
   */
  public void run() {
    heavy = false; // Initialize heavy to false
    lastTachoCount = weightMotor.getTachoCount(); // Update lastTachoCount to current value
    claw_close(MIN_POWER); // try to lift the can
    try {
      Thread.sleep(WAIT_TIME);
    } catch (Exception e) {
    }
    // If the can is heavy, then if shouldn't have moved more than NEAR_SENSOR from and should have
    // move less than OUT_OF_BOUND.
    if (colorclassification.median_filter() < NEAR_SENSOR
        || colorclassification.median_filter() > OUT_OF_BOUND) {
      heavy = true;
      LCD.clear();
      claw_open();
      LCD.clear();
      claw_close(MAX_POWER);
    } else {
      heavy = false;
    }
  }

  /**
   * This method set the power to the unregulated motor and tries to lift the can (till the angle
   * value).
   * 
   * @param power: value between 0 and 100 that represent the percentage of how much power of the
   *        motor you use
   */
  public void claw_close(int power) {
    Sound.beep();
    weightMotor.resetTachoCount();
    weightMotor.setPower(power);
    while (Math.abs(weightMotor.getTachoCount() - lastTachoCount) < ANGLE) {
      weightMotor.forward();
      if (!weightMotor.isMoving()) {
        break;
      }
    }
    weightMotor.stop();
  }

  /**
   * Drop the can back on the floor and open the claw
   */
  public void claw_open() {
    weightMotor.setPower(MAX_POWER);
    lastTachoCount = weightMotor.getTachoCount();
    while (Math.abs(weightMotor.getTachoCount() - lastTachoCount) < ANGLE) {
      weightMotor.backward();
      if (!weightMotor.isMoving()) {
        break;
      }
    }
    weightMotor.stop();
  }
  
  public int getMaxPower() {
	return MAX_POWER;  
  }

  public boolean isHeavy() {
	  return heavy;
  }
}
