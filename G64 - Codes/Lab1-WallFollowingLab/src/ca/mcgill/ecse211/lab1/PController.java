package ca.mcgill.ecse211.lab1;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Proportional Controller for ECSE211-Lab1
 *
 * @author Group 64 (Laurie Zaccarin, Rintaro Nomura)
 * @version 1.0
 * @since 2019-01-21
 */

public class PController implements UltrasonicController {
  private static final int MOTOR_SPEED = 140;
  private static final int FILTER_OUT = 30;
  private final int upper_limit = 90;
  private final int pconst = 4;
  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl;

  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0;

    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Setting up the speed, but not starting
                                                      // until sensor is ready
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
  }

  @Override
  public void processUSData(int distance) {

    /**
     * This method filters out erroneous distances measured by the ultrasonic sensor and identifies
     * correct values. If the value is very large in comparison to the previous and next values,
     * then it is erroneous and is filtered out. If it appears in a long set of large distances,
     * then it is correct. Finally if it is a small value it is correct.
     */

    // rudimentary filter - toss out invalid samples corresponding to null signal.
    if (distance >= upper_limit && filterControl < FILTER_OUT) {
      // Potentially bad value. Increment the filter value.
      filterControl++;
    } else if (distance >= upper_limit) {
      // We have repeated large values, so there must actually be nothing there: leave the distance
      // alone
      this.distance = distance;

      // sensor detects it a too early due to sensor angle, so wait for a moment until wheel aligns
      // with corner.
      try {
        // Thread.sleep(3570); // Theoretically calculated value
        Thread.sleep(1500);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }

      pcontrol();
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
      pcontrol();
    }


    /*
     * try { Thread.sleep(20); } catch (InterruptedException e) { // TODO Auto-generated catch block
     * e.printStackTrace(); }
     */

  }


  @Override
  public int readUSDistance() {
    return this.distance;
  }


  public void pcontrol() {

    /**
     * The P control functions by first calculating the error in the distance from the wall, and
     * then accordingly calculating the appropriate correction. If the error is within the tolerated
     * interval, then the robot continues at a constant speed with both motors turning at the same
     * speed. If it is too far from the wall, then the adjustment must be calculated and the right
     * motor's speed increased to move back closer to the wall. If it is too close to the wall, then
     * once more the adjustment must be calculated, then the right motor is set to move backwards
     * and the left forward in order to make a right turn.
     * 
     */

    // Calculate the distance error used to trigger bang-bang controller
    // Too close: Error < 0, Too far : Error > 0
    int distError = this.distance - this.bandCenter;

    // P-controller
    if (Math.abs(distError) <= this.bandWidth) { // within limits: continue straight
      WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
      WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
      WallFollowingLab.leftMotor.forward();
      WallFollowingLab.rightMotor.forward();
    } else if (distError < 0) { // too close to the wall: turn right
      WallFollowingLab.leftMotor.setSpeed((int) (MOTOR_SPEED));
      WallFollowingLab.rightMotor.setSpeed((int) (MOTOR_SPEED + pconst * -(distError)));
      WallFollowingLab.leftMotor.forward();
      WallFollowingLab.rightMotor.backward();
    } else if (distError > 0) { // too far from the wall (also making left turn): turn left
      if (distError > 30) {
        distError = 15;
      }
      WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
      WallFollowingLab.rightMotor.setSpeed((int) (MOTOR_SPEED + pconst * distError));
      WallFollowingLab.leftMotor.forward();
      WallFollowingLab.rightMotor.forward();
    }
  }
}
