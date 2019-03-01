package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.*;

/**
 * BangBang Controller for ECSE211-Lab1
 *
 * @author Group 64 (Laurie Zaccarin, Rintaro Nomura)
 * @version 1.0
 * @since 2019-01-21
 */

public class BangBangController implements UltrasonicController {

  private final int bandCenter;
  private final int bandwidth;
  private final int motorLow;
  private final int motorHigh;
  private final int motorDelta;
  private int distance;
  private final int upper_limit = 70;

  private static final int FILTER_OUT = 20;
  private int filterControl;

  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    this.motorDelta = 25;
    this.filterControl = 0;

    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Setting up the speed, but not starting until
                                                    // sensor is ready
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
  }

  @Override
  public void processUSData(int distance) {

    /**
     * This method filters out erroneous distances measured by the ultrasonic sensor and identifies
     * correct values. If the value is very large in comparison to the previous and next values,
     * then it is erroneous and is filtered out. If it appears in a long set of large distances,
     * then it is correct. Finally if it is a small value it is correct.
     */

    if (distance >= upper_limit && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
    } else if (distance >= upper_limit) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;

      // sensor detects it a too early due to angle, so wait for awhile until wheel aligns with
      // corner.
      try {
        Thread.sleep(300);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }

    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }

    // Call the controller
    bangControl();


  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }

  public void bangControl() {

    /**
     * The Bang Bang Control works by analyzing the distance between the robot and the obstacle and
     * reacting according to three different situations. In the first case the distance is
     * acceptable and within limit: the robot continues at the same speed. In the second case the
     * robot is too close to the wall. The robot then turns right to increase the distance, both by
     * turning the right motor backwards and my turning the left forward. Finally in the third case
     * the robot is too far from the wall. Here the robot must turn left simply by turning the right
     * motor more.
     */

    // Too close: Error < 0, Too far : Error > 0
    int distError = this.distance - this.bandCenter;

    if (Math.abs(distError) <= bandwidth) { // within limits: continue straight
      WallFollowingLab.leftMotor.setSpeed(motorHigh);
      WallFollowingLab.rightMotor.setSpeed(motorHigh);
      WallFollowingLab.leftMotor.forward();
      WallFollowingLab.rightMotor.forward();
    } else if (distError < 0) { // too close to the wall: turn right
      if (distError < -15) { // too too close to the wall (concave turn)
        WallFollowingLab.leftMotor.setSpeed(motorLow);
        WallFollowingLab.rightMotor.setSpeed(motorHigh - motorDelta - motorDelta);
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.backward(); // right wheel moving backward to avoid collision
      } else { // usual adjustment during forward movement
        WallFollowingLab.leftMotor.setSpeed(motorLow);
        WallFollowingLab.rightMotor.setSpeed(motorHigh);
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.backward();
      }
    }

    else if (distError > 0) { // too far from the wall (also making left turn): turn left
      WallFollowingLab.leftMotor.setSpeed(motorLow + motorDelta);
      WallFollowingLab.rightMotor.setSpeed(motorHigh - motorDelta);
      WallFollowingLab.leftMotor.forward();
      WallFollowingLab.rightMotor.forward();
    } /*
       * else if (distError > 0) { // For slippery floor: too far from the wall (also making left
       * turn): turn left WallFollowingLab.leftMotor.setSpeed((motorLow + motorDelta) * 2);
       * WallFollowingLab.rightMotor.setSpeed((motorHigh - motorDelta) * 2);
       * WallFollowingLab.leftMotor.forward(); WallFollowingLab.rightMotor.forward(); }
       */

  }
}
