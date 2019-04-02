/*
 * SquareDriver.java
 */
package ca.mcgill.ecse211.test;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class is used to drive the robot on the demo floor.
 * All the settings should be done from the class that is calling this. 
 */
public class SquareDriver {
  /**
   * This method is meant to drive the robot in a square of size 4x4 Tiles. It is to run in parallel. 
   *  
   * @param leftMotor
   * @param rightMotor
   * @param leftRadius
   * @param rightRadius
   * @param width
   */
  
  public static void drive(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      double leftRadius, double rightRadius, double track) {
    // reset the motors
    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
      motor.stop();
      motor.setAcceleration(LineDetectionTest.ACCELERATION);
    }

    // Sleep for 2 seconds
    try {
      Thread.sleep(2000);
    } catch (InterruptedException e) {
      // There is nothing to be done here
    }

    // Square
    
    for (int i = 0; i < 4; i++) {
      // drive forward two tiles
      leftMotor.setSpeed(LineDetectionTest.FORWARD_SPEED);
      rightMotor.setSpeed(LineDetectionTest.FORWARD_SPEED);

      leftMotor.rotate(convertDistance(leftRadius, 3 * LineDetectionTest.TILE_SIZE), true);
      rightMotor.rotate(convertDistance(rightRadius, 3 * LineDetectionTest.TILE_SIZE), false);

      // turn 90 degrees clockwise
      leftMotor.setSpeed(LineDetectionTest.ROTATE_SPEED);
      rightMotor.setSpeed(LineDetectionTest.ROTATE_SPEED);

      leftMotor.rotate(convertAngle(leftRadius, track, 90.0), true);
      rightMotor.rotate(-convertAngle(rightRadius, track, 90.0), false);
    }
    
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
}
