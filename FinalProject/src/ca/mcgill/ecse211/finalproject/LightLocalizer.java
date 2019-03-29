package ca.mcgill.ecse211.finalproject;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * 
 * This class implements the Light localization of the robot. With the use of two light sensors one
 * on each side of the robot, the robot can localize by stopping the motor that detects a line and
 * moving the other until it detects a line.
 * 
 * @author Floria Peng
 */
public class LightLocalizer implements Runnable {

  /* PRIVATE FIELDS */
  /**
   * The odometer instance
   */
  private Odometer odometer; // The odometer instance
  /**
   * The left motor of the robot
   */
  private EV3LargeRegulatedMotor leftMotor; // The left motor of the robot
  /**
   * The right motor of the robot
   */
  private EV3LargeRegulatedMotor rightMotor; // The right motor of the robot
  /**
   * The instance of line correction
   */
  private LineCorrection linecorrection; // The instance of line correction
  /**
   * The instance of sensor rotation
   */
  private Navigation navigation; // The instance of sensor rotation

  /* CONSTANTS */
  /**
   * The tile size used for demo
   */
  public static final double TILE_SIZE = 30.48; // The tile size used for demo
  /**
   * Angle facing the corner
   */
  public static final int FACING_CORNER = 225; // Angle facing the corner
  /**
   * 360 degree for a circle
   */
  public static final int FULL_TURN = 360; // 360 degree for a circle
  /**
   * Travel back distance (distance between wheels and sensors)
   */
  private static final double BACK_DIST = 9.0; // Travel back distance (distance between wheels and
                                               // sensors)
  /**
   * The fetching rate
   */
  private static final long FETCH_PERIOD = 50;

  /* NON-PRIVATE FIELDS */
  /**
   * The left wheel radius of the robot
   */
  double leftRadius; // The left wheel radius of the robot
  /**
   * The right wheel radius of the robot
   */
  double rightRadius; // The right wheel radius of the robot
  /**
   * The track of the robot (by measuring the distance between the center of both wheel)
   */
  double track; // The track of the robot (by measuring the distance between the center of both
                // wheel)
  /**
   * The corner that the robot starts
   */
  int corner; // The corner that the robot starts

  /**
   * Initialize the last variable to a specific number
   */
  double last = Math.PI; // Initialize the last variable to a specific number
  /**
   * last and current are both used for differential filter
   */
  double current = 0; // last and current are both used for differential filter
  /**
   * The x and y tile line detect angle, clockwise
   */
  double[] detect1 = new double[4]; // The x and y tile line detect angle, clockwise
  /**
   * The x and y tile line detect angle, clockwise
   */
  double[] detect2 = new double[4]; // The x and y tile line detect angle, clockwise
  /**
   * The time of the light sensor
   */
  long[] time = new long[2]; // The time of the light sensor
  /**
   * The detection of the line of the two light sensors
   */
  boolean[] line = {false, false}; // The detection of the line of the two light sensors
  /**
   * The localization error in the x direction
   */
  double xerror = 0; // The localization error in the x direction
  /**
   * The localization error in the y direction
   */
  double yerror = 0; // The localization error in the y direction
  /**
   * The localization error in angle
   */
  double terror = 0; // The localization error in angle
  /**
   * The before line correction angle
   */
  double before = 0; // The before line correction angle
  /**
   * The starting time of each fetch sample
   */
  long tstart = 0;
  /**
   * The ending time of each fetch sampel
   */
  long tend = 0;

  /**
   * The default constructor of this class
   * 
   * @param odometer
   * @param leftMotor
   * @param rightMotor
   * @param leftRadius
   * @param rightRadius
   * @param track
   * @param navigation
   * @param linecorrection
   * @param corner
   * @throws OdometerExceptions
   */
  public LightLocalizer(Odometer odometer, EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, double leftRadius, double rightRadius, double track,
      Navigation navigation, LineCorrection linecorrection, int corner) throws OdometerExceptions {
    this.odometer = odometer;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.leftRadius = leftRadius;
    this.rightRadius = rightRadius;
    this.track = track;
    this.navigation = navigation;
    this.linecorrection = linecorrection;
    this.corner = corner;
  }

  /**
   * The run method of this class. It will travel to 45 degree forward a specific distance, detect
   * the black line, retreat for a pre-defined distance, starts rotation and records four angles of
   * the robot when the black lines are detected, and finally travel to the origin point and adjust
   * itself based on the x and y error
   * 
   * @see java.lang.Runnable#run()
   */
  public void run() {

    /* Line localization */
    // The robot will first travel 45 degree front-right first until the light sensor detects a line
    navigation.move(TILE_SIZE); // move forward (until you detect a line) to correct Y odometer
                                // reading
    correctAngle(); // when a line is detected, correct angle
    navigation.back(0, BACK_DIST); // Go back the offset distance between the wheels and sensors
    navigation.rotate(FULL_TURN / 4); // Rotate 90 degrees clockwise

    navigation.move(TILE_SIZE); // move forward (until you detect a line) to correct X odometer
                                // reading
    correctAngle(); // when a line is detected, correct angle
    navigation.back(BACK_DIST, 0); // Move back the offset distance between the wheels and sensors
    navigation.rotate(-FULL_TURN / 4); // Rotate 90 degrees anti-clockwise

    navigation.move(TILE_SIZE); // move forward (until you detect a line) to correct Y value (double
                                // check)
    correctAngle();// when a line is detected, correct angle
    navigation.back(0, BACK_DIST); // Go back offset distance, you reach the origin

    /* Correct the coordination */
    // Depending on the starting corner, set the Theta value accordingly
    switch (corner) {
      case 0:
        odometer.setXYT(1 * TILE_SIZE, 1 * TILE_SIZE, 0);
        odometer.position[2] = Math.toRadians(0);
        break;
      case 1:
        odometer.setXYT(14 * TILE_SIZE, 1 * TILE_SIZE, 270);
        odometer.position[2] = Math.toRadians(270);
        break;
      case 2:
        odometer.setXYT(14 * TILE_SIZE, 8 * TILE_SIZE, 180);
        odometer.position[2] = Math.toRadians(180);
        break;
      case 3:
        odometer.setXYT(1 * TILE_SIZE, 8 * TILE_SIZE, 90);
        odometer.position[2] = Math.toRadians(90);
        break;
    }

  }

  /**
   * This helper method is used to correct the Angle
   */
  void correctAngle() {
    while (true) {
      /*
       * Line 0 corresponds to boolean if left motor detected a line Line 1 corresponds to boolean
       * if right motor detected a line
       */
      tstart = System.currentTimeMillis();
      line[0] = linecorrection.filter1(); // set line[0] to whether or not a line was detected
      line[1] = linecorrection.filter2(); // set line[1] to whether or not a line was detected
      if (line[0]) { // If the black line is detected, the robot will stop (left motor)
        leftMotor.stop(true);
      }
      if (line[1]) { // (right motor)
        rightMotor.stop(true);
      }
      if (!leftMotor.isMoving() && !rightMotor.isMoving()) { // If both light sensor detects the
                                                             // black line
        // It means both wheels are in the same line
        line[0] = false;
        line[1] = false;
        leftMotor.stop(true);
        rightMotor.stop(false);
        if (odometer.getXYT()[2] < 30 || odometer.getXYT()[2] > 330) {
          odometer.setTheta(0);
          odometer.position[2] = Math.toRadians(0);
        } else if (Math.abs(odometer.getXYT()[2] - 90) < 60) {
          odometer.setTheta(90);
          odometer.position[2] = Math.toRadians(90);
        }
        break;
      }
      tend = System.currentTimeMillis();
      if (tend - tstart < FETCH_PERIOD) {
        try {
          Thread.sleep(FETCH_PERIOD - (tend - tstart));
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    }
  }

}
