package ca.mcgill.ecse211.finalproject;

import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

/**
 * This class contains the basic method for the navigation. The robot is able to go
 * forward/backward, turn and search for/identify objects. All the movements of the robot is
 * controlled by the this class. The methods will be called by FinalProject, UltrasonicLocalizer or
 * LightLocalizer.
 * 
 * @author Floria Peng
 */
public class Navigation {

  /* STATIC FIELDS */
  /**
   * The forward speed for the robot
   */
  public static final int FORWARD_SPEED = 200; // The forward speed for the robot
  /**
   * The run speed for the robot to travel through the tunnel
   */
  public static final int RUN_SPEED = 300; // The run speed for the robot
  /**
   * The rotating speed of the robot
   */
  public static final int ROTATE_SPEED = 160; // The rotation speed for the robot
  /**
   * The distance that the robot think there is an object in front of it
   */
  private static final double SCAN_DISTANCE = 6; // The detect a can distance
  /**
   * The distance that the robot move to go closer to the can
   */
  private static final double APPROACH_CAN = 8; // Get closer to the can
  /**
   * The sound frequency
   */
  private static final int TUNE = 880;
  /**
   * The angle of a full turn is 360 degrees
   */
  public static final int FULL_TURN = 360; // 360 degree for a circle
  /**
   * The tile size of the board that the robot is running on
   */
  public static final double TILE_SIZE = 30.48; // The tile size used for demo
  /**
   * The move method speed adjustment
   */
  public static final int MOVE_ADJ = 0;
  /**
   * The rotate method speed adjustment
   */
  public static final int ROTATE_ADJ = 20;
  /**
   * The turn method speed adjustment
   */
  public static final int TURN_ADJ = 50;
  /**
   * The approach can method speed adjustment
   */
  public static final int APPROACH_ADJ = 30;
  /**
   * The back method speed adjustment
   */
  public static final int BACK_ADJ = 20;
  /**
   * The maximum power of the claw
   */
  public static final int MAX_POWER = 35;
  /**
   * The time out for the line correction method
   */
  public static final int TIMER = 20;
  /**
   * The error allowance of the ultrasonic sensor
   */
  public static final int ERROR = 10;

  /* PRIVATE FIELDS */
  /**
   * The Odometer instance
   */
  private Odometer odometer; // The odometer instance
  /**
   * The LineCorrection instance
   */
  private LineCorrection linecorrection; // The instance of line correction
  /**
   * The leftMotor instance
   */
  private EV3LargeRegulatedMotor leftMotor; // The left motor of the robot
  /**
   * The rightMotor instance
   */
  private EV3LargeRegulatedMotor rightMotor; // The right motor of the robot
  /**
   * The sensorMotor instance
   */
  private EV3MediumRegulatedMotor sensorMotor; // The sensor motor of the robot
  /**
   * The ColorClassification instance
   */
  private ColorClassification colorclassification; // The ColorClassification instance
  /**
   * The WeightCan instance
   */
  private WeightCan weightcan; // The WeightCan instance

  /* FIELDS */
  /**
   * The left wheel radius
   */
  double leftRadius; // The left wheel radius of the robot
  /**
   * The right wheel radius
   */
  double rightRadius; // The right wheel radius of the robot
  /**
   * The track width
   */
  double track; // The track of the robot (by measuring the distance between the center of both
                // wheel)
  /**
   * The current x position before the robot moves
   */
  double lastx; // The last x position of the robot
  /**
   * The current y position before the robot moves
   */
  double lasty; // The last y position of the robot
  /**
   * The current theta angle before the robot moves
   */
  double lasttheta; // The last angle of the robot
  /**
   * The traveling distance between the current point with the next map point
   */
  double travel; // The traveling distance between the current point with the next map point
  /**
   * The distance calculated to go move
   */
  double distance; // The distance calculated to go move
  /**
   * The angle to which the robot should turn to get to the next map point
   */
  double angle; // The angle to which the robot should turn to get to the next map point
  /**
   * The angle that the robot should actually rotate
   */
  double theta; // The angle that the robot should actually rotate
  /**
   * The front ultrasonic sensor distance detected
   */
  double warning; // The front ultrasonic sensor distance detected
  /**
   * True for the robot is traveling to next point
   */
  boolean isNavigating = false; // True for the robot is traveling to next point

  /**
   * The time of the light sensor
   */
  long[] time = new long[2]; // The time of the light sensor
  /**
   * The detection of the line of the two light sensors
   */
  boolean[] line = {false, false}; // The detection of the line of the two light sensors
  /**
   * true for the object detected is not a can
   */
  boolean not_can = false;
  /**
   * true for detecting a can
   */
  boolean get_can = false;
  /**
   * The angles that the robot uses to find the can
   */
  double[] angles = new double[3];
  /**
   * The distance that the robot uses to find the can
   */
  double[] distances = new double[3];
  /**
   * The angle that the roundSearch method should end
   */
  double end_angle = 0; // The angle left for rotate search
  /**
   * The distance of the can that the Ultrasonic sensor detects during the roundSearch
   */
  double round_detect = 0;
  /**
   * The after line correction, before next correction delay
   */
  int delay = 20; // The after line correction, before next correction delay
  /**
   * The beeping time
   */
  int sound_time = 0;

  /**
   * The constructor for the Navigation class
   * 
   * @param odometer - The odometer of the robot
   * @param leftMotor - The leftMotor of the robot
   * @param rightMotor - The rightMotor of the robot
   * @param leftRadius - The left wheel radius of the robot
   * @param rightRadius - The right wheel radius of the robot
   * @param track - The track of the robot measured form the distance between the center of both
   *        wheels
   * 
   * @throws OdometerExceptions
   */
  public Navigation(Odometer odometer, EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, EV3MediumRegulatedMotor sensorMotor,
      ColorClassification colorclassification, WeightCan weightcan, LineCorrection linecorrection,
      double leftRadius, double rightRadius, double track) throws OdometerExceptions {
    this.odometer = odometer;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.sensorMotor = sensorMotor;
    this.colorclassification = colorclassification;
    this.weightcan = weightcan;
    this.linecorrection = linecorrection;
    this.leftRadius = leftRadius;
    this.rightRadius = rightRadius;
    this.track = track;
  }

  /**
   * <p>
   * This method causes the robot to travel to the absolute field location (x, y), specified in tile
   * points. This method should continuously call turnTo(double theta) and then set the motor speed
   * to forward(straight). This will make sure that your heading is updated until you reach your
   * exact goal. This method will poll the odometer for information.
   * 
   * <p>
   * This method is also detecting can while moving.
   * 
   * @param x - The x coordinate for the next point
   * @param y - The y coordinate for the next point
   * 
   * @return - void method, no return
   */
  void moveTo(double x, double y) {

    lastx = odometer.getXYT()[0]; // The last x position of the robot
    lasty = odometer.getXYT()[1]; // The last y position of the robot

    travel = Math.sqrt(Math.pow(x - lastx, 2) + Math.pow(y - lasty, 2)); // The travel distance
    angle = Math.atan2(x - lastx, y - lasty) * 180 / Math.PI; // The angle that the robot should
                                                              // rotate to

    turnTo(angle); // Call the turnTo method

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    // Travel the robot to the destination point
    leftMotor.rotate(convertDistance(leftRadius, travel), true);
    rightMotor.rotate(convertDistance(rightRadius, travel), true);

    while (leftMotor.isMoving() || rightMotor.isMoving()) { // If the robot is moving
      detectCan();
      try {
        Thread.sleep(50);
      } catch (Exception e) {
      }
    }

  }

  /**
   * <p>
   * This method causes the robot to travel to the absolute field location (x, y), specified in tile
   * points. This method should continuously call turnTo(double theta) and then set the motor speed
   * to forward(straight). This will make sure that your heading is updated until you reach your
   * exact goal. This method will poll the odometer for information. The robot will correct its
   * angle when crossing a line
   * 
   * <p>
   * This method cannot be break. Traveling to the destination point with a large speed.
   * 
   * @param x - The x coordinate for the next point
   * @param y - The y coordinate for the next point
   * 
   * @return - void method, no return
   */
  void travelTo(double x, double y) {

    /* Calculate move angle and distance */
    lastx = odometer.getXYT()[0]; // The last x position of the robot
    lasty = odometer.getXYT()[1]; // The last y position of the robot

    travel = Math.sqrt(Math.pow(x - lastx, 2) + Math.pow(y - lasty, 2)); // The travel distance
    angle = Math.atan2(x - lastx, y - lasty) * 180 / Math.PI; // The angle that the robot should
                                                              // rotate to

    turnTo(angle); // Call the turnTo method

    leftMotor.setSpeed(RUN_SPEED);
    rightMotor.setSpeed(RUN_SPEED);
    // Travel the robot to the destination point
    leftMotor.rotate(convertDistance(leftRadius, travel), true);
    rightMotor.rotate(convertDistance(rightRadius, travel), false);

  }

  /**
   * <p>
   * This method causes the robot to travel to the absolute field location (x, y), specified in tile
   * points. This method should continuously call turnTo(double theta) and then set the motor speed
   * to forward(straight). This will make sure that your heading is updated until you reach your
   * exact goal. This method will poll the odometer for information. The ultrasonic will keep
   * detecting can while the robot is moving
   * 
   * <p>
   * This method can be break. Detect can, used for approaching the can found during the rotate
   * search.
   * 
   * @param distance - The distance to go
   * 
   * @return - void method, no return
   */
  void goTo(double distance) {

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    // Travel the robot to the destination point
    leftMotor.rotate(convertDistance(leftRadius, distance), true);
    rightMotor.rotate(convertDistance(rightRadius, distance), true);

    while (leftMotor.isMoving() || rightMotor.isMoving()) { // If the robot is moving
      detectCan();
      try {
        Thread.sleep(50);
      } catch (Exception e) {
      }
    }
  }

  /**
   * This method implements the can detection. The robot will stop in front of the can and detect
   * the color and weight.
   */
  void detectCan() {
    /* INITIALIZE VARIABLES */
    get_can = false;

    warning = colorclassification.median_filter();
    if (warning < SCAN_DISTANCE) {
      leftMotor.stop(true);
      rightMotor.stop(false);

      approachCan(APPROACH_CAN); // Move close to the can

      /* Detect the color */
      Thread classificationThread = new Thread(colorclassification); // set a new thread to scan
                                                                     // the color
      classificationThread.start(); // the color scanning thread starts
      sensorMotor.setSpeed(ROTATE_SPEED / 2); // set the scanning speed
      sensorMotor.rotate(-FULL_TURN, true); // The sensor motor will rotate less than 180 degree
                                            // (as we are using a gear)
      while (sensorMotor.isMoving()) { // Wait for the sensor to stop
        try {
          Thread.sleep(50);
        } catch (Exception e) {
        }
      }
      colorclassification.stop = true; // scan is done
      try {
        classificationThread.join();
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
      int can_color = colorclassification.color;
      if (!(can_color == 1 || can_color == 2 || can_color == 3 || can_color == 4)) {
        sensorMotor.setSpeed(ROTATE_SPEED);
        sensorMotor.rotate(FULL_TURN, false);
        return;
      }

      /* Start Weighting */
      sensorMotor.setSpeed(ROTATE_SPEED); // set the scanning speed
      sensorMotor.rotate(FULL_TURN / 2, false); // The sensor motor will rotate less than 180 degree
                                                // (as we are using a gear)
      Thread weightThread = new Thread(weightcan); // set a new thread to weight the can
      weightThread.start(); // the weighting thread starts
      try {
        weightThread.join(); // wait for the weighting process to finish
      } catch (InterruptedException e) {
        e.printStackTrace();
      }

      if (weightcan.heavy) {
        sound_time = 1000; // 1000 ms
      } else {
        sound_time = 500; // 500 ms
      }

      switch (can_color) {
        case 1: // blue
          Sound.playTone(TUNE, sound_time);
          break;
        case 2: // green
          for (int i = 0; i < 2; i++) { // beep 3 times upon arriving
            Sound.playTone(TUNE, sound_time);
            try {
              Thread.sleep(200);
            } catch (Exception e) {
            }
          }
          break;
        case 3: // yellow
          for (int i = 0; i < 3; i++) { // beep 3 times upon arriving
            Sound.playTone(TUNE, sound_time);
            try {
              Thread.sleep(200);
            } catch (Exception e) {
            }
          }
          break;
        case 4: // red
          for (int i = 0; i < 4; i++) { // beep 3 times upon arriving
            Sound.playTone(TUNE, sound_time);
            try {
              Thread.sleep(200);
            } catch (Exception e) {
            }
          }
          break;
      }
      get_can = true; // The robot is getting a can
    }
  }

  /**
   * 
   * This method simply moves the robot forward to a given distance
   * 
   * @param distance - The distance to move
   */
  void move(double distance) {

    leftMotor.setSpeed(FORWARD_SPEED + MOVE_ADJ);
    rightMotor.setSpeed(FORWARD_SPEED + MOVE_ADJ);

    leftMotor.rotate(convertDistance(leftRadius, distance), true);
    rightMotor.rotate(convertDistance(rightRadius, distance), true);

  }

  /**
   * <p>
   * This method is the forward method of the robot. The forward distance is calculated by the x and
   * y parameter passed to this method (Euclidean distance).
   * 
   * <p>
   * This method cannot be break
   * 
   * @param x - The x distance the robot should move
   * @param y - The y distance the robot should move
   */
  void forward(double x, double y) {

    distance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)); // The travel distance
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.rotate(convertDistance(leftRadius, distance), true);
    rightMotor.rotate(convertDistance(rightRadius, distance), false);

  }

  /**
   * <p>
   * This method is the approaching can method of the robot. The robot will forward a certain
   * distance to approach the can.
   * 
   * <p>
   * This method cannot be break
   * 
   * @param x - The x distance the robot should move
   * @param y - The y distance the robot should move
   */
  void approachCan(double distance) {

    leftMotor.setSpeed(FORWARD_SPEED - APPROACH_ADJ);
    rightMotor.setSpeed(FORWARD_SPEED - APPROACH_ADJ);
    leftMotor.rotate(convertDistance(leftRadius, distance), true);
    rightMotor.rotate(convertDistance(rightRadius, distance), false);

  }

  /**
   * <p>
   * This method is the back method of the robot. The forward distance is calculated by the x and y
   * parameter passed to this method (Euclidean distance). And the robot will travel back this
   * distance.
   * 
   * <p>
   * This method cannot be break
   * 
   * @param x - The x distance the robot should move
   * @param y - The y distance the robot should move
   */
  void back(double x, double y) {

    distance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)); // The travel distance

    leftMotor.setSpeed(FORWARD_SPEED + BACK_ADJ);
    rightMotor.setSpeed(FORWARD_SPEED + BACK_ADJ);
    leftMotor.rotate(-convertDistance(leftRadius, distance), true);
    rightMotor.rotate(-convertDistance(rightRadius, distance), false);

  }

  /**
   * <p>
   * This method is the back method of the robot. The forward distance is calculated by the x and y
   * parameter passed to this method (Euclidean distance). And the robot will travel back this
   * distance.
   * 
   * <p>
   * This method cannot be break
   * 
   * @param x - The x distance the robot should move
   * @param y - The y distance the robot should move
   */
  void backTo(double x, double y) {

    /* Calculate move angle and distance */
    lastx = odometer.getXYT()[0]; // The last x position of the robot
    lasty = odometer.getXYT()[1]; // The last y position of the robot

    distance = Math.sqrt(Math.pow(x - lastx, 2) + Math.pow(y - lasty, 2)); // The travel distance

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.rotate(-convertDistance(leftRadius, distance), true);
    rightMotor.rotate(-convertDistance(rightRadius, distance), false);

  }

  /**
   * This method implements the round search of the can. The robot will self-rotate (up to 360
   * degrees) until it sees a can, and then it calls the goTo(s, y) method to approach and detect
   * the can.
   * 
   * @param x - The x coordinate of the robot when it is doing round search
   * @param y - The y coordinate of the robot when it is doing round search
   * @param angle - The turning angle
   * @param range - The searching range, 1 for 1 tile
   */
  void roundSearch(double x, double y, double angle, int range) {
    round_detect = 0;
    angles[0] = angles[1] = angles[2] = 0;
    distances[0] = distances[1] = distances[2] = 0;

    end_angle = odometer.getXYT()[2] + angle;

    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.rotate(convertAngle(leftRadius, track, angle), true);
    rightMotor.rotate(-convertAngle(rightRadius, track, angle), true);
    // The true is to ensure the method can be interrupted.

    while (leftMotor.isMoving() || rightMotor.isMoving()) {
      round_detect = colorclassification.median_filter();
      if (angles[0] == 0 && round_detect <= range * TILE_SIZE) {
        angles[0] = odometer.getXYT()[2];
        distances[0] = round_detect;
      }
      if (angles[0] != 0 && round_detect > distances[0]) {
        angles[1] = odometer.getXYT()[2];
        distances[1] = round_detect;
      }

      if (angles[0] != 0 && angles[1] != 0) {
        if ((angles[1] - angles[0] < 5) || (angles[1] - angles[0] < -355)) {
          angles[0] = angles[1] = 0;
          distances[0] = distances[1] = 0;
          continue;
        }
        // Stop the motors and calculate the angle and distance of the can
        angles[2] = (angles[0] + angles[1]) / 2;
        if (angles[2] >= FULL_TURN) {
          angles[2] -= FULL_TURN;
        }
        distances[2] = (distances[0] + distances[1]) / 2;
        if (!inWall(distances[2], angles[2])) {
          angles[0] = angles[1] = 0;
          distances[0] = distances[1] = 0;
          continue;
        }
        leftMotor.stop(true);
        rightMotor.stop(false);
        turnTo(angles[2]); // Turn towards the can
        goTo(distances[2] * 1.5); // Go towards the can
        backTo(x, y);
        if (get_can) { // If this is a can
          break;
        } else {
          roundSearch(x, y, end_angle - odometer.getXYT()[2], range);
        }
      }
    }

  }

  /**
   * This method will return true for the destination within the wall, false for the destination is
   * out of the wall, so that the robot will not start traveling
   * 
   * @param distance - The distance the robot will travel
   * @param angle - The angle of the robot when it begin travel
   * @return if the destination is within the wall
   */
  boolean inWall(double distance, double angle) {
    double x = odometer.getXYT()[0];
    double y = odometer.getXYT()[1];
    double dx = distance * Math.sin(Math.toRadians(angle));
    double dy = distance * Math.cos(Math.toRadians(angle));
    if (x + dx > (15 * TILE_SIZE - ERROR) || x + dx < (0 * TILE_SIZE + ERROR)
        || y + dy > (9 * TILE_SIZE - ERROR) || y + dy < (0 * TILE_SIZE + ERROR)) {
      return false;
    } else {
      return true;
    }
  }

  /**
   * <p>
   * This method causes the robot to turn (on point) to the absolute heading theta. This method
   * should turn a MINIMAL angle to its target.
   * 
   * <p>
   * This method cannot be break
   * 
   * @param angle - The angle that the robot should rotate to
   */
  void turnTo(double angle) {

    lasttheta = odometer.getXYT()[2]; // Update the last theta of the robot
    theta = angle - lasttheta; // The angle that the robot should actually rotate

    if (theta > 180) { // Convert the angle from MAXIMAL to MINIMAL
      theta = -(360 - theta);
    }
    if (theta < -180) { // Convert the angle from MAXIMAL to MINIMAL
      theta = 360 + theta;
    }

    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    leftMotor.rotate(convertAngle(leftRadius, track, theta), true);
    rightMotor.rotate(-convertAngle(rightRadius, track, theta), false);

  }

  /**
   * <p>
   * This method turns the robot an angle passed by the method call. And the method can be
   * interrupted if the caller decide the turning angle is enough.
   * 
   * <p>
   * This method can be break
   * 
   * @param theta - The angle to turn
   */
  void turn(double theta) {

    leftMotor.setSpeed(ROTATE_SPEED + TURN_ADJ);
    rightMotor.setSpeed(ROTATE_SPEED + TURN_ADJ);

    leftMotor.rotate(convertAngle(leftRadius, track, theta), true);
    rightMotor.rotate(-convertAngle(rightRadius, track, theta), true); // The true is to ensure the
                                                                       // method can be interrupted.

  }

  /**
   * <p>
   * This method rotates the robot an angle passed by the method call. And the method cannot be
   * interrupted and the robot will make sure to finish rotating the angle.
   * 
   * <p>
   * This method cannot be break
   * 
   * @param theta - The angle to turn
   */
  void rotate(double theta) {

    leftMotor.setSpeed(ROTATE_SPEED + ROTATE_ADJ);
    rightMotor.setSpeed(ROTATE_SPEED + ROTATE_ADJ);

    leftMotor.rotate(convertAngle(leftRadius, track, theta), true);
    rightMotor.rotate(-convertAngle(rightRadius, track, theta), false);
    // The false is to ensure the rotation finish before continuing.

  }

  /**
   * This method returns true if another thread has called travelTo(), forward(), back(), turn(),
   * rotate() or turnTo() and the method has yet to return; false otherwise.
   * 
   * @return - If it the robot is navigating
   */
  boolean isNavigating() {

    if (leftMotor.isMoving() || rightMotor.isMoving()) {
      return true;
    } else {
      return false;
    }

  }

  /**
   * This method allows the conversion of a distance to the total rotation of each wheel need to
   * cover that distance.
   * 
   * @param radius - wheel radius
   * @param distance - traveling distance
   * @return - The converted distance
   */
  private static int convertDistance(double radius, double distance) {
    // convert from radius and distance to distance
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  /**
   * @param radius - wheel radius
   * @param width - track width
   * @param angle - turning angle
   * @return - The distance calculated by the angle
   */
  private static int convertAngle(double radius, double width, double angle) {
    // convert from radius angle and width to a distance
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

}
