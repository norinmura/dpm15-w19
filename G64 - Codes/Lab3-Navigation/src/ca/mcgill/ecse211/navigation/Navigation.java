package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.lab3.Display;
import ca.mcgill.ecse211.navigation.NavigationExceptions;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

/**
 * This class calculate the angle and distance needed from current location to each of the
 * destination x,y coordinate and perform the navigation based on the map.
 * 
 * @author Rintaro Nomura
 * @author Laurie Zaccarin
 */

public class Navigation extends Thread {
  // Constants

  /**
   * The forward speed.
   */
  private static final int FORWARD_SPEED = 150;

  /**
   * The rotating speed.
   */
  private static final int ROTATE_SPEED = 50;

  /**
   * The tile size.
   */
  private static final double TILE_SIZE = 30.48;

  /**
   * The wheel radius.
   */
  public static final double WHEEL_RAD = 2.1;

  /**
   * The track width.
   */
  public static final double TRACK = 14.425;

  /**
   * The list of possible maps.
   */
  private int[][][] MAPS = {{{0, 2}, {1, 1}, {2, 2}, {2, 1}, {1, 0}}, // 0
      {{1, 1}, {0, 2}, {2, 2}, {2, 1}, {1, 0}}, // 1
      {{1, 0}, {2, 1}, {2, 2}, {0, 2}, {1, 1}}, // 2
      {{0, 1}, {1, 2}, {1, 0}, {2, 1}, {2, 2}}, // 3
      {{2, 1}, {1, 1}, {1, 2}, {2, 0}}};

  /**
   * Choosing which map will be used.
   */
  private int[][] MAP = MAPS[0];

  /**
   * An instance of odometer.
   */
  private static Odometer odometer;

  /**
   * An instance of the left motor.
   */
  private static EV3LargeRegulatedMotor leftMotor;

  /**
   * An instance of the right motor.
   */
  private static EV3LargeRegulatedMotor rightMotor;

  /**
   * The map counter.
   */
  private int mapCount;

  /**
   * A boolean defining if the robot is avoiding an obstacle.
   */
  private boolean isAvoiding;

  private static Navigation nav = null; // Returned as singleton


  public Navigation(Odometer odometer, EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor) throws NavigationExceptions {
    Navigation.odometer = odometer;
    Navigation.leftMotor = leftMotor;
    Navigation.rightMotor = rightMotor;
  }

  /**
   * This method is meant to ensure only one instance of the navigation is used throughout the code.
   * 
   * @param leftMotor
   * @param rightMotor
   * @return new or existing Navigation Object
   * @throws Exception
   */
  public synchronized static Navigation getNavigation(Odometer odometer,
      EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor)
      throws NavigationExceptions {
    if (nav != null) { // Return existing object
      return nav;
    } else { // create object and return it
      nav = new Navigation(odometer, leftMotor, rightMotor);
      return nav;
    }
  }

  /**
   * This class is meant to return the existing Navigation Object. It is meant to be used only if an
   * navigation object has been created
   * 
   * @return error if no previous navigation exists
   */
  public synchronized static Navigation getNavigation() throws NavigationExceptions {
    if (nav == null) {
      throw new NavigationExceptions("No previous Navigation object exits.");
    }
    return nav;
  }

  public void run() {

    // Wait until sensor is ready (around 3s).
    try {
      Thread.sleep(3000);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }

    odometer.update(0, 0, 0); // reset value
    leftMotor.stop();
    rightMotor.stop();

    // spawn a new Thread to run the obstacle avoider without interrupting navigation
    (new Thread() {
      public void run() {
        while (true) {
          if (odometer.getDistance() < 10) {
            isAvoiding = true;
            int mapCount_bak = mapCount;
            avoidObstacle();
            mapCount = mapCount_bak;
            isAvoiding = false;
          }
        }
      }
    }).start();

    while (true) {
      while (mapCount < 4 && isAvoiding == false) {
        LocalEV3.get().getTextLCD()
            .drawString("heading: " + MAP[mapCount][0] + ", " + MAP[mapCount][1], 0, 4);
        travelTo(MAP[mapCount][0], MAP[mapCount][1]);
      }
    }
  }

  /**
   * This method is used when an obstacle has been identified. The method checks in which direction
   * the robot is going when it encounters the obstacle. Depending on the direction and location, it
   * either turns left or right to avoid the obstacle.
   */
  public void avoidObstacle() {
    // Wait for a while until theta is stable
    try {
      Thread.sleep(500);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    if (mapCount < 5) {
      double[] odoData = odometer.getXYT();
      double currentX = odoData[0];
      double currentY = odoData[1];
      double currentT = odoData[2];

      leftMotor.stop(true);
      rightMotor.stop(true);

      int turnConst = 1; // 1 = turn right, -1 = turn left

      if (45 < currentT && currentT < 135) { // if heading east
        if (currentY < TILE_SIZE - 3) { // if at lower area, turn left
          turnConst = -1;
        }
      } else if (-135 < currentT && currentT < -45) { // if heading west
        if (currentY > TILE_SIZE - 3) {
          turnConst = -1; // if at higher area, turn left
        }
      } else if (-45 < currentT && currentT < 45) {
        if (currentX > TILE_SIZE - 3) {
          turnConst = -1;
        }
      } else if (-225 < currentT && currentT < -135) {
        if (currentX < TILE_SIZE - 3) {
          turnConst = -1;
        }
      }

      // Avoid obstacles
      turnTo(turnConst * Math.PI / 2);
      avoidingMove(30);
      turnTo(-turnConst * Math.PI / 2);
      avoidingMove(30);
    }
  }

  /**
   * This method performs the avoiding obstacle.
   * 
   * @param dist
   */

  public void avoidingMove(int dist) {
    // move forward
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    rightMotor.rotate(convertDistance(WHEEL_RAD, dist), true);
    leftMotor.rotate(convertDistance(WHEEL_RAD, dist), false);
  }

  /**
   * This method first obtains from the odometer the location of the robot. It then compares the
   * location to the destination and calculates the distance that must be traveled along with the
   * direction. The robot then turns to face the correct direction according to the shortest turn
   * possible. The robot moves forward to its destination. Once the sensor identifies the next line,
   * there is a beep and the counter is increased.
   */

  public void travelTo(double x, double y) {
    // Obtain the current location and angle
    double[] odoData = odometer.getXYT();
    double currentX = odoData[0];
    double currentY = odoData[1];
    double currentTheta = odoData[2] * Math.PI / 180; // convert to radian so it's easier to work on
    // math

    // calculate the distance required to travel
    double dx = (x * TILE_SIZE) - currentX;
    double dy = (y * TILE_SIZE) - currentY;
    double distance = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));

    // calculate direction
    double theta = Math.atan2(dx, dy) - currentTheta;

    // adjust the theta to eliminate the unnecessary 360 degrees turn
    if (theta <= -Math.PI) {
      theta = theta + Math.PI * 2;
    } else if (theta > Math.PI) {
      theta = theta - Math.PI * 2;
    }

    if (!this.isAvoiding) {
      // perform the turn
      turnTo(theta);

      // move forward
      leftMotor.setSpeed(FORWARD_SPEED);
      rightMotor.setSpeed(FORWARD_SPEED);
      rightMotor.rotate(convertDistance(WHEEL_RAD, distance), true);
      leftMotor.rotate(convertDistance(WHEEL_RAD, distance), false);
      leftMotor.stop(true);
      rightMotor.stop(true);

      // finally, beep and increase the counter
      Sound.setVolume(20);
      Sound.beep();
      mapCount++;
    }
  }

  /**
   * This method determines which way the robot must turn depending on the angle of theta, then
   * perform the turn.
   */

  public void turnTo(double theta) {
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    theta = theta * 180.0 / Math.PI;

    // turn to the left if angle is negative
    if (theta < 0) {
      leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, -theta), true);
      rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, -theta), false);
    } else { // turn to the right if angle is positive
      leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
      rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);
    }

  }


  private int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  private int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

}
