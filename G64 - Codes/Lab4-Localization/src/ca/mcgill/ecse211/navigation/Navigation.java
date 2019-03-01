package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.lab4.Lab4;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;

/**
 * This class calculate the angle and distance needed from current location to each of the
 * destination x,y coordinate and perform the navigation based on the map.
 * 
 * @author Rintaro Nomura
 * @author Laurie Zaccarin
 */

public class Navigation {

  /**
   * Choosing which map will be used.
   */
  private int[] MAP = {0, 0};

  /**
   * An instance of odometer.
   */
  private static Odometer odometer;


  public Navigation(Odometer odometer) {
    Navigation.odometer = odometer;
  }

  /**
   * Default method runs when the thread starts. Shows the destiation coordinate on the display and
   * triggers the navigation.
   */
  public void navigate() {
    Lab4.leftMotor.stop();
    Lab4.rightMotor.stop();
    LocalEV3.get().getTextLCD().drawString("heading: " + MAP[0] + ", " + MAP[1], 0, 4);
    travelTo(MAP[0], MAP[1]);
    // Flag
    Lab4.completed = true;
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
    double dx = (x * Lab4.TILE_SIZE) - currentX;
    double dy = (y * Lab4.TILE_SIZE) - currentY;
    double distance = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));

    // calculate direction
    double theta = Math.atan2(dx, dy) - currentTheta;

    // adjust the theta to eliminate the unnecessary 360 degrees turn
    if (theta <= -Math.PI) {
      theta = theta + Math.PI * 2;
    } else if (theta > Math.PI) {
      theta = theta - Math.PI * 2;
    }

    // perform the turn
    Lab4.turnTo(theta);

    // move forward
    Lab4.leftMotor.setSpeed(Lab4.FORWARD_SPEED);
    Lab4.rightMotor.setSpeed(Lab4.FORWARD_SPEED);
    Lab4.rightMotor.rotate(Lab4.convertDistance(Lab4.WHEEL_RAD, distance), true);
    Lab4.leftMotor.rotate(Lab4.convertDistance(Lab4.WHEEL_RAD, distance), false);
    Lab4.leftMotor.stop(true);
    Lab4.rightMotor.stop(true);

    // finally, beep and increase the counter
    Sound.setVolume(20);
    Sound.beep();
  }
}
