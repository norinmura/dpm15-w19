// Lab2.java
package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.localizer.LightLocalizer;
import ca.mcgill.ecse211.localizer.UltrasonicLocalizer;
import ca.mcgill.ecse211.localizer.UltrasonicPoller;
import ca.mcgill.ecse211.navigation.Navigation;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab4 {


  /**
   * The constants
   */
  public static int FORWARD_SPEED = 150;
  public static int ROTATE_SPEED = 60;
  public static int WALL_DISTANCE = 30;
  public static int WALL_ERROR = 3;
  public static final double TILE_SIZE = 30.48;
  public static final double WHEEL_RAD = 2.1;
  public static final double TRACK = 14.425;
  public static final double SENSOR_OFFSET = 12.725;

  public static boolean completed;


  /* 
   * EV3 parts objects, and robot related parameters
   */
  public static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  public static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

  private static final Port colorPort = LocalEV3.get().getPort("S1");
  public static EV3ColorSensor colorSensor = new EV3ColorSensor(colorPort);
  public static SampleProvider colorValue = colorSensor.getMode("Red");
  public static float[] colorData = new float[colorValue.sampleSize()];

  private static final Port usPort = LocalEV3.get().getPort("S2");
  static SensorModes usSensor = new EV3UltrasonicSensor(usPort);
  static SampleProvider usDistance = usSensor.getMode("Distance");
  static float[] usData = new float[usDistance.sampleSize()];

  private static final TextLCD lcd = LocalEV3.get().getTextLCD();



  public static void main(String[] args) throws OdometerExceptions {

    int buttonChoice;

    // Odometer related object
    final Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
    Display odometryDisplay = new Display(lcd);
    final LightLocalizer lightLocalizer = new LightLocalizer(odometer);
    final Navigation navigator = new Navigation(odometer);
    final UltrasonicLocalizer usLocalizer;


    // EV3 Parts related objects


    do {
      // clear the display
      lcd.clear();

      // ask the user whether the motors should drive in a square or float
      lcd.drawString("< Left | Right >", 0, 0);
      lcd.drawString("       |        ", 0, 1);
      lcd.drawString("Falling| Rising ", 0, 2);
      lcd.drawString("edge   |   edge ", 0, 3);
      lcd.drawString("       |        ", 0, 4);

      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

    // Display changes in position as wheels are (manually) moved
    Thread odoThread = new Thread(odometer);
    odoThread.start();
    Thread odoDisplayThread = new Thread(odometryDisplay);
    odoDisplayThread.start();


    if (buttonChoice == Button.ID_LEFT) { // Falling Edge
      usLocalizer =
          new UltrasonicLocalizer(odometer, UltrasonicLocalizer.LocalizationType.FALLING_EDGE);
    } else { // Rising Edge
      usLocalizer =
          new UltrasonicLocalizer(odometer, UltrasonicLocalizer.LocalizationType.RISING_EDGE);
    }


    UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, usLocalizer);
    usPoller.start();

    // spawn a new Thread to avoid methods from blocking main thread
    Thread mainThread = new Thread() {
      public void run() {
        // Run ultrasonic sensor localization
        LocalEV3.get().getTextLCD().drawString("Current: UltrasonicLocalization", 0, 3);
        usLocalizer.localize();
        Button.waitForAnyPress(); // wait for TA

        // Run light sensor localization
        LocalEV3.get().getTextLCD().drawString("Current: LightLocalization", 0, 3);
        lightLocalizer.localize();
        // Navigate itself to (0,0)
        navigator.navigate();

        // Turn north
        Lab4.turnTo(-odometer.getXYT()[2]*Math.PI/180);
        
        // Set odometer to 0,0,0
        odometer.setXYT(0, 0, 0);
      }
    };

    mainThread.start();

    try {
      mainThread.join();
    } catch (InterruptedException e) {
      System.out.println(e);
    }

    Button.waitForAnyPress();
    System.exit(0);
  }



  public static void rotate(boolean reverse) {
    if (!reverse) {
      leftMotor.forward();
      rightMotor.backward();
    } else {
      leftMotor.backward();
      rightMotor.forward();
    }
  }

  /**
   * This method determines which way the robot must turn depending on the angle of theta, then
   * perform the turn.
   */

  public static void turnTo(double theta) {
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

  public static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  public static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

}
