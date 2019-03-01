package ca.mcgill.ecse211.localizer;

import ca.mcgill.ecse211.lab4.Lab4;
import ca.mcgill.ecse211.navigation.Navigation;
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class UltrasonicLocalizer implements UltrasonicController {
  // Constants
  private final int upper_limit = 50;
  private static final int FILTER_OUT = 10;

  // Instances
  private Odometer odo;

  // Variables
  public enum LocalizationType {
    FALLING_EDGE, RISING_EDGE
  };

  public double angleA;
  public double angleB;
  public int filterControl;
  public LocalizationType selected_type;

  public UltrasonicLocalizer(Odometer odo, LocalizationType type) {
    this.odo = odo;
    this.selected_type = type;
  }
  
  
  /**
   * This method first detects which localization type is used. If falling edge is used, then the robot
   * turns to face the wall and the turns way until it detects the first falling edge. The angle A is registered. The robot then faces 
   * the wall again, and turns away in order to detect another falling edge. The angle B is registered.
   * If rising edge is used, then the robot turns to face away from the wall, and then turns back until it detects the first rising edge,
   * registering angle A. The robot then faces away form the wall again and turns back, detecting another rising edge,
   * and registering angle B.
   * Then, the angles A and B are used in order to determine which way to orient the robot.
   * 
   * 
   */

  /**
   * This method first detects which localization type is used. If falling edge is used, then the robot
   * turns to face the wall and  it detects the first falling edge and registers an angle A. The robot then faces the wall again, and 
   * another falling edge is detected after and registers angle B.
   * If rising edge is used, then the robot turns to face the wall and it detects the first falling edge, registering angle A. The robot then faces the wall again, and 
   * another falling edge is detected after, registering angle B.
   * Then, the angles A and B are used in order to determine which way to orient the robot.
   * 
   * 
   */
  public void localize() {
    Lab4.leftMotor.setSpeed(Lab4.ROTATE_SPEED);
    Lab4.rightMotor.setSpeed(Lab4.ROTATE_SPEED);

    if (this.selected_type == LocalizationType.FALLING_EDGE) {
      // let the robot to face the wall to begin with
      detect_risingedge(false);

      // detect the first falling edge
      this.angleA = detect_fallingedge(false);
      
      Lab4.leftMotor.stop(true);;
      Lab4.rightMotor.stop();

      // let the robot to face the wall again to begin with
      detect_risingedge(true);

      // detect another falling edge
      this.angleB = detect_fallingedge(true);
      Lab4.leftMotor.stop(true);;
      Lab4.rightMotor.stop();
      
    } else {
      // let the robot not to face the wall to begin with
      detect_fallingedge(false);

      // detect the first rising edge
      this.angleA = detect_risingedge(false);
      Lab4.leftMotor.stop(true);;
      Lab4.rightMotor.stop();

      // let the robot to not facing the wall again to begin with
      detect_fallingedge(true);

      // detect another rising edge
      this.angleB = detect_risingedge(true);
      Lab4.leftMotor.stop(true);;
      Lab4.rightMotor.stop();
    }

    LocalEV3.get().getTextLCD().drawString("AngleA:" + this.angleA, 0, 4);
    LocalEV3.get().getTextLCD().drawString("AngleB:" + this.angleB, 0, 5);
    
    // Compute which way is more "East"
    if (angleB > angleA) {
      angleB = angleB - 360;
    }

    // Let the robot to face north
    double dTheta = angleA - (angleB + angleA) / 2 - 45;
    if (this.selected_type == LocalizationType.FALLING_EDGE) {
      dTheta += 180;
    }
    
    LocalEV3.get().getTextLCD().drawString("dTheta:" + dTheta, 0, 6);
    Lab4.turnTo(dTheta * Math.PI / 180);

    // Overwrite odometer's theta value
    odo.setTheta(0);
  }


  /**
   * This method detects if there is a falling edge. The robot is rotated until it is no longer facing any walls,
   * meaning it detects a falling edge.
   * 
   */


  public double detect_fallingedge(boolean reverse) {
    // rotate the robot until it detects the falling edge (i.e. until not facing walls)
    while (odo.getDistance() < Lab4.WALL_DISTANCE) {
      LocalEV3.get().getTextLCD().drawString("US Reading: " + odo.getDistance(), 0, 4);
      Lab4.rotate(reverse);
    }

    Sound.beep();
    return odo.getXYT()[2];
  }


  /**
   * This method detects if there is a rising edge. The robot is rotated until it is facing a wall,
   * meaning it detects a rising edge.
   * 
   */

  public double detect_risingedge(boolean reverse) {
    // rotate the robot until it detects the rising edge (i.e. until detects walls)
    while (odo.getDistance() > Lab4.WALL_DISTANCE) {
      LocalEV3.get().getTextLCD().drawString("US Reading: " + odo.getDistance(), 0, 4);
      Lab4.rotate(reverse);
    }

    Sound.beep();
    return odo.getXYT()[2];
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
      odo.setDistance(distance);

    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      odo.setDistance(distance);
    }
    
    try {
      Thread.sleep(20);
    } catch (Exception e) {
    } // Poor man's timed sampling
  }

  @Override
  public int readUSDistance() {
    return odo.getDistance();
  }

  public static void drive(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      double leftRadius, double rightRadius, double track) {


  }
}
