package ca.mcgill.ecse211.localizer;

import ca.mcgill.ecse211.lab4.Lab4;
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;

public class LightLocalizer {
  private Odometer odometer;

  private double sensorValueOld = 0;

  public LightLocalizer(Odometer odo) {
    this.odometer = odo;
  }

  /**
   * This method first makes the robot move north until it crosses a first line and stops. The robot then turns 90 degrees
   * and does the same with the x axis. Once this is done, the robot is positioned at the right place on the tile and begins 
   * a 360 degree rotation on itself to detect all four lines. From this position it is possible to determine a x and y theta. 
   * Using simple trigonometry the value of x and y can be calculated using this information. Finally, the angle theta is determined
   * by manipulation the value of theta y.
   */

  public void localize() {
    // Currently facing North. Move until hitting first line.
    moveUntilLine();

    // Turn right and do the similar thing with x-axis
    Lab4.turnTo(Math.PI / 2);
    moveUntilLine();
    
    Lab4.rightMotor.stop(true);
    Lab4.leftMotor.stop();

    // Rotate around to detect 4 lines
    Lab4.rightMotor.setSpeed(Lab4.ROTATE_SPEED);
    Lab4.leftMotor.setSpeed(Lab4.ROTATE_SPEED);
    Lab4.leftMotor.rotate(-Lab4.convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 360), true);
    Lab4.rightMotor.rotate(Lab4.convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 360), true);
        
    int lineCount = 0;
    double[] angles = {0, 0, 0, 0};

    while (true) {
      try {
        Thread.sleep(100);
      } catch (Exception e) {
      } // Poor man's timed sampling

      // Obtain the value from the color sensor
      Lab4.colorValue.fetchSample(Lab4.colorData, 0);
      double value = Lab4.colorData[0];

      // setting up differential filter
      double dydx = value - this.sensorValueOld;
      this.sensorValueOld = value;

      LocalEV3.get().getTextLCD().drawString("LS Reading: " + value, 0, 4);

      if (dydx < -0.050) {
        Sound.beep();
        angles[lineCount] = odometer.getXYT()[2];
        lineCount++;
      }

      if (lineCount == 4) {
        break;
      }
    }
    
    Sound.beep();
    Sound.beep();

    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    
    // Turn back to face north
    Lab4.turnTo(-Math.PI / 2);
    
    // Compute Trig
    double dy = angles[2] - angles[0];
    double dx = angles[3] - angles[1];

    double absx = Lab4.SENSOR_OFFSET * Math.cos(Math.PI * dx / (2 * 180)) * -1;
    double absy = Lab4.SENSOR_OFFSET * Math.cos(Math.PI * dy / (2 * 180)) * -1;

    double dtheta = 180 - dy / 2 - angles[0];
    
    LocalEV3.get().getTextLCD().clear();
    LocalEV3.get().getTextLCD().drawString("dx: " + (int) dx + " Ax:" + (int) absx, 0, 3);
    LocalEV3.get().getTextLCD().drawString("dy: " + (int) dy + " Ay:" + (int) absy, 0, 4);
    LocalEV3.get().getTextLCD().drawString("dT: " + (int) dtheta, 0, 5);
    
    // Overwrite odometer
    //odometer.update(absx, absy, dtheta);
    odometer.setX(absx);
    odometer.setY(absy);
    odometer.setTheta(0);
    
    Sound.beep();
    Sound.beep();
  }


  /**
   *This method first makes the robot move forward, with both motors advancing at the same speed. Once a line is reached,
   *the value of the color sensor is obtained and a differential filter is set up (reducing noise). Finally, the robot 
   *moves backwards.
   */

  private void moveUntilLine() {
    // Satrt moving forward
    Lab4.rightMotor.setSpeed(Lab4.FORWARD_SPEED);
    Lab4.leftMotor.setSpeed(Lab4.FORWARD_SPEED);
    Lab4.leftMotor.forward();
    Lab4.rightMotor.forward();

    // Stop when reaching first line
    while (true) {
      // Obtain the value from the color sensor
      Lab4.colorValue.fetchSample(Lab4.colorData, 0);
      double value = Lab4.colorData[0];

      // setting up differential filter
      double dydx = value - this.sensorValueOld;
      this.sensorValueOld = value;

      LocalEV3.get().getTextLCD().drawString("LS Reading: " + value, 0, 4);

      if (dydx < -0.050) {
        Lab4.leftMotor.stop(true);
        Lab4.rightMotor.stop();
        Sound.beep();
        break;
      }
      try {
        Thread.sleep(100);
      } catch (Exception e) {
      } // Poor man's timed sampling
    }

    // Move backward for a set offset
    Lab4.rightMotor.rotate(-Lab4.convertDistance(Lab4.WHEEL_RAD, Lab4.TILE_SIZE / 2 + 2), true);
    Lab4.leftMotor.rotate(-Lab4.convertDistance(Lab4.WHEEL_RAD, Lab4.TILE_SIZE / 2 + 2), false);
    Lab4.leftMotor.stop();
    Lab4.rightMotor.stop();
  }
}
