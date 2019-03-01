// Lab3.java
package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.navigation.Navigation;
import ca.mcgill.ecse211.navigation.NavigationExceptions;
import ca.mcgill.ecse211.navigation.UltrasonicController;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class Lab3 {

  // Motor Objects, and Robot related parameters
  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();



  public static final double WHEEL_RAD = 2.10;
  public static final double TRACK = 14.465;

  public static void main(String[] args) throws OdometerExceptions, NavigationExceptions {

    int buttonChoice;

    // Odometer related objects
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
    Navigation navigation = Navigation.getNavigation(odometer, leftMotor, rightMotor);
    UltrasonicController avoider = new UltrasonicController();
    Display odometryDisplay = new Display(lcd);


    do {
      // clear the display
      lcd.clear();

      // ask the user whether odometery correction should be run or not
      lcd.drawString("< Left | Right >", 0, 0);
      lcd.drawString("  No   | with   ", 0, 1);
      lcd.drawString(" Obst- | Obst-  ", 0, 2);
      lcd.drawString(" acles | acles  ", 0, 3);
      lcd.drawString("       |        ", 0, 4);

      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);


    // Start odometer and display threads
    Thread odoThread = new Thread(odometer);
    odoThread.start();
    Thread odoDisplayThread = new Thread(odometryDisplay);
    odoDisplayThread.start();

    // Start navigation
    Thread navThread = new Thread(navigation);
    navThread.start();

    // Calling new thread depending on the choice
    if (buttonChoice == Button.ID_RIGHT) { // No obstacles
      Thread avoiderThread = new Thread(avoider);
      avoiderThread.start();


    }



    lcd.clear();



    // Wait here forever until button pressed to terminate the robot
    Button.waitForAnyPress();
    System.exit(0);
  }
}
