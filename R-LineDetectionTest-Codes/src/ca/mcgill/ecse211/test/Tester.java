package ca.mcgill.ecse211.test;

import java.io.FileNotFoundException;
import java.io.UnsupportedEncodingException;
import lejos.hardware.Button;

public class Tester {
  
   /**
    * Setup your test here: 
   * USCanDetection = 0
   * LineDetection = 1
   * 
   */
  static int option = 1;

  /**
   * Main method: waits until the user press any button. 
   * Performs the desired test. 
   * Exits automatically. 
   * @param args
   */
  public static void main(String[] args) {
    Button.waitForAnyPress();

    switch (option) {
      case 0:
        try {
          USCanDetectionTest.usSensorTest();
        } catch (FileNotFoundException | UnsupportedEncodingException | InterruptedException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
      case 1:
        try {
          LineDetectionTest.lineSensorTest();
        } catch (FileNotFoundException | UnsupportedEncodingException | InterruptedException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }   
    }

    System.exit(0);
  }

}
