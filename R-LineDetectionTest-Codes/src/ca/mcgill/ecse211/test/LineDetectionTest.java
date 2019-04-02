package ca.mcgill.ecse211.test;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class LineDetectionTest {

  /**
   * Setup the constant parameters here
   * These parameters will be passed to SquareDriver.java
   */
  public static final int ACCELERATION = 3000;
  public static final int FORWARD_SPEED = 170;
  public static final int ROTATE_SPEED = 100;
  public static final double WHEEL_RAD = 2.1; // The radius of the wheel
  public static final double TRACK = 13.70; // The width of the robot measured
  public static final int FULL_TURN = 360; // 360 degree for a circle
  public static final double TILE_SIZE = 30.48; // The tile size used for demo
  
  
  

  /**
   * EV3 Related Objects
   * It assumes: 
   * Port A - Left Wheel Motor
   * Port B - Right Wheel Motor
   * Port S1 - Light sensor at the side of left wheel
   * Port S2 - Light sensor at the side of right wheel
   */
  public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

  private static final Port portColor1 = LocalEV3.get().getPort("S1"); // Light sensor port1
  private static final Port portColor2 = LocalEV3.get().getPort("S2"); // Light sensor port2

  // Color Sensor (Line Detection, Left)
  public static SensorModes myColor1 = new EV3ColorSensor(portColor1); // Get sensor instance
  public static SampleProvider myColorStatus1 = myColor1.getMode("Red"); // Get sample provider as "RGB"
  public static float[] sampleColor1 = new float[myColorStatus1.sampleSize()]; // Create a data buffer

  // Color Sensor (Line Detection, Right)
  public static SensorModes myColor2 = new EV3ColorSensor(portColor2); // Get sensor instance
  public static SampleProvider myColorStatus2 = myColor2.getMode("Red"); // Get sample provider as "RGB"
  public static  float[] sampleColor2 = new float[myColorStatus2.sampleSize()]; // Create a data buffer

  // ArrayList to store the sensor readings
  public static List<Integer> list1 = new ArrayList<>();
  public static List<Integer> list2 = new ArrayList<>();

  // Just to detect when the testing is done. 
  public static boolean isTesting = true;
  
  /**
   * This method first creates a thread to call drive() from SquareDriver.java to let the robot to move in a square. 
   * Meanwhile, it reads the value of the 2 light sensors (through the poller), apply median filter, and then stores 
   * them into 2 separate ArrayList. 
   * 
   * When SquareDriver thread finished its operation, this method attempts to read each value from the ArrayList and 
   * uses Writer to save the data as CSV, line by line. 
   * 
   * The format of CSV is: "i, SensorValue1, SensorValue2" where i indicates the reading is i-th element in the ArrayList. 
   * 
   * @throws FileNotFoundException
   * @throws UnsupportedEncodingException
   * @throws InterruptedException
   */
  public static void lineSensorTest () throws FileNotFoundException, UnsupportedEncodingException, InterruptedException {
    // Initialize writer
    PrintWriter writer = new PrintWriter("linedata.csv", "UTF-8");

    // Sensor poller Initialization
    SensorPoller poller1 = new SensorPoller(myColorStatus1, sampleColor1);
    SensorPoller poller2 = new SensorPoller(myColorStatus2, sampleColor2);

    poller1.start();
    poller2.start();

    // spawn a new Thread to avoid SquareDriver.drive() from blocking
    (new Thread() {
      public void run() {
        LineDetectionTest.isTesting = true;
        SquareDriver.drive(leftMotor, rightMotor, WHEEL_RAD, WHEEL_RAD, TRACK);
        LineDetectionTest.isTesting = false;
      }
    }).start();

    // Collect data from poller, apply median filter, store the value
    while (isTesting == true) {    // 100 value
      double[] arr = new double[5]; // temp storage for readings
      double[] arr2 = new double[5]; // temp storage for readings
      
      for (int i = 0; i < 5; i++) { // take 5 readings
        arr[i] = sampleColor1[0] * 100.0; // signal amplification
        arr2[i] = sampleColor2[0] * 100.0;
      }

      list1.add((int) arr[2]); // take median value, add to list
      list2.add((int) arr2[2]); // take median value, add to list
      
      try {
        Thread.sleep(10);
      } catch (Exception e) {
      } // Poor man's timed sampling
    }

    // Output
    try {
      System.out.println("Writing...");
      for(int i=0; i < Math.min(list1.size(), list2.size()); i++) {
        writer.write(i + "," + list1.get(i) + "," + list2.get(i) + "\n");
        Thread.sleep(100);
      }
    } finally {
      writer.close();
    }

    Sound.beep();
  }

}
