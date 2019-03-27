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
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class USCanDetectionTest {

  private static final int motorSpeed = 80; // Speed of slower rotating wheel (deg/sec)

  /**
   * EV3 Related Objects
   */
  private static final Port usPort = LocalEV3.get().getPort("S3");  // Ultrasonic Sensor is connected to
  public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

  public static SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
  public static SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from this instance
  public static float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are returned
  public static List<Double> list = new ArrayList<>();

  
  public static void usSensorTest () throws FileNotFoundException, UnsupportedEncodingException, InterruptedException {
    
    
    int samples = 500;
    
      // Initialize writer
      PrintWriter writer = new PrintWriter("usdata.csv", "UTF-8");;

      // Let the robot start moving clockwise
      leftMotor.setSpeed(motorSpeed);
      rightMotor.setSpeed(motorSpeed);
      leftMotor.forward();
      rightMotor.backward();    

      // Sensor Initialization
      SensorPoller poller = new SensorPoller(usDistance, usData);
      poller.start();

      // Collect data from poller, apply median filter, store the value
      while (list.size() != samples) {    // 100 value
        double[] arr = new double[5]; // temp storage for readings
        for (int i = 0; i < 5; i++) { // take 5 readings
          arr[i] = usData[0] * 100.0; // signal amplification
        }
        Arrays.sort(arr); // sort readings
        list.add(arr[2]); // take median value, add to list
        
        try {
          Thread.sleep(50);
        } catch (Exception e) {
        } // Poor man's timed sampling
      }

      // Output
      try {
        System.out.println("Writing...");
        for(int i=0; i < list.size(); i++) {
          writer.write(i + "," + list.get(i) + "\n");
          Thread.sleep(100);
        }
      } finally {
        writer.close();
      }
      
      Sound.beep();
    }

}
