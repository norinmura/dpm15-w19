package ca.mcgill.ecse211.finalproject;

import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import java.util.Arrays;
import java.util.Map;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This is the main class of the program for the Final Project. Starting in a known corner, localize
 * to the grid, and perform a search in the prescribed area for a can of specified color. It will
 * then detect the can and identify its color. The prescribed area is given to us by a lower left
 * corner and an upper right corner.
 * 
 * @author Floria Peng
 *
 */
public class FinalProject {

  /* STATIC FIELDS */

  // ** Set these as appropriate for your team and current situation **
  private static final String SERVER_IP = "192.168.2.31";
  private static final int TEAM_NUMBER = 15; // Team 15

  // Enable/disable printing of debug info from the WiFi class
  private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;

  // Motors
  // Instantiate motors; left right and sensor motor to rotate the color sensor.
  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
  private static final EV3MediumRegulatedMotor sensorMotor =
      new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));
  private static final UnregulatedMotor weightMotor =
      new UnregulatedMotor(LocalEV3.get().getPort("D"));

  /* Ports */
  private static final Port usPort = LocalEV3.get().getPort("S3"); // Ultrasonic sensor port
  private static final Port portColor1 = LocalEV3.get().getPort("S1"); // Light sensor port1
  private static final Port portColor2 = LocalEV3.get().getPort("S2"); // Light sensor port2
  private static final Port colorPort = LocalEV3.get().getPort("S4"); // Light sensor port for color
                                                                      // detection
  /* LCD */
  private static final TextLCD lcd = LocalEV3.get().getTextLCD(); // The LCD display

  /* CONSTANTS */
  public static final double WHEEL_RAD = 2.1; // The radius of the wheel
  public static final double TRACK = 11.93; // The width of the robot measured
  public static final int FULL_TURN = 360; // 360 degree for a circle
  public static final double TILE_SIZE = 30.48; // The tile size used for demo

  /**
   * The main method for the Final Project. This class will start the threads used for the program.
   * 
   * @param args
   * 
   * @throws OdometerExceptions
   * @throws InterruptedException
   * 
   */
  @SuppressWarnings({"resource", "rawtypes"})
  public static void main(String[] args) throws OdometerExceptions, InterruptedException {

    System.out.println("Running..");

    /* WiFi connection */

    // Initialize WifiConnection class
    WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

    // Initializing the parameters
    int redTeam, greenTeam, corner, ll_x, ll_y, ur_x, ur_y, tn_ll_x, tn_ll_y, tn_ur_x, tn_ur_y,
        sz_ll_x, sz_ll_y, sz_ur_x, sz_ur_y, island_ll_x, island_ll_y, island_ur_x, island_ur_y;
    redTeam = greenTeam =
        corner = ll_x = ll_y = ur_x = ur_y = tn_ll_x = tn_ll_y = tn_ur_x = tn_ur_y = sz_ll_x =
            sz_ll_y = sz_ur_x = sz_ur_y = island_ll_x = island_ll_y = island_ur_x = island_ur_y = 0;

    int target_color = 0;

    // Connect to server and get the data, catching any errors that might occur
    try {
      /*
       * getData() will connect to the server and wait until the user/TA presses the "Start" button
       * in the GUI on their laptop with the data filled in. Once it's waiting, you can kill it by
       * pressing the upper left hand corner button (back/escape) on the EV3. getData() will throw
       * exceptions if it can't connect to the server (e.g. wrong IP address, server not running on
       * laptop, not connected to WiFi router, etc.). It will also throw an exception if it connects
       * but receives corrupted data or a message from the server saying something went wrong. For
       * example, if TEAM_NUMBER is set to 1 above but the server expects teams 17 and 5, this robot
       * will receive a message saying an invalid team number was specified and getData() will throw
       * an exception letting you know.
       */
      Map data = conn.getData();

      // Example 1: Print out all received data
      System.out.println("Map:\n" + data);

      // Example 2 : Print out specific values
      redTeam = ((Long) data.get("RedTeam")).intValue();
      System.out.println("Red Team: " + redTeam);

      greenTeam = ((Long) data.get("GreenTeam")).intValue();
      System.out.println("Green Team: " + greenTeam);

      target_color = greenTeam;

      if (redTeam == TEAM_NUMBER) {
        // Red team's starting corner
        corner = ((Long) data.get("RedCorner")).intValue();
        System.out.println("RedCorner: " + corner);

        // Lower left hand corner of Red Zone
        ll_x = ((Long) data.get("Red_LL_x")).intValue();
        System.out.println("Red_LL_x: " + ll_x);
        ll_y = ((Long) data.get("Red_LL_y")).intValue();
        System.out.println("Red_LL_y: " + ll_y);
        // Upper right hand corner of Red Zone
        ur_x = ((Long) data.get("Red_UR_x")).intValue();
        System.out.println("Red_UR_x: " + ur_x);
        ur_y = ((Long) data.get("Red_UR_y")).intValue();
        System.out.println("Red_UR_y: " + ur_y);

        // Lower left hand corner of the red tunnel footprint
        tn_ll_x = ((Long) data.get("TNR_LL_x")).intValue();
        System.out.println("TNR_LL_x: " + tn_ll_x);
        tn_ll_y = ((Long) data.get("TNR_LL_y")).intValue();
        System.out.println("TNR_LL_y: " + tn_ll_y);
        // Upper right hand corner of the red tunnel footprint
        tn_ur_x = ((Long) data.get("TNR_UR_x")).intValue();
        System.out.println("TNR_UR_x: " + tn_ur_x);
        tn_ur_y = ((Long) data.get("TNR_UR_y")).intValue();
        System.out.println("TNR_UR_y: " + tn_ur_y);

        // Lower left hand corner of the red player search zone
        sz_ll_x = ((Long) data.get("SZR_LL_x")).intValue();
        System.out.println("SZR_LL_x: " + sz_ll_x);
        sz_ll_y = ((Long) data.get("SZR_LL_y")).intValue();
        System.out.println("SZR_LL_y: " + sz_ll_y);
        // Upper right hand corner of the red player search zone
        sz_ur_x = ((Long) data.get("SZR_UR_x")).intValue();
        System.out.println("SZR_UR_x: " + sz_ur_x);
        sz_ur_y = ((Long) data.get("SZR_UR_y")).intValue();
        System.out.println("SZR_UR_y: " + sz_ur_y);

        // Lower left hand corner of the Island
        island_ll_x = ((Long) data.get("Island_LL_x")).intValue();
        System.out.println("Island_LL_x: " + island_ll_x);
        island_ll_y = ((Long) data.get("Island_LL_y")).intValue();
        System.out.println("Island_LL_y: " + island_ll_y);
        // Upper right hand corner of the Island
        island_ur_x = ((Long) data.get("Island_UR_x")).intValue();
        System.out.println("Island_UR_x: " + island_ur_x);
        island_ur_y = ((Long) data.get("Island_UR_y")).intValue();
        System.out.println("Island_UR_y: " + island_ur_y);
      } else {
        System.exit(0);
      }

    } catch (Exception e) {
      System.err.println("Error: " + e.getMessage());
    }

    /* Sensor related objects */

    // US Sensor (Obstacle Detection, Front)
    SensorModes usSensor = new EV3UltrasonicSensor(usPort); // Create usSensor instance
    SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
                                                              // the instance
    float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer where data is
                                                         // stored

    // Color Sensor (Line Detection, Left)
    SensorModes myColor1 = new EV3ColorSensor(portColor1); // Get sensor instance
    SampleProvider myColorStatus1 = myColor1.getMode("Red"); // Get sample provider as "RGB"
    float[] sampleColor1 = new float[myColorStatus1.sampleSize()]; // Create a data buffer

    // Color Sensor (Line Detection, Right)
    SensorModes myColor2 = new EV3ColorSensor(portColor2); // Get sensor instance
    SampleProvider myColorStatus2 = myColor2.getMode("Red"); // Get sample provider as "RGB"
    float[] sampleColor2 = new float[myColorStatus2.sampleSize()]; // Create a data buffer

    // Color Sensor (Color Classification, Front)
    SensorModes colorSensor = new EV3ColorSensor(colorPort); // Get sensor instance
    SampleProvider colorReading = colorSensor.getMode("RGB"); // Get sample provider as "RGB"
    float[] colorData = new float[colorReading.sampleSize()]; // Create a data buffer


    /* Obtaining Instances */

    // instance of Odometer
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);

    Display odometryDisplay = new Display(lcd); // instance of Display

    ColorClassification colorclassification =
        new ColorClassification(usDistance, usData, colorReading, colorData); // instance of
                                                                              // ColorClassification

    WeightCan weightcan = new WeightCan(weightMotor, colorclassification);

    // instance of LineCorrection
    LineCorrection linecorrection =
        new LineCorrection(myColorStatus1, sampleColor1, myColorStatus2, sampleColor2);

    Navigation navigation = new Navigation(odometer, leftMotor, rightMotor, sensorMotor,
        colorclassification, weightcan, linecorrection, WHEEL_RAD, WHEEL_RAD, TRACK, target_color); // instance
                                                                                                    // of
    // Navigation

    UltrasonicLocalizer uslocalizer = new UltrasonicLocalizer(odometer, leftMotor, rightMotor,
        WHEEL_RAD, WHEEL_RAD, TRACK, usDistance, usData, navigation); // instance of
                                                                      // UltrasonicLocalizer

    LightLocalizer lightlocalizer = new LightLocalizer(odometer, leftMotor, rightMotor, WHEEL_RAD,
        WHEEL_RAD, TRACK, navigation, linecorrection, corner); // instance of LightLocalizer

    Sound.beepSequenceUp(); // Shows its ready

    /* STARTING THREADS */

    // Starting odometer thread
    Thread odoThread = new Thread(odometer);
    odoThread.start();

    // Starting display thread
    Thread odoDisplayThread = new Thread(odometryDisplay);
    odoDisplayThread.start();

    // Start the thread for us localizer
    Thread usThread = new Thread(uslocalizer);
    usThread.start();
    usThread.join();

    // Start the thread for light localizer
    Thread lightThread = new Thread(lightlocalizer);
    lightThread.start();
    lightThread.join();

    Sound.beep();

    /* Generating the search map */
    int[] upperRight = {sz_ur_x, sz_ur_y};
    int[] lowerLeft = {sz_ll_x, sz_ll_y};
    int horizontal = upperRight[0] - lowerLeft[0] + 1; // The x nodes that will be traveled
    int vertical = upperRight[1] - lowerLeft[1] + 1; // The y nodes that will be traveled

    int[][] fullPath = new int[horizontal * vertical][3]; // Set up a 2D array of map
    int direction = 1; // Traveling to the right
    for (int i = 0; i < vertical; i++) {
      for (int j = 0; j < horizontal; j++) {
        if (direction == 1) { // Map generation
          fullPath[i * horizontal + j][0] = lowerLeft[0] + j;
          fullPath[i * horizontal + j][1] = lowerLeft[1] + i;
        } else {
          fullPath[i * horizontal + j][0] = upperRight[0] - j;
          fullPath[i * horizontal + j][1] = upperRight[1] + i;
        }
      }
      direction *= -1; // Traveling to the left
    }
    for (int i = 0; i < fullPath.length; i++) {
      if (i % (2 * horizontal) == horizontal - 1) {
        // right lower side can
        fullPath[i][2] = 0;
      } else if (i % (2 * horizontal) == horizontal) {
        // right upper side can
        fullPath[i][2] = 1;
      } else if (i % (2 * horizontal) == 2 * horizontal - 1) {
        // left lower side can
        fullPath[i][2] = 2;
      } else if (i % (2 * horizontal) == 0) {
        // left upper side can
        fullPath[i][2] = 3;
      } else { // straight line can
        fullPath[i][2] = 4;
      }
    }

    /* Traverse the search map and navigate */
    // Traveling to island and iterating the map TODO
    int i = 0;
    if (redTeam == TEAM_NUMBER) {
      navigation.travelTo(1 * TILE_SIZE, (tn_ll_y + tn_ur_y) / 2 * TILE_SIZE);
      navigation.travelTo((tn_ur_x + (1 / 2)) * TILE_SIZE, (tn_ll_y + tn_ur_y) / 2 * TILE_SIZE);
      navigation.travelTo((tn_ur_x + (1 / 2)) * TILE_SIZE, fullPath[i][0] * TILE_SIZE);
      navigation.travelTo(fullPath[i][0] * TILE_SIZE, fullPath[i][1] * TILE_SIZE);
      Sound.beep();
      Sound.twoBeeps();
      Sound.twoBeeps();
      navigation.roundSearch(fullPath[i][0] * TILE_SIZE, fullPath[i][1] * TILE_SIZE, -FULL_TURN / 4);
      i++;
      while (i < fullPath.length) {
        navigation.moveTo(fullPath[i][0] * TILE_SIZE, fullPath[i][1] * TILE_SIZE);
        if (fullPath[i][2] == 4) {
          if (odometer.getXYT()[2] < 180) {
            navigation.roundSearch(fullPath[i][0] * TILE_SIZE, fullPath[i][1] * TILE_SIZE, -FULL_TURN / 4);
          } else {
            navigation.roundSearch(fullPath[i][0] * TILE_SIZE, fullPath[i][1] * TILE_SIZE, FULL_TURN / 4);
          }
        } else if (fullPath[i][2] == 1) {
          navigation.roundSearch(fullPath[i][0] * TILE_SIZE, fullPath[i][1] * TILE_SIZE, -FULL_TURN / 4);
        } else if (fullPath[i][2] == 3) {
          navigation.roundSearch(fullPath[i][0] * TILE_SIZE, fullPath[i][1] * TILE_SIZE, FULL_TURN / 4);
        }
      }
    }

    /* Waiting for exit */
    // Wait here forever until button pressed to terminate the robot
    Button.waitForAnyPress();
    System.exit(0);
  }
}
