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
 * This is the main class of the program for the Final Project. Starting in a known
 * corner, localize to the grid, and perform a search in the prescribed area for a can of specified
 * color. It will then detect the can and identify its color. The prescribed area is given to us by
 * a lower left corner and an upper right corner.
 * 
 * @author Floria Peng
 *
 */
public class FinalProject {

  /* STATIC FIELDS */

  // ** Set these as appropriate for your team and current situation **
  private static final String SERVER_IP = "192.168.2.16";
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

  // Ports
  private static final Port usPort = LocalEV3.get().getPort("S3"); // Ultrasonic sensor port
  private static final Port portColor1 = LocalEV3.get().getPort("S1"); // Light sensor port1
  private static final Port portColor2 = LocalEV3.get().getPort("S2"); // Light sensor port2
  private static final Port colorPort = LocalEV3.get().getPort("S4"); // Light sensor port for color
                                                                      // detection

  private static final TextLCD lcd = LocalEV3.get().getTextLCD(); // The LCD display

  /* CONSTANTS */
  public static final double WHEEL_RAD = 2.1; // The radius of the wheel
  public static final double TRACK = 11.85; // The width of the robot measured
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

      } else if (greenTeam == TEAM_NUMBER) {
        // Green team's starting corner
        corner = ((Long) data.get("GreenCorner")).intValue();
        System.out.println("GreenCorner: " + corner);

        // Lower left hand corner of Green Zone
        ll_x = ((Long) data.get("Green_LL_x")).intValue();
        System.out.println("Green_LL_x: " + ll_x);
        ll_y = ((Long) data.get("Green_LL_y")).intValue();
        System.out.println("Green_LL_y: " + ll_y);
        // Upper right hand corner of Green Zone
        ur_x = ((Long) data.get("Green_UR_x")).intValue();
        System.out.println("Green_UR_x: " + ur_x);
        ur_y = ((Long) data.get("Green_UR_y")).intValue();
        System.out.println("Green_UR_y: " + ur_y);

        // Lower left hand corner of the red tunnel footprint
        tn_ll_x = ((Long) data.get("TNG_LL_x")).intValue();
        System.out.println("TNG_LL_x: " + tn_ll_x);
        tn_ll_y = ((Long) data.get("TNG_LL_y")).intValue();
        System.out.println("TNG_LL_y: " + tn_ll_y);
        // Upper right hand corner of the red tunnel footprint
        tn_ur_x = ((Long) data.get("TNG_UR_x")).intValue();
        System.out.println("TNG_UR_x: " + tn_ur_x);
        tn_ur_y = ((Long) data.get("TNG_UR_y")).intValue();
        System.out.println("TNG_UR_y: " + tn_ur_y);

        // Lower left hand corner of the green player search zone
        sz_ll_x = ((Long) data.get("SZG_LL_x")).intValue();
        System.out.println("SZG_LL_x: " + sz_ll_x);
        sz_ll_y = ((Long) data.get("SZG_LL_y")).intValue();
        System.out.println("SZG_LL_y: " + sz_ll_y);
        // Upper right hand corner of the green player search zone
        sz_ur_x = ((Long) data.get("SZG_UR_x")).intValue();
        System.out.println("SZG_UR_x: " + sz_ur_x);
        sz_ur_y = ((Long) data.get("SZG_UR_y")).intValue();
        System.out.println("SZG_UR_y: " + sz_ur_y);

      }

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
        colorclassification, weightcan, linecorrection, WHEEL_RAD, WHEEL_RAD, TRACK); // instance of
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

    // Starting localization thread
    UltrasonicLocalizer.OPTION = false; // The user is choosing rising edge

    // Start the thread for us localizer
    Thread usThread = new Thread(uslocalizer);
    usThread.start();
    usThread.join();

    // Start the thread for light localizer
    Thread lightThread = new Thread(lightlocalizer);
    lightThread.start();
    lightThread.join();

    // Path Generation
    int width = sz_ur_x - sz_ll_x;
    int height = sz_ur_y - sz_ll_y;
    double middleR = (double) (tn_ll_y + tn_ur_y) / 2;
    double middleG = (double) (tn_ll_x + tn_ur_x) / 2;
    int pathLength = width * height;
    double[][] path = new double[pathLength][3];
    double[][] fullPath = new double[pathLength + height - 1][3];

    if (redTeam == TEAM_NUMBER) {
      int pointNumber = 0;
      for (int i = 0; pointNumber < pathLength;) { // Recurse until the all the points are assigned
        if ((i / width) % 2 == 0) { // even, right side of the search
          if ((middleR - (pointNumber / width) / 2) < sz_ll_y) {
            i += width; // If the map is filled up in one side of the tunnel, make it fill the other
                        // side
          } else {
            path[pointNumber][0] = sz_ll_x + pointNumber % width;
            path[pointNumber][1] = middleR - (i / width) / 2;
            path[pointNumber][2] = 1;
            pointNumber++;
            i++;
          }
        } else { // odd, left side of the search region
          if ((middleR + ((pointNumber / width) + 1) / 2) > sz_ur_y) {
            i += width; // If the map is filled up in one side of the tunnel, make it fill the other
                        // side
          } else {
            path[pointNumber][0] = sz_ll_x + pointNumber % width;
            path[pointNumber][1] = middleR + ((i / width) / 2 + 1);
            path[pointNumber][2] = 1;
            pointNumber++;
            i++;
          }
        }
      }
      int j = 1;
      double[] initialPoint = path[0];
      fullPath[0] = initialPoint;
      for (int i = 1; i < pointNumber; i++) {
        if (path[i][0] < path[i - 1][0]) { // switch line
          fullPath[j] = Arrays.copyOf(initialPoint, initialPoint.length);
          fullPath[j++][2] = -1; // The point is for correction purpose
          initialPoint = path[i];
        }
        fullPath[j++] = path[i];
      }
    } else if (greenTeam == TEAM_NUMBER) {
      int pointNumber = 0;
      for (int i = 0; pointNumber < pathLength;) { // Recurse until the all the points are assigned
        if ((i / height) % 2 == 0) { // even, right side of the search
          if ((middleG - (pointNumber / height) / 2) < sz_ll_x) {
            i += height; // If the map is filled up in one side of the tunnel, make it fill the
                         // other side
          } else {
            path[pointNumber][1] = sz_ll_y + pointNumber % height;
            path[pointNumber][0] = middleG - (i / height) / 2;
            path[pointNumber][2] = 1;
            pointNumber++;
            i++;
          }
        } else { // odd, left side of the search region
          if ((middleG + ((pointNumber / height) + 1) / 2) > sz_ur_x) {
            i += height; // If the map is filled up in one side of the tunnel, make it fill the
                         // other side
          } else {
            path[pointNumber][1] = sz_ll_y + pointNumber % height;
            path[pointNumber][0] = middleG + ((i / height) / 2 + 1);
            path[pointNumber][2] = 1;
            pointNumber++;
            i++;
          }
        }
      }
      int j = 1;
      double[] initialPoint = path[0];
      fullPath[0] = initialPoint;
      for (int i = 1; i < pointNumber; i++) {
        if (path[i][1] < path[i - 1][1]) { // switch line
          fullPath[j] = Arrays.copyOf(initialPoint, initialPoint.length);
          fullPath[j++][2] = -1; // The point is for correction purpose
          initialPoint = path[i];
        }
        fullPath[j++] = path[i];
      }
    }

    // Traveling to island and iterating the map TODO
    int i = 0;
    if (redTeam == TEAM_NUMBER) {
      navigation.travelTo(1 * TILE_SIZE, (tn_ll_y + tn_ur_y) / 2 * TILE_SIZE);
      navigation.travelTo(tn_ur_x * TILE_SIZE, (tn_ll_y + tn_ur_y) / 2 * TILE_SIZE);
      while (i < fullPath.length) {
        navigation.moveTo(fullPath[i][0] * TILE_SIZE, fullPath[i][1] * TILE_SIZE);
        if (navigation.get_can) {
          navigation.travelTo(path[0][0] * TILE_SIZE, odometer.getXYT()[1]);
          navigation.travelTo(path[0][0] * TILE_SIZE, path[0][1] * TILE_SIZE);
          navigation.travelTo(1 * TILE_SIZE, (tn_ll_y + tn_ur_y) / 2 * TILE_SIZE);
          navigation.dropCan();
          navigation.travelTo(path[0][0] * TILE_SIZE, path[0][1] * TILE_SIZE);
        } else if (path[i][2] > 0) {
          navigation.roundSearch();
          if (navigation.get_can) {
            navigation.travelTo(path[0][0] * TILE_SIZE, odometer.getXYT()[1]);
            navigation.travelTo(path[0][0] * TILE_SIZE, path[0][1] * TILE_SIZE);
            navigation.travelTo(1 * TILE_SIZE, (tn_ll_y + tn_ur_y) / 2 * TILE_SIZE);
            navigation.dropCan();
            navigation.travelTo(path[0][0] * TILE_SIZE, path[0][1] * TILE_SIZE);
          }
        }
      }
    } else if (greenTeam == TEAM_NUMBER) {
      navigation.travelTo((tn_ll_x + tn_ur_x) / 2 * TILE_SIZE, 1 * TILE_SIZE);
      navigation.travelTo((tn_ll_x + tn_ur_x) / 2 * TILE_SIZE, tn_ur_y * TILE_SIZE);
      while (i < fullPath.length) {
        navigation.moveTo(fullPath[i][0] * TILE_SIZE, fullPath[i][1] * TILE_SIZE);
        if (navigation.get_can) {
          navigation.travelTo(odometer.getXYT()[0], path[0][1] * TILE_SIZE);
          navigation.travelTo(path[0][0] * TILE_SIZE, path[0][1] * TILE_SIZE);
          navigation.travelTo((tn_ll_x + tn_ur_x) / 2 * TILE_SIZE, 1 * TILE_SIZE);
          navigation.dropCan();
          navigation.travelTo(path[0][0] * TILE_SIZE, path[0][1] * TILE_SIZE);
        } else if (path[i][2] > 0) {
          navigation.roundSearch();
          if (navigation.get_can) {
            navigation.travelTo(odometer.getXYT()[0], path[0][1] * TILE_SIZE);
            navigation.travelTo(path[0][0] * TILE_SIZE, path[0][1] * TILE_SIZE);
            navigation.travelTo((tn_ll_x + tn_ur_x) / 2 * TILE_SIZE, 1 * TILE_SIZE);
            navigation.dropCan();
            navigation.travelTo(path[0][0] * TILE_SIZE, path[0][1] * TILE_SIZE);
          }
        }
      }
    }

    // Wait here forever until button pressed to terminate the robot
    Button.waitForAnyPress();
    System.exit(0);
  }
}
