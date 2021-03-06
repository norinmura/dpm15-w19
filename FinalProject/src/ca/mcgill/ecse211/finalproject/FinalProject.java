package ca.mcgill.ecse211.finalproject;

import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import java.util.Map;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * <p>
 * This is the main class of the program for the Final Project. It contains the
 * 4 motor/output ports (two large regulated motor, one medium regulated motor and one large
 * unregulated motor), and 4 sensor/input ports (three color sensor, one ultrasonic sensor). In
 * the constant field, the user should change, before running the program, first, the SERVER_IP
 * according to the IP address of his/her own computer; second, the TEAM_NUMBER to the user team
 * number. The static constant field of this class also contains the parameters of the robot (wheel
 * radius and track width). The program will control the robot to complete the following tasks:
 * localize at corner, navigate through tunnel, arrive at search zone and search for cans, identify
 * the color and weight the cans, bring the can back to the starting corner and stop (see Software
 * Document section 2.0).
 * 
 * <p>
 * The visualized flow of the program is explained in Software Document section 10.0. For this
 * version, the robot will start at an unknown corner 0, 1, 2, or 3 (the lower left corner of the
 * board is the (0, 0) coordinate). At the beginning, the robot will start to initialize all the
 * sensors (two color sensor to RED mode, one color sensor to RGB mode and the ultrasonic sensor to
 * distance mode), and create the instances for the program (odometer, display, weight can, line
 * correction, navigation, ultrasonic localizer and light localizer). After initialization, The
 * Wi-Fi connection that connects the robot with the server using Wi-Fi, will obtain data from the
 * server. The Wi-Fi connection is implemented by importing and building path to a helper class.
 * Using the if/else statement, the robot will only store the useful data and exit the program if
 * there is error in the server input or Wi-Fi connection (see Software Document section 9.0). The
 * x, y, theta coordinate of the robot is: x - horizontal axis, y - vertical axis, theta - clockwise
 * angle, (0, 0, 0) as the lower left corner of the board and facing towards positive y.
 * 
 * <p>
 * The Odometer class keep track of the position (x, y, and theta) of the robot, with x, y and theta
 * initialized according to the result of the localization and the starting corner (see Software
 * Document section 3.0). 
 * <p>
 * The ColorClassification class will detect the color of the can (fetching
 * R, G, B sample using the RGB mode) and identify the color (comparing the reading with the
 * standardized default value of each color, i.e., blue, green, yellow and red); the color sensor
 * carried by the arm (rotates 180 degrees), so the color sensor will keep detecting the color while
 * the arm is moving, and finally returns the color that is detected most (see Software Document
 * section 6.0). 
 * <p>
 * The WeightCan class contains the control of the claw of the robot, including the
 * lifting and dropping can, and weight detection (see Software Document section 7.0). 
 * <p>
 * The LineCorrection class contains two differential filter method for the two color sensor in RED mode
 * for line detection. The Navigation class contains the control of the motion of the robot
 * (turning, traveling and angle correction), it will also call ColorClassification and WeightCan
 * class and run them in threads (see Software Document section 4.0). The UltrasonicLocalizer class
 * is able to localize the orientation (angle) of the robot and the LightLocalizer class will
 * localize the localization and the orientation (angle) of the robot, and reset the coordinates
 * according to the starting corner (see Software Document section 5.0).
 * <p>
 * After initialing all the instances, the main method will start the thread for odometer, and
 * ultrasonic localizer at the same time. odometer thread uses Odometer class, algorithm see Software
 * Document section 3.0. Ultrasonic localizer thread uses the ultrasonic sensor at the front of the
 * robot to detect the distance from the left wall and the distance form the back wall, and use the
 * two distances and odometer to calculate the angle between its current angle and absolute 0 angle
 * (UltrasonicLocalizer class, algorithm see Software Document section 5.1). After the termination
 * of the ultrasonic localizer, the light localizer thread will be created and started. Light
 * localizer thread uses the two light sensors at the back of the robot facing down to detect the
 * grid line. The first light sensor to detect the line will stop the wheel motor on its side and
 * wait for the second sensor to arrive at this line, stop both motors; therefore the wheels will be
 * parallel to the line (LightLocalizer class, algorithm see Software Document section 5.2).
 * Finally, after the termination of the light localization, the robot will beep to indicate the
 * user that the robot is well localized and ready to run.
 * 
 * <p>
 * The following program is run in the main method, using the methods in Navigation class. After
 * localizing to the grid, the main method will generate a path to travel through the tunnel,
 * containing the point of before and after tunnel localization point. After that, it will generate
 * a S-shape search map, then navigate the robot to travel through the tunnel, arrive at the first
 * map point, turn 360 degree to perform the search in the prescribed area for a can (search path is
 * following the generated map). The robot will arrive at each map point, turn to find the cans
 * around it, go approach the can, detect the can and identify its color, and beeps according to the
 * color and weight of the can. Before termination, the robot will go back through tunnel and place
 * the can at its starting corner. (The flowchart and thread map are in Software Document section
 * 10.0).
 * 
 * <p>
 * In summary, at any time, there will be at most 2 thread running at in parallel: odometer thread with 
 * either ultrasonicLocalizer, lightLocalizer, colorClassification or weightCan thread. Most threads are
 * run sequentially (wait for their completion), expect for the Odometer for which its value needs to constantly be updated.
 * 
 * <p>
 * The final state machine description of this program is the following: 
 * The program starts at the initial state where it initializes ports, and sensors.
 * It then transitions to state B where it connects to the Wifi Class and gets parameters.
 * It then transitions to state C where it performs ultrasonic localization. 
 * After that, it transitions to state D where it performs light localization.
 * Next, it transitions to state E which makes the robot navigate to a point where it will re-localize (state E1). 
 * It them transitions to state E2 where it gets ready to go through the tunnel. 
 * Next, it transitions to the composite state F (in particular F1) which is when it reaches the search zone. This state is composed
 * of sub-states which are a can is detected (F1) and a can is not detected (F2). 
 * After detecting a can it moves to state G where it goes back to tunnel.
 * Finally, it transition to state H which is the state where it goes back to starting corner.
 * 
 * 
 * @author Floria Peng
 */
public class FinalProject {

  /* STATIC FIELDS */
  // Set these as appropriate for your team and current situation
  /**
   * The IP address of the server
   */

  private static final String SERVER_IP = "192.168.2.15";

  /**
   * The team number of the user
   */
  private static final int TEAM_NUMBER = 15; // Team 15

  // Enable/disable printing of debug info from the WiFi class
  /**
   * Control the printing
   */
  private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;

  // Motors
  // Instantiate motors; left right and sensor motor to rotate the color sensor.
  /**
   * The large regulated left motor
   */
  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  /**
   * The large regulated right motor
   */
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
  /**
   * The medium regulated sensor motor that carries the color sensor arm
   */
  private static final EV3MediumRegulatedMotor sensorMotor =
      new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));
  /**
   * The unregulated weight motor that carries the claw
   */
  private static final UnregulatedMotor weightMotor =
      new UnregulatedMotor(LocalEV3.get().getPort("D"));

  /**
   * Ultrasonic sensor port
   */
  private static final Port usPort = LocalEV3.get().getPort("S3"); // Ultrasonic sensor port
  /**
   * The left light sensor port
   */
  private static final Port portColor1 = LocalEV3.get().getPort("S1"); // Light sensor port1
  /**
   * The right light sensor port
   */
  private static final Port portColor2 = LocalEV3.get().getPort("S2"); // Light sensor port2
  /**
   * The color sensor on the arm port
   */
  private static final Port colorPort = LocalEV3.get().getPort("S4"); // Light sensor port for color
                                                                      // detection
  /**
   * The radius of the wheels
   */
  public static final double WHEEL_RAD = 2.1; // The radius of the wheel
  /**
   * The track width of the robot
   */
  public static final double TRACK = 13.71; // The width of the robot measured
  /**
   * The angle for a full turn is 360 degrees
   */
  public static final int FULL_TURN = 360; // 360 degree for a circle
  /**
   * The tile size for the board that the robot is running on
   */
  public static final double TILE_SIZE = 30.48; // The tile size used for demo
  /**
   * The tunnel adjustment
   */
  public static final double TUNNEL_ADJ = 2; // More distance when traveling through the tunnel
  /**
   * Travel back distance (distance between wheels and sensors)
   */
  private static final double BACK_DIST = 9.0; // Travel back distance (distance between wheels and
                                               // sensors)
  /**
   * Time to display
   */
  private static final long TIME_OUT = 270000; // 270000

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

    System.out.println("Running..");

    /* WiFi connection */

    // Initialize WifiConnection class
    WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

    long timeStart = System.currentTimeMillis();

    // Initializing the parameters
    int redTeam, greenTeam, corner, ll_x, ll_y, ur_x, ur_y, tn_ll_x, tn_ll_y, tn_ur_x, tn_ur_y,
        sz_ll_x, sz_ll_y, sz_ur_x, sz_ur_y, island_ll_x, island_ll_y, island_ur_x, island_ur_y;
    redTeam = greenTeam =
        corner = ll_x = ll_y = ur_x = ur_y = tn_ll_x = tn_ll_y = tn_ur_x = tn_ur_y = sz_ll_x =
            sz_ll_y = sz_ur_x = sz_ur_y = island_ll_x = island_ll_y = island_ur_x = island_ur_y = 0;

    // Connect to server and get the data, catching any errors that might occur
    try {
      /**
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

      greenTeam = ((Long) data.get("GreenTeam")).intValue();

      if (redTeam == TEAM_NUMBER) {
        // Red team's starting corner
        corner = ((Long) data.get("RedCorner")).intValue();

        // Lower left hand corner of Red Zone
        ll_x = ((Long) data.get("Red_LL_x")).intValue();
        ll_y = ((Long) data.get("Red_LL_y")).intValue();
        // Upper right hand corner of Red Zone
        ur_x = ((Long) data.get("Red_UR_x")).intValue();
        ur_y = ((Long) data.get("Red_UR_y")).intValue();

        // Lower left hand corner of the red tunnel footprint
        tn_ll_x = ((Long) data.get("TNR_LL_x")).intValue();
        tn_ll_y = ((Long) data.get("TNR_LL_y")).intValue();
        // Upper right hand corner of the red tunnel footprint
        tn_ur_x = ((Long) data.get("TNR_UR_x")).intValue();
        tn_ur_y = ((Long) data.get("TNR_UR_y")).intValue();

        // Lower left hand corner of the red player search zone
        sz_ll_x = ((Long) data.get("SZR_LL_x")).intValue();
        sz_ll_y = ((Long) data.get("SZR_LL_y")).intValue();
        // Upper right hand corner of the red player search zone
        sz_ur_x = ((Long) data.get("SZR_UR_x")).intValue();
        sz_ur_y = ((Long) data.get("SZR_UR_y")).intValue();

      } else if (greenTeam == TEAM_NUMBER) {
        // Green team's starting corner
        corner = ((Long) data.get("GreenCorner")).intValue();

        // Lower left hand corner of Green Zone
        ll_x = ((Long) data.get("Green_LL_x")).intValue();
        ll_y = ((Long) data.get("Green_LL_y")).intValue();
        // Upper right hand corner of Green Zone
        ur_x = ((Long) data.get("Green_UR_x")).intValue();
        ur_y = ((Long) data.get("Green_UR_y")).intValue();

        // Lower left hand corner of the red tunnel footprint
        tn_ll_x = ((Long) data.get("TNG_LL_x")).intValue();
        tn_ll_y = ((Long) data.get("TNG_LL_y")).intValue();
        // Upper right hand corner of the red tunnel footprint
        tn_ur_x = ((Long) data.get("TNG_UR_x")).intValue();
        tn_ur_y = ((Long) data.get("TNG_UR_y")).intValue();

        // Lower left hand corner of the green player search zone
        sz_ll_x = ((Long) data.get("SZG_LL_x")).intValue();
        sz_ll_y = ((Long) data.get("SZG_LL_y")).intValue();
        // Upper right hand corner of the green player search zone
        sz_ur_x = ((Long) data.get("SZG_UR_x")).intValue();
        sz_ur_y = ((Long) data.get("SZG_UR_y")).intValue();

      }

      // Lower left hand corner of the Island
      island_ll_x = ((Long) data.get("Island_LL_x")).intValue();
      island_ll_y = ((Long) data.get("Island_LL_y")).intValue();
      // Upper right hand corner of the Island
      island_ur_x = ((Long) data.get("Island_UR_x")).intValue();
      island_ur_y = ((Long) data.get("Island_UR_y")).intValue();

    } catch (Exception e) {
      System.err.println("Error: " + e.getMessage());
    }

    /* Obtaining Instances */

    // instance of Odometer
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);

    ColorClassification colorclassification =
        new ColorClassification(usDistance, usData, colorReading, colorData); // instance of
                                                                              // ColorClassification

    WeightCan weightcan = new WeightCan(weightMotor, colorclassification);

    // instance of LineCorrection
    LineCorrection linecorrection =
        new LineCorrection(myColorStatus1, sampleColor1, myColorStatus2, sampleColor2);

    // instance of Navigation
    Navigation navigation = new Navigation(odometer, leftMotor, rightMotor, sensorMotor,
        colorclassification, weightcan, linecorrection, WHEEL_RAD, WHEEL_RAD, TRACK);

    // instance of UltrasonicLocalizer
    UltrasonicLocalizer uslocalizer = new UltrasonicLocalizer(odometer, leftMotor, rightMotor,
        WHEEL_RAD, WHEEL_RAD, TRACK, usDistance, usData, navigation);

    // instance of LightLocalizer
    LightLocalizer lightlocalizer = new LightLocalizer(odometer, leftMotor, rightMotor, WHEEL_RAD,
        WHEEL_RAD, TRACK, navigation, linecorrection, corner);

    /* STARTING THREADS */

    // Starting odometer thread
    Thread odoThread = new Thread(odometer);
    odoThread.start();

    // Start the thread for us localizer
    Thread usThread = new Thread(uslocalizer);
    usThread.start();
    usThread.join();
    sleep(100);

    // Start the thread for light localizer
    Thread lightThread = new Thread(lightlocalizer);
    lightThread.start();
    lightThread.join();
    sleep(100);

    for (int i = 0; i < 3; i++) {
      Sound.beep();
      sleep(50);
    }

    /* Travel through tunnel */
    double[][] tunnel_points = getPoints(corner, tn_ll_x, tn_ll_y, tn_ur_x, tn_ur_y);
    sleep(50);
    if (tunnel_points[4][0] != -1 || tunnel_points[4][1] != -1) {
      navigation.travelTo(tunnel_points[4][0] * TILE_SIZE, tunnel_points[4][1] * TILE_SIZE);
      localizer(0, navigation, lightlocalizer, odometer);
      odometer.setXYT(tunnel_points[4][0] * TILE_SIZE, tunnel_points[4][1] * TILE_SIZE, 0);
      sleep(50);
    }

    // Obtain starting corner's xy, determine if it's edge case
    int[] lastloc = getStartingXY(corner);
    boolean horizontal = isTunnelHorizontal(tn_ll_x, tn_ll_y, tn_ur_x, tn_ur_y);

    if (horizontal) {
      if (tn_ll_x == lastloc[0] || tn_ur_x == lastloc[0]) {
        tunnel_points =
            TweakPointsEdgeCaseX(tunnel_points, lastloc, tn_ll_x, tn_ll_y, tn_ur_x, tn_ur_y);
      }
    } else {
      if (tn_ll_y == lastloc[1] || tn_ur_y == lastloc[1]) {
        tunnel_points =
            TweakPointsEdgeCaseY(tunnel_points, lastloc, tn_ll_x, tn_ll_y, tn_ur_x, tn_ur_y);
      }
    }
    sleep(50);
    // Travel to the nearest point to the tunnel and re-localize, only if robot not already there
    navigation.travelTo(tunnel_points[0][0] * TILE_SIZE, tunnel_points[0][1] * TILE_SIZE);
    sleep(50);
    if (relocalizeBeforeTunnel(lastloc, tn_ll_x, tn_ll_y, tn_ur_x, tn_ur_y)) {
      localizer(0, navigation, lightlocalizer, odometer);
      odometer.setXYT(tunnel_points[0][0] * TILE_SIZE, tunnel_points[0][1] * TILE_SIZE, 0);
      sleep(50);
    }

    // Travel to the center of the tunnel, go through the tunnel, find another nearest point, then
    // localize.
    navigation.travelTo(tunnel_points[1][0] * TILE_SIZE, tunnel_points[1][1] * TILE_SIZE);
    sleep(50);
    navigation.travelTo(tunnel_points[2][0] * TILE_SIZE, tunnel_points[2][1] * TILE_SIZE);
    sleep(50);

    navigation.travelTo(tunnel_points[3][0] * TILE_SIZE, tunnel_points[3][1] * TILE_SIZE);
    localizer(0, navigation, lightlocalizer, odometer);
    odometer.setXYT(tunnel_points[3][0] * TILE_SIZE, tunnel_points[3][1] * TILE_SIZE, 0);
    sleep(50);

    /* Traverse the search map and navigate */
    // Traveling to island and iterating the map
    int[][] map = searchMap(sz_ll_x, sz_ll_y, sz_ur_x, sz_ur_y);
    int i = 0;
    while (i < map.length) {
      navigation.moveTo(map[i][0] * TILE_SIZE, map[i][1] * TILE_SIZE);
      sleep(50);
      if (navigation.get_can) {
        navigation.travelTo(map[i][0] * TILE_SIZE, map[i][1] * TILE_SIZE);
        sleep(50);
        if (i == 0) {
          for (int j = 0; j < 3; j++) { // beep 3 times upon arriving
            Sound.beep();
            sleep(50);
          }
        }
      } else {
        if (i == 0) {
          for (int j = 0; j < 3; j++) { // beep 3 times upon arriving
            Sound.beep();
            sleep(50);
          }
        }
        // Localize every 3 tiles
        if (i % 3 == 0) {
          localizer(0, navigation, lightlocalizer, odometer);
          odometer.setXYT(map[i][0] * TILE_SIZE, map[i][1] * TILE_SIZE, 0);
          sleep(50);
        }
        navigation.roundSearch(map[i][0] * TILE_SIZE, map[i][1] * TILE_SIZE, FULL_TURN, map[i][2]);
        sleep(50);
      }
      if (navigation.get_can) { // If the robot get a can, break
        localizer(0, navigation, lightlocalizer, odometer);
        odometer.setXYT(map[i][0] * TILE_SIZE, map[i][1] * TILE_SIZE, 0);
        sleep(50);
        break;
      }
      i++;
      sleep(50);
    }

    // Praying the god thread doesn't crash
    sleep(50);
    
    /* Going back */
    // Navigate to the nearest point to the tunnel and localize
    navigation.travelTo(tunnel_points[3][0] * TILE_SIZE, tunnel_points[3][1] * TILE_SIZE);
    localizer(0, navigation, lightlocalizer, odometer);
    odometer.setXYT(tunnel_points[3][0] * TILE_SIZE, tunnel_points[3][1] * TILE_SIZE, 0);
    sleep(50);

    // Travel to the entrance of the tunnel
    navigation.travelTo(tunnel_points[2][0] * TILE_SIZE, tunnel_points[2][1] * TILE_SIZE);
    sleep(50);

    // Travel to the other end of the tunnel
    navigation.travelTo(tunnel_points[1][0] * TILE_SIZE, tunnel_points[1][1] * TILE_SIZE);
    sleep(50);

    // Travel to the nearest point to the exit of the tunnel and localize, but only if robot is not
    // already at the starting pt.
    navigation.travelTo(tunnel_points[0][0] * TILE_SIZE, tunnel_points[0][1] * TILE_SIZE);
    sleep(50);
    if (relocalizeBeforeTunnel(lastloc, tn_ll_x, tn_ll_y, tn_ur_x, tn_ur_y)) {
      localizer(0, navigation, lightlocalizer, odometer);
      odometer.setXYT(tunnel_points[0][0] * TILE_SIZE, tunnel_points[0][1] * TILE_SIZE, 0);
      sleep(50);
    }

    if (tunnel_points[4][0] != -1 || tunnel_points[4][1] != -1) {
      navigation.travelTo(tunnel_points[4][0] * TILE_SIZE, tunnel_points[4][1] * TILE_SIZE);
      localizer(0, navigation, lightlocalizer, odometer);
      odometer.setXYT(tunnel_points[4][0] * TILE_SIZE, tunnel_points[4][1] * TILE_SIZE, 0);
      sleep(50);
    }

    switch (corner) {
      case 0:
        navigation.travelTo(0.5 * TILE_SIZE, 0.5 * TILE_SIZE);
        sleep(50);
        break;
      case 1:
        navigation.travelTo(14.5 * TILE_SIZE, 0.5 * TILE_SIZE);
        sleep(50);
        break;
      case 2:
        navigation.travelTo(14.5 * TILE_SIZE, 8.5 * TILE_SIZE);
        sleep(50);
        break;
      case 3:
        navigation.travelTo(0.5 * TILE_SIZE, 8.5 * TILE_SIZE);
        sleep(50);
        break;
    }
    for (int j = 0; j < 5; j++) { // beep 3 times upon arriving
      Sound.beep();
      sleep(50);
    }
    weightcan.claw_open();
    sensorMotor.setSpeed(FULL_TURN / 2); // set the scanning speed
    sensorMotor.rotate(FULL_TURN / 2, false); // The sensor motor will rotate less than 180 degree
                                              // (as we are using a gear)

    /* Waiting for exit */
    // Wait here forever until button pressed to terminate the robot
    Button.waitForAnyPress();
    System.exit(0);
  }

  /**
   * This method will generate the path that the robot will follow to travel through the tunnel. It
   * will use the starting corner and the position of the tunnel to generate a proper path,
   * considering different cases: the tunnel is next to the wall, or next to the river. And if the
   * tunnel is far, the 5th point is another localization point before the first point
   * 
   * @param corner - The starting corner
   * @param tn_ll_x - The x position of the lower left corner of the tunnel
   * @param tn_ll_y - The y position of the lower left corner of the tunnel
   * @param tn_ur_x - The x position of the upper right corner of the tunnel
   * @param tn_ur_y - The y position of the upper right corner of the tunnel
   * @return - The points map generated
   */
  private static double[][] getPoints(int corner, int tn_ll_x, int tn_ll_y, int tn_ur_x,
      int tn_ur_y) {
    boolean orientation = (tn_ur_x - tn_ll_x) > (tn_ur_y - tn_ll_y); // true if the tunnel is placed
                                                                     // horizontally
    int lastx, lasty, enter_angle;
    double[][] tunnel_point = new double[2][2]; // The two points at the entrance of the tunnel
    double[] distance = new double[2]; // The distance of the tunnel points from the starting grid
    double[][] points = new double[5][2]; // The return values
    switch (corner) {
      case 0:
        lastx = 1;
        lasty = 1;
        break;
      case 1:
        lastx = 14;
        lasty = 1;
        break;
      case 2:
        lastx = 14;
        lasty = 8;
        break;
      case 3:
        lastx = 1;
        lasty = 8;
        break;
      default:
        lastx = -1;
        lasty = -1;
        break;
    }
    if (orientation) { // Horizontal tunnel
      tunnel_point[0][0] = tn_ll_x;
      tunnel_point[0][1] = (tn_ur_y + tn_ll_y) * 0.5;
      tunnel_point[1][0] = tn_ur_x;
      tunnel_point[1][1] = (tn_ur_y + tn_ll_y) * 0.5;
    } else { // Vertical tunnel
      tunnel_point[0][0] = (tn_ur_x + tn_ll_x) * 0.5;
      tunnel_point[0][1] = tn_ll_y;
      tunnel_point[1][0] = (tn_ur_x + tn_ll_x) * 0.5;
      tunnel_point[1][1] = tn_ur_y;
    }
    distance[0] = Math
        .sqrt(Math.pow(lastx - tunnel_point[0][0], 2) + Math.pow(lasty - tunnel_point[0][1], 2));
    distance[1] = Math
        .sqrt(Math.pow(lastx - tunnel_point[1][0], 2) + Math.pow(lasty - tunnel_point[1][1], 2));
    enter_angle = distance[0] < distance[1] ? 1 : 2; // 1 for entering at lower left, 2 for entering
                                                     // at upper right
    points[0][0] = points[0][1] = points[1][0] = points[1][1] = points[2][0] =
        points[2][1] = points[3][0] = points[3][1] = points[4][0] = points[4][1] = -1;
    if (enter_angle == 1) {
      // Get the first localization point before tunnel
      points[0][0] = localization(lastx, tunnel_point[0][0]);
      points[0][1] = localization(lasty, tunnel_point[0][1]);
      if (orientation) {
        // The robot travel through tunnel from point[1] to point[2]
        points[1][0] = points[0][0];
        points[1][1] = tunnel_point[0][1];
        points[2][0] = points[0][0] + 4;
        points[2][1] = tunnel_point[0][1];
        // The after exit tunnel localization point
        points[3][0] = points[0][0] + 4;
        points[3][1] = points[0][1];
      } else {
        points[1][0] = tunnel_point[0][0];
        points[1][1] = points[0][1];
        points[2][0] = tunnel_point[0][0];
        points[2][1] = points[0][1] + 4;
        points[3][0] = points[0][0];
        points[3][1] = points[0][1] + 4;
      }
    } else if (enter_angle == 2) {
      points[0][0] = localization(lastx, tunnel_point[1][0]);
      points[0][1] = localization(lasty, tunnel_point[1][1]);
      if (orientation) {
        points[1][0] = points[0][0];
        points[1][1] = tunnel_point[1][1];
        points[2][0] = points[0][0] - 4;
        points[2][1] = tunnel_point[1][1];
        points[3][0] = points[0][0] - 4;
        points[3][1] = points[0][1];
      } else {
        points[1][0] = tunnel_point[1][0];
        points[1][1] = points[0][1];
        points[2][0] = tunnel_point[1][0];
        points[2][1] = points[0][1] - 4;
        points[3][0] = points[0][0];
        points[3][1] = points[0][1] - 4;
      }
    } else {
      System.out.println("Error");
    }
    // If the tunnel is far, generate another localization point somewhere between the starting grid
    // and the near tunnel localization point
    double between =
        Math.sqrt(Math.pow(lastx - points[0][0], 2) + Math.pow(lasty - points[0][1], 2));
    if (between > 5) {
      points[4][0] = points[0][0];
      points[4][1] = points[0][1];
      int addx = lastx > points[4][0] ? 1 : -1;
      int addy = lasty > points[4][1] ? 1 : -1;
      boolean[] flag = {false, false};
      while (between > 5) {
        if (points[4][0] > 1.5 && points[4][0] < 13.5) {
          points[4][0] += addx;
        } else {
          flag[0] = true;
        }
        if (points[4][1] > 1.5 && points[4][1] < 7.5) {
          points[4][1] += addy;
        } else {
          flag[1] = true;
        }
        if (flag[0] && flag[1]) {
          break;
        }
        between = Math.sqrt(Math.pow(lastx - points[4][0], 2) + Math.pow(lasty - points[4][1], 2));
      }
    }
    return points;
  }

  /**
   * This method will calculate the x or y position of before/after tunnel light localization. The
   * input of this method is the starting corner, and one value indicating the entrance of the
   * tunnel.
   * 
   * @param starting - The x or y position of the starting corner
   * @param tunnel - The x or y position of the middle point of the tunnel entrance
   * @return the x or y value of the before/after tunnel localization point
   */
  private static double localization(int starting, double tunnel) {
    do {
      if (starting < tunnel) {
        tunnel -= 0.5;
      } else if (starting > tunnel) {
        tunnel += 0.5;
      }
    } while (Math.abs(tunnel - Math.round(tunnel)) > 0.1);
    return tunnel;
  }

  /**
   * This method implements the light localization before and after tunnel.
   * 
   * @param navigation - The instance of the navigation class
   * @param lightlocalizer - The instance of the lightlocalizer class
   * @param odometer - The instance of the odometer class
   */
  private static void localizer(double angle, Navigation navigation, LightLocalizer lightlocalizer,
      Odometer odometer) {
    sleep(50);
    navigation.turnTo(angle);   

    // Praying the god thread doesn't crash
    sleep(50);
    
    navigation.move(TILE_SIZE); // move forward (until you detect a line) to correct Y odometer
                                // reading    
    lightlocalizer.correctAngle(); // when a line is detected, correct angle
    
    // Praying the god thread doesn't crash
    sleep(50);
    
    navigation.back(0, BACK_DIST); // Go back the offset distance between the wheels and sensors
    
    // Praying the god thread doesn't crash
    sleep(50);
    
    navigation.rotate(FULL_TURN / 4);
    
    // Praying the god thread doesn't crash
    sleep(50);
    
    navigation.move(TILE_SIZE); // move forward (until you detect a line) to correct Y odometer
                                // reading
    lightlocalizer.correctAngle(); // when a line is detected, correct angle
    
    // Praying the god thread doesn't crash
    sleep(50);
    
    navigation.back(0, BACK_DIST); // Go back the offset distance between the wheels and sensors
    
    // Praying the god thread doesn't crash
    sleep(50);
    
    navigation.rotate(-FULL_TURN / 4);
    
    // Praying the god thread doesn't crash
    sleep(50);
    
    odometer.position[2] = Math.toRadians(angle);
    sleep(50);
  }

  /**
   * This method implements the sleep of this thread.
   * 
   * @param time - The sleeping time
   */
  private static void sleep(int time) {
    try {
      Thread.sleep(time);
    } catch (Exception e) {
    }
  }

  /**
   * This method implements the generation of search map. The robot will going to adjust the
   * scanning region of the round search, and its path according to the size of the search zone. The
   * robot will arrive at each map point and do 360 turn to search the can around it.
   * 
   * @param sz_ll_x - The x coordinate of the lower left corner of the search zone
   * @param sz_ll_y - The y coordinate of the lower left corner of the search zone
   * @param sz_ur_x - The x coordinate of the upper right corner of the search zone
   * @param sz_ur_y - The y coordinate of the upper right corner of the search zone
   * @return the map generated
   */
  private static int[][] searchMap(int sz_ll_x, int sz_ll_y, int sz_ur_x, int sz_ur_y) {
    int horizontal = sz_ur_x - sz_ll_x; // The x nodes that will be traveled
    int vertical = sz_ur_y - sz_ll_y; // The y nodes that will be traveled
    int search_region = Math.min(horizontal / 2, vertical / 2);
    sz_ll_x += search_region;
    sz_ll_y += search_region;
    sz_ur_x -= search_region;
    sz_ur_y -= search_region;
    horizontal = sz_ur_x - sz_ll_x + 1;
    vertical = sz_ur_y - sz_ll_y + 1;
    int[][] fullPath = new int[horizontal * vertical][3]; // Set up a 2D array of map
    int direction = 1; // Traveling to the right
    for (int i = 0; i < vertical; i++) {
      for (int j = 0; j < horizontal; j++) {
        if (direction == 1) { // Map generation
          fullPath[i * horizontal + j][0] = sz_ll_x + j;
          fullPath[i * horizontal + j][1] = sz_ll_y + i;
        } else {
          fullPath[i * horizontal + j][0] = sz_ur_x - j;
          fullPath[i * horizontal + j][1] = sz_ll_y + i;
        }
      }
      direction *= -1; // Traveling to the left
    }
    for (int i = 0; i < fullPath.length; i++) {
      fullPath[i][2] = search_region;
    }
    return fullPath;
  }

  /**
   * This method will return the real (x,y) coordinate of the robot after localization. It depends
   * on the corner of which the robot performs the localization.
   * 
   * @param corner
   * @return int[] location of the starting grid
   */
  private static int[] getStartingXY(int corner) {
    int[] location = {-1, -1};

    switch (corner) {
      case 0:
        location[0] = 1;
        location[1] = 1;
        break;
      case 1:
        location[0] = 14;
        location[1] = 1;
        break;
      case 2:
        location[0] = 14;
        location[1] = 8;
        break;
      case 3:
        location[0] = 1;
        location[1] = 8;
        break;
      default:
        location[0] = -1;
        location[1] = -1;
        break;
    }
    return location;
  }

  /**
   * This method will take in the points generated in getPoints() and modify it to support the edge
   * cases. This method is called for the edge cases when tunnels are located at the same
   * x-coordinate as robot's starting point.
   * 
   * @param points[][] - The points generated for the tunnel before tweaking
   * @param tn_ll_x - x coordinate of lower left corner of tunnel
   * @param tn_ll_y - y coordinate of lower left corner of tunnel
   * @param tn_ur_x - x coordinate of upper right corner of tunnel
   * @param tn_ur_y - y coordinate of upper right corner of tunnel
   * 
   * @return modified points[][]
   */
  private static double[][] TweakPointsEdgeCaseX(double[][] points, int[] cornerXY, int tn_ll_x,
      int tn_ll_y, int tn_ur_x, int tn_ur_y) {
    double[][] pointsNew = points;
    if (tn_ll_x == 1) { // it's approaching from left
      if (cornerXY[1] < tn_ll_y) { // from below
        pointsNew[0][0] = 0.5;
        pointsNew[0][1] = tn_ll_y;
        pointsNew[1][0] = 0.5;
        pointsNew[2][0] = tn_ur_x + 1;
        pointsNew[3][0] = tn_ur_x + 1;
      } else if (cornerXY[1] > tn_ur_y) { // from above
        pointsNew[0][0] = 0.5;
        pointsNew[0][1] = tn_ur_y;
        pointsNew[1][0] = 0.5;
        pointsNew[2][0] = tn_ur_x + 1;
        pointsNew[3][0] = tn_ur_x + 1;
      }
    } else if (tn_ur_x == 14) { // it's approaching from right
      if (cornerXY[1] < tn_ll_y) { // from below
        pointsNew[0][0] = 14.5;
        pointsNew[0][1] = tn_ll_y;
        pointsNew[1][0] = 14.5;
        pointsNew[2][0] = tn_ll_x - 1;
        pointsNew[3][0] = tn_ll_x - 1;
      } else if (cornerXY[1] > tn_ur_y) { // from above
        pointsNew[0][0] = 14.5;
        pointsNew[0][1] = tn_ur_y;
        pointsNew[1][0] = 14.5;
        pointsNew[2][0] = tn_ll_x - 1;
        pointsNew[3][0] = tn_ll_x - 1;
      }
    }
    return pointsNew;
  }

  /**
   * This method will take in the points generated in getPoints() and modify it to support the edge
   * cases. This method is called for the edge cases when tunnels are located at the same
   * y-coordinate as robot's starting point.
   * 
   * @param points[][] - The points generated for the tunnel before tweaking
   * @param tn_ll_x - x coordinate of lower left corner of tunnel
   * @param tn_ll_y - y coordinate of lower left corner of tunnel
   * @param tn_ur_x - x coordinate of upper right corner of tunnel
   * @param tn_ur_y - y coordinate of upper right corner of tunnel
   * 
   * @return modified points[][]
   */
  private static double[][] TweakPointsEdgeCaseY(double[][] points, int[] cornerXY, int tn_ll_x,
      int tn_ll_y, int tn_ur_x, int tn_ur_y) {
    double[][] pointsNew = points;
    if (tn_ll_y == 1) { // it's approaching from below
      if (cornerXY[0] < tn_ll_x) { // from left
        pointsNew[0][0] = tn_ll_x;
        pointsNew[0][1] = 0.5;
        pointsNew[1][1] = 0.5;
        pointsNew[2][1] = tn_ur_y + 1;
        pointsNew[3][1] = tn_ur_y + 1;
      } else if (cornerXY[0] > tn_ur_x) { // from right
        pointsNew[0][0] = tn_ur_x;
        pointsNew[0][1] = 0.5;
        pointsNew[1][1] = 0.5;
        pointsNew[2][1] = tn_ur_y + 1;
        pointsNew[3][1] = tn_ur_y + 1;
      }
    } else if (tn_ur_y == 8) { // it's approaching from above
      if (cornerXY[0] < tn_ll_x) { // from left
        pointsNew[0][0] = tn_ll_x;
        pointsNew[0][1] = 8.5;
        pointsNew[1][1] = 8.5;
        pointsNew[2][1] = tn_ll_y - 1;
        pointsNew[3][1] = tn_ll_y - 1;
      } else if (cornerXY[0] > tn_ur_x) { // from right
        pointsNew[0][0] = tn_ur_x;
        pointsNew[0][1] = 8.5;
        pointsNew[1][1] = 8.5;
        pointsNew[2][1] = tn_ll_y - 1;
        pointsNew[3][1] = tn_ll_y - 1;
      }
    }
    return pointsNew;
  }

  /**
   * This method helps to determine if the robot should re-localize before getting into the tunnel.
   * First identifies the current robot's position (x,y) based on the given corner. Then, whether it
   * is an edge case.
   * 
   * @param corner - the starting corner of the robot
   * @param tn_ll_x - x coordinate of lower left corner of tunnel
   * @param tn_ll_y - y coordinate of lower left corner of tunnel
   * @param tn_ur_x - x coordinate of upper right corner of tunnel
   * @param tn_ur_y - y coordinate of upper right corner of tunnel
   * @return boolean if it is an edge case (not localize before tunnel)
   */
  private static boolean relocalizeBeforeTunnel(int[] cornerXY, int tn_ll_x, int tn_ll_y,
      int tn_ur_x, int tn_ur_y) {
    if (tn_ll_x == cornerXY[0] || tn_ur_x == cornerXY[0]) {
      if (isTunnelHorizontal(tn_ll_x, tn_ll_y, tn_ur_x, tn_ur_y)) {
        return false;
      }
    } else if (tn_ll_y == cornerXY[1] || tn_ur_y == cornerXY[1]) {
      if (!isTunnelHorizontal(tn_ll_x, tn_ll_y, tn_ur_x, tn_ur_y)) {
        return false;
      }
    }
    return true;
  }

  /**
   * This method returns true if the tunnel is placed horizontal
   * 
   * @param tn_ll_x - x coordinate of lower left corner of tunnel
   * @param tn_ll_y - y coordinate of lower left corner of tunnel
   * @param tn_ur_x - x coordinate of upper right corner of tunnel
   * @param tn_ur_y - y coordinate of upper right corner of tunnel
   * @return if the tunnel is horizontally placed
   */
  private static boolean isTunnelHorizontal(int tn_ll_x, int tn_ll_y, int tn_ur_x, int tn_ur_y) {
    boolean orientation = (tn_ur_x - tn_ll_x) > (tn_ur_y - tn_ll_y);
    return orientation;
  }

}
