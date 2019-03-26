package ca.mcgill.ecse211.finalproject;

import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.WiFiClient.WifiConnection;
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
 * <p>
 * This is the main class of the program for the Final Project (beta demo version). It contains the
 * 4 motor/output ports (two large regulated motor, one medium regulated motor and one large
 * unregulated motor), and 4 sensor/input ports (three color sensor, one ultrasonic sensor). And in
 * the constant field, the user should change, before running the program, first, the SERVER_IP
 * according to the IP address of his/her own computer; second, the TEAM_NUMBER to the user team
 * number. The static constant field of this class also contains the parameters of the robot (wheel
 * radius and track width).
 * 
 * <p>
 * For this version, the robot will start at an unknown corner 0, 1, 2, or 3 (the lower left corner
 * of the board is the (0, 0) coordinate). The Wi-Fi connection that connects the robot with the
 * server using Wi-Fi, will obtain data from the server. The Wi-Fi connection is implemented by
 * importing and building path to a helper class. Using the if/else statement, the robot will only
 * store the useful data and exit the program if there is error in the server input or Wi-Fi
 * connection. After obtaining all the essential parameters, it will start to initialize all the
 * sensors (two color sensor to RED mode, one color sensor to RGB mode and the ultrasonic sensor to
 * distance mode), and create the instances for the program (odometer, display, weight can, line
 * correction, navigation, ultrasoinc localizer and light localizer). The x, y, theta coordinate of
 * the robot is: x - horizontal axis, y - vertical axis, theta - clockwise angle, (0, 0, 0) as the
 * lower left corner of the board and facing towards positive y. The Odometer class keep track of
 * the position (x, y, and theta) of the robot, with x, y and theta initialized according to the
 * result of the localization and the starting corner. The ColorClassification class will detect the
 * color of the can (fetching R, G, B sample using the RGB mode) and identify the color (comparing
 * the reading with the standardized default value of each color, i.e., blue, green, yellow and
 * red); the color sensor carried by the arm (rotates 180 degrees), so the color sensor will keep
 * detecting the color while the arm is moving, and finally return the color that is detected most.
 * The WeightCan class contains the control of the claw of the robot, including the lifting and
 * dropping can, and weight detection. The LineCorrection class contains two differential filter
 * method for the two color sensor in RED mode for line detection. The Navigation class contains the
 * control of the motion of the robot (turning, traveling and angle correction), it will also call
 * ColorClassification and WeightCan class and run them in threads. The UltrasonicLocalizer class is
 * able to localize the orientation (angle) of the robot and the LightLocalizer class will localize
 * the localization and the orientation (angle) of the robot, and reset the coordinates according to
 * the starting corner.
 * 
 * <p>
 * After initialing all the instances, the main method will start the thread for odometer, and
 * ultrasonic localizer at the same time. After the termination of the ultrasonic localizer, the
 * light localizer thread will be created and start. Finally, after the termination of the light
 * localization, the robot will beep to indicate the user that the robot is well localized and ready
 * to run.
 * 
 * <p>
 * After localizing to the grid, the main method will generate a path to travel through the tunnel,
 * containing the point of before and after tunnel localization point. After that, it will generate
 * a S-shape search map, then navigate the robot to travel through the tunnel, arrive at the lower
 * left corner of the search region, and perform a search in the prescribed area for a can of
 * specified color (search path is following the generated map). The robot will arrive at each map
 * point, turn to find the cans around it, go approach the can, detect the can and identify its
 * color, and beeps if the target color is found. Before termination, the robot will be navigated to
 * the upper right corner of the search region.
 * 
 * @author Floria Peng
 */
public class FinalProject {
  
  /* STATIC FIELDS */
  // Set these as appropriate for your team and current situation
  /**
   * The IP address of the server
   */

  private static final String SERVER_IP = "192.168.2.5";

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

    Navigation navigation = new Navigation(odometer, leftMotor, rightMotor, sensorMotor,
        colorclassification, weightcan, linecorrection, WHEEL_RAD, WHEEL_RAD, TRACK); // instance
                                                                                                    // of
    // Navigation

    UltrasonicLocalizer uslocalizer = new UltrasonicLocalizer(odometer, leftMotor, rightMotor,
        WHEEL_RAD, WHEEL_RAD, TRACK, usDistance, usData, navigation); // instance of
                                                                      // UltrasonicLocalizer

    LightLocalizer lightlocalizer = new LightLocalizer(odometer, leftMotor, rightMotor, WHEEL_RAD,
        WHEEL_RAD, TRACK, navigation, linecorrection, corner); // instance of LightLocalizer

    /* STARTING THREADS */

    // Starting odometer thread
    Thread odoThread = new Thread(odometer);
    odoThread.start();

    // Start the thread for us localizer
    Thread usThread = new Thread(uslocalizer);
    usThread.start();
    usThread.join();

    // Start the thread for light localizer
    Thread lightThread = new Thread(lightlocalizer);
    lightThread.start();
    lightThread.join();

    for (int i = 0; i < 3; i++) {
      Sound.beep();
      sleep(50);
    }
    
    /* Travel through tunnel */
    double[][] tunnel_points = getPoints(corner, tn_ll_x, tn_ll_y, tn_ur_x, tn_ur_y);
    navigation.travelTo(tunnel_points[0][0] * TILE_SIZE, tunnel_points[0][1] * TILE_SIZE);
    localizer(0, navigation, lightlocalizer, odometer);
    odometer.setXYT(tunnel_points[0][0] * TILE_SIZE, tunnel_points[0][1] * TILE_SIZE, 0);
    sleep(50);
    navigation.travelTo(tunnel_points[1][0] * TILE_SIZE, tunnel_points[1][1] * TILE_SIZE);
    navigation.travelTo(tunnel_points[2][0] * TILE_SIZE, tunnel_points[2][1] * TILE_SIZE);
    navigation.travelTo(tunnel_points[3][0] * TILE_SIZE, tunnel_points[3][1] * TILE_SIZE);
    localizer(0, navigation, lightlocalizer, odometer);
    odometer.setXYT(tunnel_points[0][0] * TILE_SIZE, tunnel_points[0][1] * TILE_SIZE, 0);
    sleep(50);
    
    /* Arrive at lower left corner of the search zone */
    navigation.travelTo(sz_ll_x * TILE_SIZE, sz_ll_y * TILE_SIZE); // First map point
    localizer(90, navigation, lightlocalizer, odometer);
    odometer.setXYT(sz_ll_x * TILE_SIZE, sz_ll_y * TILE_SIZE, 90);
    
    /* Waiting for exit */
    // Wait here forever until button pressed to terminate the robot
    Button.waitForAnyPress();
    System.exit(0);
  }

  /**
   * This method will generate the path that the robot will follow to travel through the tunnel. It
   * will use the starting corner and the position of the tunnel to generate a proper path,
   * considering different cases: the tunnel is next to the wall, or next to the river.
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
    double[][] tunnel_point = new double[2][2];
    double[] distance = new double[2];
    double[][] points = new double[4][2];
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
    if (orientation) {
      tunnel_point[0][0] = tn_ll_x;
      tunnel_point[0][1] = (tn_ur_y + tn_ll_y) * 0.5;
      tunnel_point[1][0] = tn_ur_x;
      tunnel_point[1][1] = (tn_ur_y + tn_ll_y) * 0.5;
    } else {
      tunnel_point[0][0] = (float) ((tn_ur_x + tn_ll_x) * 0.5);;
      tunnel_point[0][1] = tn_ll_y;
      tunnel_point[1][0] = (float) ((tn_ur_x + tn_ll_x) * 0.5);;
      tunnel_point[1][1] = tn_ur_y;
    }
    distance[0] = (float) Math
        .sqrt(Math.pow(lastx - tunnel_point[0][0], 2) + Math.pow(lasty - tunnel_point[0][1], 2));
    distance[1] = (float) Math
        .sqrt(Math.pow(lastx - tunnel_point[1][0], 2) + Math.pow(lasty - tunnel_point[1][1], 2));
    enter_angle = distance[0] < distance[1] ? 1 : 2; // 1 for entering at lower left, 2 for entering
                                                     // at upper right
    points[0][0] = points[0][1] = points[1][0] =
        points[1][1] = points[2][0] = points[2][1] = points[3][0] = points[3][1] = -1;
    if (enter_angle == 1) {
      points[0][0] = localization(lastx, tunnel_point[0][0]);
      points[0][1] = localization(lasty, tunnel_point[0][1]);
      if (orientation) {
        points[1][0] = points[0][0];
        points[1][1] = tunnel_point[0][1];
        points[2][0] = points[0][0] + 4;
        points[2][1] = tunnel_point[0][1];
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
  private static double localization (int starting, double tunnel) {
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
  private static void localizer (double angle, Navigation navigation, LightLocalizer lightlocalizer, Odometer odometer) {
    sleep(50);
    navigation.turnTo(angle);
    navigation.move(TILE_SIZE); // move forward (until you detect a line) to correct Y odometer reading
    lightlocalizer.correctAngle(); // when a line is detected, correct angle
    navigation.back(0, BACK_DIST); // Go back the offset distance between the wheels and sensors
    navigation.rotate(FULL_TURN / 4);
    navigation.move(TILE_SIZE); // move forward (until you detect a line) to correct Y odometer reading
    lightlocalizer.correctAngle(); // when a line is detected, correct angle
    navigation.back(0, BACK_DIST); // Go back the offset distance between the wheels and sensors
    navigation.rotate(-FULL_TURN / 4);
    odometer.position[2] = Math.toRadians(angle);
    sleep(50);
  }
  
  /**
   * This method implements the sleep of this thread.
   * 
   * @param time - The sleeping time
   */
  private static void sleep (int time) {
    try {
      Thread.sleep(time);
    } catch (Exception e) {
    }
  }
  
}
