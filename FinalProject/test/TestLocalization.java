
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.finalproject.*;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
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
 * This is the test class for the UltrasonicLocalizer
 * 
 * @author Michel Fadi Majdalani
 *
 */
public class TestLocalization {

  /* STATIC FIELDS */

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
            WHEEL_RAD, TRACK, navigation, linecorrection, 0); // instance of LightLocalizer

    Sound.beepSequenceUp(); // Shows its ready

    /* STARTING THREADS */

    int option = 0;
	while (option == 0)  {// and wait for a button press. The button
		LCD.clear();
		LCD.drawString("Press any button", 0, 1);
		LCD.drawString(" to start test  ", 0, 2);
		option = Button.waitForAnyPress();
	}
	
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
        
    //Start the thread for light localizer
    Thread lightThread = new Thread(lightlocalizer);
    lightThread.start();
    lightThread.join();
    
    
    // Wait here forever until button pressed to terminate the robot
    LCD.clear();
    LCD.drawString("Test completed!", 0, 2);
    Button.waitForAnyPress();
    System.exit(0);
    
  }
}
