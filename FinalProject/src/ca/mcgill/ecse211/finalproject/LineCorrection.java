package ca.mcgill.ecse211.finalproject;

import lejos.robotics.SampleProvider;

/**
 * This class implements differential filter for the color sensor detecting lines.
 * 
 * @author Floria Peng
 */
public class LineCorrection {

  /* PRIVATE FIELDS */
  /**
   * The sample provider for the color sensor
   */
  private SampleProvider myColorStatus1;
  /**
   * The data buffer for the color sensor reading
   */
  private float[] sampleColor1;
  /**
   * The sample provider for the color sensor
   */
  private SampleProvider myColorStatus2;
  /**
   * The data buffer for the color sensor reading
   */
  private float[] sampleColor2;

  /* CONSTANTS */
  /**
   * Initialize the last variable to a specific number
   */
  double[] last = {Math.PI, Math.PI};
  /**
   * Last and current are both used for differential filter
   */
  double[] current = {0, 0};

  /**
   * This is a constructor for LineCorrection.
   * 
   * @param myColorStatus1 - sample provider for the left color sensor
   * @param sampleColor1 - data buffer for left color sensor
   * @param myColorStatus2 - sample provider for the right color sensor
   * @param sampleColor2 - data buffer for right color sensor
   */
  public LineCorrection(SampleProvider myColorStatus1, float[] sampleColor1,
      SampleProvider myColorStatus2, float[] sampleColor2) {

    this.myColorStatus1 = myColorStatus1;
    this.sampleColor1 = sampleColor1;
    this.myColorStatus2 = myColorStatus2;
    this.sampleColor2 = sampleColor2;

  }

  /**
   * The differential filter of the light sensor, it will consider detecting a line if there is a
   * huge increase of the reading (the derivative if large)
   * 
   * @return - true for detecting an line, vice versa
   */
  boolean filter1() { // Differential filter

    /* Obtain and store the color reading */
    myColorStatus1.fetchSample(sampleColor1, 0); // Used for obtaining color reading from the
                                                 // SampleProvider

    if (Math.abs(last[0] - Math.PI) < Math.pow(0.1, 5)) { // If last has not been assigned for any
                                                          // number yet
      last[0] = current[0] = sampleColor1[0];
    } else {
      last[0] = current[0]; // Update the last
      current[0] = sampleColor1[0]; // Update the current
    }
    System.out.println("");

    /* Detect the differential */
    if ((current[0] - last[0]) * 1000 < -38) { // If there is a black line detected
      System.out.println("");
      return true;
    }
    return false;
  }

  /**
   * The differential filter of the light sensor, it will consider detecting a line if there is a
   * huge increase of the reading (the derivative if large)
   * 
   * @return - true for detecting an line, vice versa
   */
  boolean filter2() { // Differential filter

    /* Obtain and store the color reading */
    myColorStatus2.fetchSample(sampleColor2, 0); // Used for obtaining color reading from the
                                                 // SampleProvider

    if (Math.abs(last[1] - Math.PI) < Math.pow(0.1, 5)) { // If last has not been assigned for any
                                                          // number yet
      last[1] = current[1] = sampleColor2[0];
    } else {
      last[1] = current[1]; // Update the last
      current[1] = sampleColor2[0]; // Update the current
    }
    System.out.println("");

    /* Detect the differential */
    if ((current[1] - last[1]) * 1000 < -38) { // If there is a black line detected
      System.out.println("");
      return true;
    }
    return false;
  }

}
