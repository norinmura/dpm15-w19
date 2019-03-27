package ca.mcgill.ecse211.test;

import lejos.robotics.SampleProvider;

/**
 * This class works as a poller to obtain the value from the sensor, periodically. 
 * The while loop does the sample fetching periodically. 
 * Assuming that the us.fetchSample operate in about 20mS, and that the thread sleeps for 50 mS at the end of each loop, then
 * one cycle through the loop is approximately 70 mS. This corresponds to a sampling rate of 1/70mS i.e. about 14 Hz.
 */
public class SensorPoller extends Thread {
  private SampleProvider sp;
  private float[] data;

  public SensorPoller(SampleProvider sp, float[] data) {
    this.sp = sp;
    this.data = data;
  }


  public void run() {
    while (true) {
      sp.fetchSample(data, 0); // acquire data
      try {
        Thread.sleep(50);
      } catch (Exception e) {
      } // Poor man's timed sampling
    }
  }

}
