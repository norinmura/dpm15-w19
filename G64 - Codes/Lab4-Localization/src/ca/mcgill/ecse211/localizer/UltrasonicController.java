package ca.mcgill.ecse211.localizer;

public interface UltrasonicController {

  public void processUSData(int distance);

  public int readUSDistance();
}
