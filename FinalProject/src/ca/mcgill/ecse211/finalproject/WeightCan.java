package ca.mcgill.ecse211.finalproject;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.Port;

public class WeightCan implements Runnable {
  
  private static final EV3LargeRegulatedMotor liftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
  private static final UnregulatedMotor weightMotor = new UnregulatedMotor(LocalEV3.get().getPort("C"));
  private static final Port touchPort = LocalEV3.get().getPort("S4");
    
  public void run() {
    return;
  }
  
  boolean weight() {
    return false;
  }

}
