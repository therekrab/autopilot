package com.therekrab.autopilot;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Demo class for loading the driver via JNI.
 */
public class Library {
  public static double getNumber() {
    return Rotation2d.kPi.getCos();
  }
}
