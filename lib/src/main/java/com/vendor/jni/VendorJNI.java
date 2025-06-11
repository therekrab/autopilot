package com.vendor.jni;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Demo class for loading the driver via JNI.
 */
public class VendorJNI {
  public static double getNumber() {
    return Rotation2d.kPi.getCos();
  }
}
