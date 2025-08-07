package com.therekrab.autopilot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * A helper class that converts a robot's ChassisSpeeds to a Translation2d for Autopilot calculations.
 */
public class APVelocity {
  protected double vx;
  protected double vy;
  protected double omega;

  /**
   *  Creates a new APVelocity with zero velocity and omega.
   * 
   * <p> This constructor creates an APVelocity that will result in no movement when passed into Autopilot.calculate().
   */
  public APVelocity() {
    vx = 0;
    vy = 0;
    omega = 0;
  }

      /**
   * Creates a new APVelocity from a given ChassisSpeeds.
   * 
   * <p> Autopilot utilizes Translation2d objects for velocity calculations, so this method allows users to skip creating a helper
   * method to convert their robot's ChassisSpeeds to a Translation2d. If your drivetrain implementation doesn't have a method for accessing its ChassisSpeeds,
   * see the ChassisSpeeds documentation for more information on creating one.
   * 
   * @param robotRelativeSpeeds The current robot-relative ChassisSpeeds
   * 
   */

  public APVelocity(ChassisSpeeds robotRelativeSpeeds) {
    vx = robotRelativeSpeeds.vxMetersPerSecond;
    vy = robotRelativeSpeeds.vyMetersPerSecond;
    omega = robotRelativeSpeeds.omegaRadiansPerSecond;  
  } 


        /**
   * Update this APVelocity with a new ChassisSpeeds.
   * 
   * <p> As Autopilot is intended to be called periodically, if you are using APVelocity, 
   * use this to update the APVelocity with the robot's current ChassisSpeeds.
   * 
   * @return a Translation2d used for Autopilot calculations
   */

  public APVelocity updateSpeeds(ChassisSpeeds robotRelativeSpeeds) {
    this.vx = robotRelativeSpeeds.vxMetersPerSecond;
    this.vy = robotRelativeSpeeds.vyMetersPerSecond;
    this.omega = robotRelativeSpeeds.omegaRadiansPerSecond;
    return this;  
  }


        /**
   * Get the Translation2d representation of this APVelocity.
   * 
   * <p> If you are using APVelocity, pass this getter into Autopilot.calculate().
   * 
   * @return a Translation2d used for Autopilot calculations
   */

  public Translation2d get() {
    return new Translation2d(vx, vy).
    rotateBy(
      new Rotation2d(omega)
      );
  }



}
