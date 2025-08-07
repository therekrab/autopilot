package com.therekrab.autopilot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * A helper class that converts a robot's ChassisSpeeds and pose to a Translation2d for Autopilot calculations.
 */
public class APVelocity {
  protected double vx;
  protected double vy;
  protected Rotation2d theta;

  /**
   *  Creates a new APVelocity with zero velocity and omega.
   * 
   * <p> This constructor creates an blank APVelocity. This can be dangerous, as it will cause Autopilot to 
   * assume the robot is not moving. If you are using APVelocity, you should always update it with the robot's current velocity.
   */
  public APVelocity() {
    vx = 0;
    vy = 0;
    theta = new Rotation2d();
  }

      /**
   * Creates a new APVelocity from a given ChassisSpeeds and robot pose.
   * 
   * <p> Autopilot utilizes Translation2d objects for velocity calculations, so this method allows users to skip creating a helper
   * method to convert their robot's ChassisSpeeds and pose to a Translation2d. If your drivetrain implementation doesn't have a method for accessing its ChassisSpeeds,
   * see the ChassisSpeeds documentation for more information on creating one.
   * 
   * @param robotRelativeSpeeds The current robot-relative ChassisSpeeds
   * @param robotPose The current robot Pose2d
   * 
   */

  public APVelocity(ChassisSpeeds robotRelativeSpeeds, Pose2d robotPose) {
    vx = robotRelativeSpeeds.vxMetersPerSecond;
    vy = robotRelativeSpeeds.vyMetersPerSecond;
    theta = robotPose.getRotation();  
  } 


        /**
   * Update this APVelocity with a new ChassisSpeeds and robot pose.
   * 
   * <p> As Autopilot is intended to be called periodically, if you are using APVelocity, 
   * use this to update the APVelocity with the robot's current ChassisSpeeds.
   * 
   * @param robotRelativeSpeeds The current robot-relative ChassisSpeeds
   * 
   * @return a Translation2d used for Autopilot calculations
   */

  public APVelocity update(ChassisSpeeds robotRelativeSpeeds, Pose2d robotPose) {
    this.vx = robotRelativeSpeeds.vxMetersPerSecond;
    this.vy = robotRelativeSpeeds.vyMetersPerSecond;
    this.theta = robotPose.getRotation();
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
      theta
      );
  }



}
