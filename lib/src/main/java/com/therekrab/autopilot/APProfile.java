package com.therekrab.autopilot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/**
 * A class representing a profile that determines how AP approaches a target.
 *
 * The constraints property of the profile limits the robot's behavior.
 *
 * Acceptable error for the controller (both translational and rotational) are stored here.
 *
 * The "beeline radius" determines the distance at which the robot drives directly at the target and
 * no longer respects entry angle. This is helpful because if the robot overshoots by a small
 * amount, that error should not cause the robot do completely circle back around.
 */
public class APProfile {
  protected APConstraints constraints;
  protected Distance errorXY;
  protected Angle errorTheta;
  protected Distance beelineRadius;

  public APProfile() {
    errorXY = Meters.of(0);
    errorTheta = Rotations.of(0);
    beelineRadius = Meters.of(0);
  }

  /**
   * Modifies this profile's tolerated error in the XY plane and returns itself
   */
  public APProfile withErrorXY(Distance errorXY) {
    this.errorXY = errorXY;
    return this;
  }

  /**
   * Modifies this profile's tolerated angular error and returns itself
   */
  public APProfile withErrorTheta(Angle errorTheta) {
    this.errorTheta = errorTheta;
    return this;
  }

  /**
   * Modifies this profile's path generation constraints and returns itself
   */
  public APProfile withConstraints(APConstraints constraints) {
    this.constraints = constraints;
    return this;
  }

  /**
   * Modifies this profile's beeline radius and returns itself
   *
   * The beeline radius is a distance where, under that range, entry angle is no longer respected.
   * This prevents small overshoots from causing the robot to make a full arc and instaed correct
   * itself.
   */
  public APProfile withBeelineRadius(Distance beelineRadius) {
    this.beelineRadius = beelineRadius;
    return this;
  }

  /**
   * Returns the tolerated translation error for this profile
   */
  public Distance getErrorXY() {
    return errorXY;
  }

  /**
   * Returns the tolerated angular error for this profile
   */
  public Angle getErrorTheta() {
    return errorTheta;
  }

  /**
   * Returns the path generation constraints for this profile
   */
  public APConstraints getConstraints() {
    return constraints;
  }

  /**
   * Returns the beeline radius for this profile
   */
  public Distance getBeelineRadius() {
    return beelineRadius;
  }
}
