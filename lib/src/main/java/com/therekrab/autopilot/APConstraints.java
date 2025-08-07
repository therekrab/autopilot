package com.therekrab.autopilot;

/**
 * A class that holds constraint information for an Autopilot action.
 * 
 * Constraints are max velocity, acceleration, and jerk.
 */
public class APConstraints {
  protected double velocity;
  protected double acceleration;
  protected double jerk;

  /** Creates a blank APConstraints object.
   * <p> A blank APConstraints will not limit velocity, acceleration, or jerk.
  */
  public APConstraints() {
    this.velocity = Double.POSITIVE_INFINITY; // Default to no limit on velocity
  }

  /**
   * Creates a new APConstraints object with a given max velocity, acceleration, and jerk.
   * @param velocity The maximum velocity that Autopilot will demand, in m/s
   * @param acceleration The maximum acceleration that Autopilot action will use to correct initial velocities, in m/s^2
   * @param jerk The maximum jerk that Autopilot will use to decelerate at the end of an action, in m/s^3
   */

  public APConstraints(double velocity, double acceleration, double jerk) {
    this.velocity = velocity;
    this.acceleration = acceleration;
    this.jerk = jerk;
  }

  /**
   * Create a new APConstraints object with a given max acceleration and jerk.
   * <p> This constructor defaults the velocity to unlimited.
   * @param acceleration The maximum acceleration that  Autopilot action will use to correct initial velocities, in m/s^2
   * @param jerk The maximum jerk that Autopilot will use to decelerate at the end of an action, in m/s^3
   * 
   */
  public APConstraints(double acceleration, double jerk) {
    this.acceleration = acceleration;
    this.jerk = jerk;
    this.velocity = Double.POSITIVE_INFINITY;
  }

  /**
   * Modifies this APConstraints object's max velocity and returns itself. 
   * This affects the maximum velocity that Autopilot can demand.
   * 
   * @param velocity The maximum velocity that Autopilot will demand, in m/s
   */
  public APConstraints withVelocity(double velocity) {
    this.velocity = velocity;
    return this;
  }

  /**
   * Modifies this APConstraint object's acceleration and returns itself. This affects the maximum
   * acceleration that Autopilot will use to correct initial velocities.
   *
   * <p> Autopilot's acceleration is used at the beginning of an action (not relevant to Autopilot's end behavior).
   * 
   * @param acceleration The maximum acceleration that Autopilot will use to start a path, in m/s^2
   */
  public APConstraints withAcceleration(double acceleration) {
    this.acceleration = acceleration;
    return this;
  }

  /**
   * Modifies this constraint's max jerk value and returns itself. Higher values mean a faster
   * deceleration.
   * 
   * Autopilot's jerk is used at the end of an action (not relevant to Autopilot's start behavior).
   * 
   * @param jerk The maximum jerk that Autopilot will use to decelerate at the end of an action, in m/s^3
   */
  public APConstraints withJerk(double jerk) {
    this.jerk = jerk;
    return this;
  }
}
