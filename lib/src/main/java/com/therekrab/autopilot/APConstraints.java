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

  /**
   * Creates a blank APConstraints object.
   * <p>
   * A blank APConstraints will not limit velocity, acceleration, or jerk.
   */
  public APConstraints() {
    this(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
  }

  /**
   * Creates a new APConstraints object with a given max velocity, acceleration, and jerk.
   * 
   * @param velocity The maximum velocity that Autopilot will demand, in m/s
   * @param acceleration The maximum acceleration that Autopilot action will use to correct initial
   *        velocities, in m/s^2
   * @param jerk The maximum jerk that Autopilot will use to decelerate at the end of an action, in
   *        m/s^3
   */

  public APConstraints(double velocity, double acceleration, double jerk) {
    this.velocity = velocity;
    this.acceleration = acceleration;
    this.jerk = jerk;
  }

  /**
   * Create a new APConstraints object with a given max acceleration and jerk.
   * <p>
   * This constructor defaults the velocity to unlimited.
   * 
   * @param acceleration The maximum acceleration that Autopilot action will use to correct initial
   *        velocities, in m/s^2
   * @param jerk The maximum jerk that Autopilot will use to decelerate at the end of an action, in
   *        m/s^3
   * 
   */
  public APConstraints(double acceleration, double jerk) {
    this(Double.POSITIVE_INFINITY, acceleration, jerk);
  }

  /**
   * Copies this APConstraints object, changes the copy's max velocity, and returns the copy. This
   * affects the maximum velocity that Autopilot can demand.
   * 
   * @param newVelocity The maximum velocity that Autopilot will demand, in m/s
   */
  public APConstraints withVelocity(double newVelocity) {
    return new APConstraints(newVelocity, this.acceleration, this.jerk);
  }

  /**
   * Copies this APConstraint object, changes the copy's acceleration, and returns the copy. This
   * affects the maximum acceleration that Autopilot will use to correct initial velocities.
   *
   * <p>
   * Autopilot's acceleration is used at the beginning and end of an action
   * 
   * @param newAcceleration The maximum acceleration that Autopilot will use to start a path, in
   *        m/s^2
   */
  public APConstraints withAcceleration(double newAcceleration) {
    return new APConstraints(this.velocity, newAcceleration, this.jerk);
  }

  /**
   * Copies this APConstraint object, changes the copy's max jerk, and returns the copy. Higher
   * values mean a faster deceleration.
   * 
   * Autopilot's jerk is used at the end of an action (not relevant to Autopilot's start behavior).
   * 
   * @param newJerk The maximum jerk that Autopilot will use to decelerate at the end of an action,
   *        in m/s^3
   */
  public APConstraints withJerk(double newJerk) {
    return new APConstraints(this.velocity, this.acceleration, newJerk);
  }
}
