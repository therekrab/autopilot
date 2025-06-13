package com.therekrab.autopilot;

/**
 * A class that holds constrain information for an autopilot action.
 * 
 * Constraints are max velocity, acceleration, and jerk.
 */
public class APConstraints {
  protected double velocity;
  protected double acceleration;
  protected double jerk;

  /** Creates a blank APConstraints */
  public APConstraints() {
    this.velocity = Double.POSITIVE_INFINITY; // Default to no limit on velocity
  }

  /**
   * Creates a new APConstraints with given max velocity, acceleration, and jerk
   */
  public APConstraints(double velocity, double acceleration, double jerk) {
    this.velocity = velocity;
    this.acceleration = acceleration;
    this.jerk = jerk;
  }

  /**
   * Creates a new APConstraints with a given max acceleration and jerk. Velocity is left unlimited
   */
  public APConstraints(double acceleration, double jerk) {
    this.acceleration = acceleration;
    this.jerk = jerk;
  }

  /** Unlimited constraints */
  public static APConstraints unlimited() {
    return new APConstraints(
        Double.POSITIVE_INFINITY,
        Double.POSITIVE_INFINITY,
        Double.POSITIVE_INFINITY);
  }

  /**
   * Modifies this constraint's max velocity and returns itself. This is the maximum velocity that
   * autopilot will demand.
   */
  public APConstraints withVelocity(double velocity) {
    this.velocity = velocity;
    return this;
  }

  /**
   * Modifies this constraint's max acceleration value and returns itself. This affects the maximum
   * acceleration that the autopilot action will use to correct initial velocities.
   *
   * This value is only used for the start of an autopilot action, not the end behavior.
   */
  public APConstraints withAcceleration(double acceleration) {
    this.acceleration = acceleration;
    return this;
  }

  /**
   * Modifies this constraint's max jerk value and returns itself. Higher values mean a faster
   * deceleration.
   *
   * This is only used at the end of an autopilot action, not the beginning.
   */
  public APConstraints withJerk(double jerk) {
    this.jerk = jerk;
    return this;
  }
}
