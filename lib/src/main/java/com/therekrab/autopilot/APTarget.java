package com.therekrab.autopilot;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

/**
 * A class representing the goal end state of an autopilot action
 * 
 * A target needs a reference Pose2d, but can optionally have a specified entry angle and rotation
 * radius
 *
 * A target may also specify an end velocity.
 *
 * The target also may have a desired end velocity.
 */
public class APTarget {
  protected Pose2d m_reference;
  protected Optional<Rotation2d> m_entryAngle;
  protected double m_velocity;
  protected Optional<Distance> m_rotationRadius;

  /**
   * Creates a autopilot target with reference (0,0) and rotation of zero.
   *
   * This target has an end velocity of 0, and no specified entry angle.
   */
  public APTarget() {
    m_reference = Pose2d.kZero;
    m_entryAngle = Optional.empty();
    m_velocity = 0;
    m_rotationRadius = Optional.empty();
  }

  /**
   * Creates a new autopilot target with the given target pose, no entry angle, and no end velocity
   */
  public APTarget(Pose2d pose) {
    m_reference = pose;
    m_velocity = 0;
    m_entryAngle = Optional.empty();
    m_rotationRadius = Optional.empty();
  }

  /**
   * Returns a copy of this target with the given reference.
   */
  public APTarget withReference(Pose2d reference) {
    APTarget target = this.clone();
    target.m_reference = reference;
    return target;
  }

  /**
   * Returns a copy of this target with the given entry angle.
   */
  public APTarget withEntryAngle(Rotation2d entryAngle) {
    APTarget target = this.clone();
    target.m_entryAngle = Optional.of(entryAngle);
    return target;
  }

  /**
   * Returns a copy of this target with the given end velocity.
   */
  public APTarget withVelocity(double velocity) {
    APTarget target = this.clone();
    target.m_velocity = velocity;
    return target;
  }

  /**
   * Returns a copy of this target with the given rotation radius.
   *
   * Rotation radius is the distance from the target pose that rotation goals are respected. By
   * default, rotation goals are always respected, but if autopilot shouldn't reorient the robot
   * until X distance from setpoint, this can be used to make that change.
   */
  public APTarget withRotationRadius(Distance radius) {
    APTarget copy = this.clone();
    copy.m_rotationRadius = Optional.of(radius);
    return copy;
  }

  /**
   * Returns this target's reference pose.
   */
  public Pose2d getReference() {
    return m_reference;
  }

  /**
   * Returns this target's desired entry angle.
   */
  public Optional<Rotation2d> getEntryAngle() {
    return m_entryAngle;
  }

  /**
   * Returns this target's end velocity.
   */
  public double getVelocity() {
    return m_velocity;
  }

  /**
   * Returns this target's rotation radius.
   */
  public Optional<Distance> getRotationRadius() {
    return m_rotationRadius;
  }

  /**
   * Creates a copy of this APTarget
   */
  public APTarget clone() {
    APTarget target = new APTarget();
    target.m_reference = m_reference;
    target.m_velocity = m_velocity;
    target.m_entryAngle = m_entryAngle;
    target.m_rotationRadius = m_rotationRadius;
    return target;
  }
}
