package com.therekrab.autopilot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * Autopilot is a class that tries to drive a target to a goal in 2 dimensional space.
 *
 * Autopilot is a fast algorithm because it doesn not think ahead. Any and all math is already
 * worked out such that only a small amount of computation is necessary on the fly.
 *
 *
 * This means that autopilot is un able to avoid obstacles, because it cannot think ahead.
 */
public class Autopilot {
  private APProfile m_profile;

  private final double dt = 0.020;

  /**
   * Constructs an Autopilot from a given profile. This is the profile that the autopilot will use
   * for all actions.
   */
  public Autopilot(APProfile profile) {
    m_profile = profile;
  }

  /**
   * Returns the next field relative velocity for the trajectory
   *
   * @param current The robot's current position.
   * @param robotRelativeSpeeds The robot's current <b>robot relative</b> ChassisSpeeds.
   * @param target The target the robot should drive towards.
   */
  public APResult calculate(Pose2d current, ChassisSpeeds robotRelativeSpeeds, APTarget target) {
    Translation2d offset = toTargetCoorinateFrame(
        target.m_reference.getTranslation().minus(current.getTranslation()), target);

    if (offset.equals(Translation2d.kZero)) {
      return new APResult(
          MetersPerSecond.zero(),
          MetersPerSecond.zero(),
          target.m_reference.getRotation());
    }

    Translation2d fieldRelativeSpeeds = new Translation2d(
        robotRelativeSpeeds.vxMetersPerSecond,
        robotRelativeSpeeds.vyMetersPerSecond).rotateBy(current.getRotation());

    Translation2d initial = toTargetCoorinateFrame(fieldRelativeSpeeds, target);
    double disp = offset.getNorm();
    if (target.m_entryAngle.isEmpty() || disp < m_profile.beelineRadius.in(Meters)) {
      Translation2d towardsTarget = offset.div(disp);
      Translation2d goal = towardsTarget.times(calculateMaxVelocity(disp, target.m_velocity));
      Translation2d out = correct(initial, goal);
      Translation2d velo = toGlobalCoordinateFrame(out, target);
      Rotation2d rot = getRotationTarget(current.getRotation(), target, disp);
      return new APResult(MetersPerSecond.of(velo.getX()), MetersPerSecond.of(velo.getY()), rot);
    }
    Translation2d goal = calculateSwirlyVelocity(offset, target);
    Translation2d out = correct(initial, goal);
    Translation2d velo = toGlobalCoordinateFrame(out, target);
    Rotation2d rot = getRotationTarget(current.getRotation(), target, disp);
    return new APResult(MetersPerSecond.of(velo.getX()), MetersPerSecond.of(velo.getY()), rot);
  }

  /**
   * Turns any other coordinate frame into a coordinate frame with positive x meaning in the
   * direction of the target's entry angle, if applicable (otherwise no change to angles).
   */
  private Translation2d toTargetCoorinateFrame(Translation2d coords, APTarget target) {
    Rotation2d entryAngle = target.m_entryAngle.orElse(Rotation2d.kZero);
    return coords.rotateBy(entryAngle.unaryMinus());
  }

  /**
   * Turns a translation from a target-relative coordinate frame to a global coordinate frame.
   */
  private Translation2d toGlobalCoordinateFrame(Translation2d coords, APTarget target) {
    Rotation2d entryAngle = target.m_entryAngle.orElse(Rotation2d.kZero);
    return coords.rotateBy(entryAngle);
  }

  /**
   * Determines the maximum velocity required to travel the given distance and end at the desired
   * end velocity.
   */
  private double calculateMaxVelocity(double dist, double endVelo) {
    return Math.pow((4.5 * Math.pow(dist, 2.0)) * m_profile.constraints.jerk, 1.0 / 3.0)
        + endVelo;
  }

  /**
   * Attempts to drive the initial translation to the goal translation using the parameters for
   * acceleration given in the profile.
   */
  private Translation2d correct(Translation2d initial, Translation2d goal) {
    Rotation2d angleOffset = Rotation2d.kZero;
    if (!goal.equals(Translation2d.kZero)) {
      angleOffset = new Rotation2d(goal.getX(), goal.getY());
    }
    Translation2d adjustedGoal = goal.rotateBy(angleOffset.unaryMinus());
    Translation2d adjustedInitial = initial.rotateBy(angleOffset.unaryMinus());
    double initialI = adjustedInitial.getX();
    double goalI = adjustedGoal.getX();
    // we cap the adjusted I because we'd rather adjust now than overshoot.
    if (goalI > m_profile.constraints.velocity) {
      goalI = m_profile.constraints.velocity;
    }
    double adjustedI = Math.min(goalI,
        push(initialI, goalI, m_profile.constraints.acceleration));
    return new Translation2d(adjustedI, 0).rotateBy(angleOffset);
  }

  /**
   * Using the provided acceleration, "pushes" the start point towards the end point.
   *
   * This is used for ensuring that changes in velocity are withing the acceleration threshold
   */
  private double push(double start, double end, double accel) {
    double maxChange = accel * dt;
    if (Math.abs(start - end) < maxChange) {
      return end;
    }
    if (start > end) {
      return start - maxChange;
    }
    return start + maxChange;
  }

  /**
   * Uses the swirly method to calculate the correct velocities for the robot, respecting entry
   * angles
   *
   * @param offset The offset from the robot to the target, in the target's coordinate frame
   */
  private Translation2d calculateSwirlyVelocity(Translation2d offset, APTarget target) {
    double disp = offset.getNorm();
    Rotation2d theta = new Rotation2d(offset.getX(), offset.getY());
    double rads = theta.getRadians();
    double dist = calculateSwirlyLength(rads, disp);
    double vx = theta.getCos() - rads * theta.getSin();
    double vy = rads * theta.getCos() + theta.getSin();
    return new Translation2d(vx, vy)
        .div(Math.hypot(vx, vy)) // normalize
        .times(calculateMaxVelocity(dist, target.m_velocity)); // and scale to new length
  }

  /**
   * Using a precomputed integral, returns the length of the path that the swirly method generates.
   *
   * More specificallu, this calcualtes the arc length of the polar curve r=theta from the given
   * angle to zero, then scales it to match the current state.
   */
  private double calculateSwirlyLength(double theta, double radius) {
    if (theta == 0) {
      return radius;
    }
    theta = Math.abs(theta);
    double hypot = Math.hypot(theta, 1);
    double u1 = radius * hypot;
    double u2 = radius * Math.log(theta + hypot) / theta;
    return 0.5 * (u1 + u2);
  }

  /**
   * Returns the correct target heading for the current state
   */
  private Rotation2d getRotationTarget(Rotation2d current, APTarget target, double dist) {
    if (target.m_rotationRadius.isEmpty()) {
      return target.m_reference.getRotation();
    }
    double radius = target.m_rotationRadius.get().in(Meters);
    if (radius > dist) {
      return target.m_reference.getRotation();
    } else {
      return current;
    }
  }

  /**
   * Returns whether the given pose is within tolerance for the target
   */
  public boolean atTarget(Pose2d current, APTarget target) {
    Pose2d goal = target.m_reference;
    boolean okXY = Math.hypot(current.getX() - goal.getX(),
        current.getY() - goal.getY()) <= m_profile.errorXY.in(Meters);
    boolean okTheta = Math.abs(current.getRotation().minus(goal.getRotation())
        .getRadians()) <= m_profile.errorTheta.in(Radians);
    return okXY && okTheta;
  }

  /**
   * The resultant motion from a call to <code>Autopilot.calculate()</code>
   */
  public record APResult(LinearVelocity vx, LinearVelocity vy, Rotation2d targetAngle) {
  }
}
