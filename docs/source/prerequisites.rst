Prerequisites
=============

Autopilot is designed to be easy to implement, however it still requires a few
features from the robot in order to ensure that Autopilot works as desired.

What's *necessary*?
-------------------

The following functionality on your robot *must* be present in order to run
Autopilot:

Field-relative position
~~~~~~~~~~~~~~~~~~~~~~~

Your robot project must include a manner to access the current, field-relative
position of the robot. Autopilot needs to know where the robot is on the field
as well as where the target is in order for a trajectory to be constructed.

Autopilot expects these position values as the ``Pose2d`` type.

Robot-relative speeds
~~~~~~~~~~~~~~~~~~~~~

Autopilot also needs to know how fast the robot is going when
``Autopilot.calculate()`` is called. Autopilot expects these values to be in
the robot's frame of reference, following the `WPILib coordinate system
<https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#wpilib-coordinate-system>`_.

If you're using a CTRE generated drivetrain, the method here is
``getRobotRelativeSpeeds()``.

Autopilot expects speeds of the type ``ChassisSpeeds``.

Apply field-relative speeds
~~~~~~~~~~~~~~~~~~~~~~~~~~~

After Autopilot knows where the robot is and where it's going, it's going to
return a field-relative velocity. This velocity is from the same reference
frame as the field-relative position as earlier.

The robot project needs to have the ability to apply field-relative velocities.
Again, with a CTRE drivetrain, the appropriate swerve request is going to be
``FieldCentricFacingAngle``.

.. note:: With CTRE's ``FieldCentricFacingAngle`` request, ensure that you set
   the forward perspective as the blue alliance with
   ``.withForwardPerspective(ForwardPerspectiveValue.BlueAlliance``. This
   ensures that the velocity is applied from the correct frame of reference.

Apply field-relative angular setpoint
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Autopilot is designed with translation motion in mind. For this reason,
rotation control is left up to the user. A profiled PID controller on the
robot's angle is fine.

With CTRE drivetrains, the ``FieldCentricFacingAngle`` request allows the user
to supply P, I, and D values as well as a field-relative angular setpoint.

What's *recommended*?
---------------------

Although you can run Autopilot the following features in your robot code, it is
*severely* recommended to ensure their presence such that the robot acts in the
desired manner.

Accurate drivetrain characterization
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Autopilot demands a field-relative velocity, but this is only as good as the
drivetrain's ability to follow those demanded setpoints. The drivetrain needs
to be able to accurately *and* quickly follow those demanded velocities.

If the constraints passed to Autopilot have an acceleration constraint *but*
the robot fails to quickly follow the demanded velocities, then the measured
acceleration can be significantly lower than expected. 

Feedforward constants such as ``kA`` and ``kS`` can help the drivetrain quickly
follow velocity references.

Accurate vision/localization
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Although a field-relative pose estimate is necessary to run Autopilot, its
performance is significantly improved when the system powering those
field-relative estimates - i.e. localization with vision updates - is more
accurate. Jittery vision causes jittery motion through Autopilot.
