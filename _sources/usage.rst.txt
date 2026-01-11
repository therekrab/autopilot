Usage
=====

Autopilot is structured off of four fundamental classes, each playing an
important role.

``APConstraints``
-----------------

The ``APConstraints`` class carries with it all the limits on the robot's
motion. These are:

- Velocity: The speed of the robot that the takeoff phase will stop at. The
  robot will never be demanded to go faster than this speed. Units are in
  meters per second.

- Acceleration: The maximum acceleration that the robot will obey in the
  takeoff phase. The units are meters per second squared.

- Jerk: The change in acceleration that will be used during the landing phase.
  These units are meters per second cubed.

With the constructor ``new APConstraints()``, velocity is uncapped, and the other
two values are set to zero. With the constructor ``new APConstraints(double
acceleration, double jerk)``, velocity is left unlimited.

Unless there's a need for a limit on velocity, it is recommended to
leave the velocity unlimited, and instead governed by the robot's physical maximum
speed.

You can find docs on ``APConstraints`` 
`here <https://therekrab.github.io/autopilot/javadoc/com/therekrab/autopilot/APConstraints.html>`_.

``APProfile``
-------------

The ``APProfile`` class holds all the data associated with a robot's behavior.
This includes its constraints, as well as its tolerated error and beeline
radius. Here's their meanings:

- Constraints: An instance of ``APConstraints`` that will be used to limit the
  robot's behavior.

- Tolerated error: The maximum error at which an instance of autopilot
  configured with this profile will regard as "at target". This consists of a
  translational error as well as rotational.

- Beeline radius: To prevent overshooting the target and attempting to back up
  again, there is a given radius below which the "beeline" strategy (drive
  straight towards target) takes over, even if an entry angle is desired. This should be set to a small
  value.

The default constructor for this class, ``new APProfile(APConstraints
constraints)``, sets all values to zero. More documentation for this class can
be found at the docs `here
<https://therekrab.github.io/autopilot/javadoc/com/therekrab/autopilot/APProfile.html>`_.

``APTarget``
------------

This is a class that stores information about a target. This includes:

- Reference: This is the goal state, and includes both heading and position.

- Entry angle: This is an optional angle that tells autopilot what direction
  from the target the robot should come from.

- End velocity: The ideal end velocity of the path. If this is greater than the
  maximum velocity set by a profile's constraints, or the robot is incapable of
  reaching this velocity, Autopilot will continue to demand a higher velocity,
  but this will not interfere with whether the path is complete or not. If this
  is not supplied, the robot will always drive directly at the target.
  
  If this value is nonzero, it is recommended to use a profile with a large
  amount of translational tolerance, because overshooting and never being within
  tolerance could cause the robot to turn around and drive towards the target again.

- Rotation radius: The (optional) distance is the minimum distance for the
  robot to begin to rotate towards the target. If this is left unset, there is
  no limit on when the robot can rotate, and it will be told to rotate as soon
  as the flight begins.

The default constructor, ``new APTarget()`` sets the reference pose to
``Pose2d.kZero``, sets the target velocity to 0, and leaves both entry angle
and rotation radius unset. The other constructor, ``new APTarget(Pose2d
reference)`` does the same thing except sets the reference pose as supplied.

The docs on ``APTarget`` can be found `here
<https://therekrab.github.io/autopilot/javadoc/com/therekrab/autopilot/APTarget.html>`_

``Autopilot``
-------------

This is the class that actually handles any computations. This has two public
methods, ``calculate(Pose2d current, ChassisSpeeds robotRelativeSpeeds,
APTarget target)`` and ``atTarget(Pose2d current, APTarget target)``.

The ``calculate(Pose2d current, Translation2d velocity, APTarget target)``
method computes the field-relative speeds of the robot and its ideal heading at
this point.

.. important:: The rotation component of the result from Autopilot is a heading
   setpoint - it is up to the user to find another form of control (Profiled
   PID is recommended) to reach the heading goal.

The ``atTarget(Pose2d current, APTarget target)`` method is a method that
returns whether the current pose of the robot is within the ``Autopilot``'s
tolerances. Read more about the ``Autopilot`` class `here
<https://therekrab.github.io/autopilot/javadoc/com/therekrab/autopilot/Autopilot.html>`_

