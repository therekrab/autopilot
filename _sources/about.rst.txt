What is Autopilot?
==================

Autopilot is a solution to the problem of holonomic motion control. It aims to
cross the bridge between powerful motion control, which is often very complex,
and simple motion control that is limited in functionality.

Autopilot's goal is to be **powerful** yet **simple**, and this is reflected heavily throughout this
documentation.

Autopilot plays the same role as tools like PathPlanner or Choreo - to an
extent. Autopilot doesn't work with "trajectories" - a plan of where the robot
should be in the future. Instead, Autopilot uses the same methodology as a PID
controller - it has a clever system in place to determine the correct output to
reach a reference *without ever planning ahead*. It's easy to see why this can
be beneficial; without the robot trying to develop a plan for the future,
Autopilot relies on a much less computationally expensive design.

The primary goal of Autopilot is not to follow some trajectory or avoid
obstacles, or even to move the robot in the fastest manner from point A to
point B. Instead, Autopilot excels at robot motion control on the fly, where
the exact path cannot be predetermined, but constraints still need to be in
place.

Smooth Robot Motion
-------------------

Autopilot uses a custom motion profile for maximizing both speed and smoothness
in a path. The sections of this profile can be broken into three major
sections: the takeoff, the glide, and the landing. Each of these has their own
behavior that optimizes motion across the flight.

Takeoff
~~~~~~~

The takeoff portion of an Autopilot flight is the process of accelerating the
robot to a maximum velocity. This is the first stage in any flight, and uses a
constant acceleration to reach speed quickly.

Glide
~~~~~

The glide is the simplest phase of an Autopilot flight. After the top speed is
reached, the robot will stay at its glide speed until landing. This is a
constant velocity.

Landing
~~~~~~~

The landing is the final part of a flight and involves decelerating the robot.
Up to this point, what I have been describing fits the description of a
trapezoid profile. But this is where Autopilot breaks off from that. Instead,
Autopilot uses constant-jerk deceleration rather. This means that as the robot
approaches the target position, the acceleration of the flight approaches zero.
However, it does this faster than a PID controller, where all derivatives of
position approach zero. In the landing phase, the robot experiences constant
jerk.

The reason that the entire curve doesn't implement an S-curve for its motion
profile is simple. An S-curve is very smooth, but it can result in slightly
more time to reach setpoint. There is no real reason to have the robot smooth
at the start of a path. The only time that smoothness *really* matters is
during the landing period - and in that, it's really only important during the
end of the landing period.

The profile used with Autopilot is, to my knowledge, unique in FRC.

Midpoint between complex and simple
-----------------------------------

Autopilot is more complex than a simple drive-to-point implementation that
moves the robot in a straight line across the field, but is less complex than a
solution that fully generates a trajectory to follow. There are advantages and
disadvantages to this tradeoff.

What Autopilot Gives You
~~~~~~~~~~~~~~~~~~~~~~~~

Autopilot offers all the functionality of a simple drive-to-point controller
that uses a "beeline" path - one that drives straight towards the target
location. However, Autopilot also offers more. 

Autopilot can optionally respect **entry angle** - the desired direction that
the robot should approach from. By respecting entry angle, more complex paths
can be created than a beeline controller could create. This adds curvature to
the generated paths, and can be used to ensure the robot ends going the right
way. For example, entry angle could be used in 2025's Reefscape to ensure that
the robot always finished the path travelling directly toward the reef branch,
and not sideswiping the reef.

**Example**:

.. figure:: entry-angle.png
   :width: 500

The image above shows two methods of driving that Autopilot allows; The dashed
black line represents the beeline path. The robot drives directly in the
direction of the target and makes a straight line. The second, solid purple
line, is the path that the robot will follow if it's entry angle is 0 degrees
(to the right along the x axis).

.. important::
   Entry angle is not the goal direction of the robot from the
   target. Instead, it is the goal direction **of the target from the robot**.
   This means it is the direction that the robot will end up going right before
   it reaches the target.

Autopilot also offers control over rotation, through the use of an optional
**rotation radius**. The rotation radius is the minimum distance from the
target that the robot must be to rotate in the direction of the target. This
means that the robot can hold its heading until within some distance of the
target, giving more control over *when* the robot rotates.

What Autopilot Leaves Out
~~~~~~~~~~~~~~~~~~~~~~~~~

The other major feature that comes with tools like PathPlanner and Choreo is
the ability to manually construct paths and modify the shape of the robot's
motion. But Autopilot was never meant to stand in as a replacement for complex
paths that require such nuanced behavior. Autopilot offers a fast and smooth
solution that works in most use cases. Autopilot also does not offer any
solutions for obstacle avoidance. 

Path customizability is often not a necessary feature. Teams in the uppermost
echelon of FRC, like 2056 and 2910, have used (very successfully) beeline paths
during autonomous routines. Autopilot continues on from this simple base and
respects the need to have straightforward autonomous navigation, while still
coming with more features than its predecessor.
