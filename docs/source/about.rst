Why Autopilot?
==============

Autopilot aims to cross the bridge between powerful motion control, which is often very complex, and
simple motion control that is limited in functionality.

Autopilot's goal is to be **powerful** yet **simple**, and this is reflected heavily throughout this
documentation.

Simple stateless control
------------------------

Top-level teams in 2025 used simple straight-line control, presumably using PID
control or a motion profile to travel in a line across the field. Stringing
these together could create fast and reliable motion. This type of controller
also doesn't look ahead into the future, meaning that it doesn't have to
generate a whole path to follow. Instead, controllers like a PID controller
work because they are cleverly designed to naturally arrive at the setpoint.
The actual path isn't generated - only some input depending on current state.

This solution is simple, but can be limiting. By constraining the motion to a
straight line, less paths are possible, or implementing them would be more
complex or difficult.

Advanced, time-based path following
-----------------------------------

On the other end of path following complexity, there are solutions like Choreo
and PathPlanner. These solutions often involve pregenerated paths, and trying
to follow those paths. This provides a significant amount of control over the
robot's behavior, but it does have a problem that can become serious.

These types of control systems are often based on time, not robot position.
This means that if the robot's initial position doesn't match with the start
position of the path, there can be issues following the path where the path
"ends" before it really should, leaving a large amount of error.

The solution to this is to implement dynamic, on-the-fly path generation so
that paths always start from the robot's actual positoin. However, because
these are not prebuilt, the entire path has to be generated on-the-fly.
Depending on the algorithm used, this can be computationally intensive. Also,
because errors can occur, the path may need to be regenerated repeatedly to
maintain a reasonable path.

Where Autopilot fits in
-----------------------

Autopilot offers a tradeoff between these two solutions, taking the best from
each. Similarly to simple straight-line control, Autopilot never builds a path
that the robot needs to follow. This prevents excess calculations, and makes
the path robust to error. It will find its way to the target, but it will not
follow a pregenerated path to arrive there.

Autopilot also isn't constrained to simple linear motion. Autopilot uses a
property called **entry angle**, which refers to the angle that the robot
should end the motion going. This is the angle from which the robot will
approach the target. Although not as customizable as bezier curves, this
doesn't need to be precomputed beforehand - making it much faster.

Smooth motion profile
---------------------

Another feature of Autopilot that makes it ideal for autonomous robot control
is its motion profile that it uses. Autopilot uses a custom motion profile that
provides both **fast** and **fluid** travel.

The way this works is by breaking the path into different sections: takeoff and
landing. Takeoff is the phase where the robot is accelerating. During this part
of the travel, the robot is commanded to accelerate at a constant rate. This
provides fast initial movement, which is typically the part of the path that
doesn't require the most accuracy, so getting more speed more quickly is
reasonable. The last phase of the robot's motion, where it approaches the
target and is decelerating, is called the landing. During this part of an autopilot routine,
the robot moves with decreasing acceleration, providing maximum smoothness.
