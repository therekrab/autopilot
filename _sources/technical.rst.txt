Under the Hood
==============

.. note::
   This page describes the algorithm behind Autopilot's path-following
   algorithm. This knowledge is **not** necessary to understand how Autopilot
   works from the end user's perspective and is only provided for those
   interested.

To see a live demo of the paths that Autopilot approaches, check out `this
<https://www.desmos.com/calculator/w7penby7ds>`_ Desmos graph. The purple dot
represents the robot's initial position.

Prerequisites
-------------

A general understanding of polar curves and calculus (Calc II is probably
suitable for this) is recommended to understand what is about to happen.

Setup
-----

The first thing that Autopilot does to prepare for the computations that follow
is by transforming the robot's current position into a position from the
target's point of view, with entry angle being counted as forwards (+x
direction). This also places the target at the origin. This is important.

After all computations are done with velocity calculations, Autopilot undoes
this transformation to return to a global coordinate frame.

Step 1: The curve
-----------------

Consider the curve :math:`r=a\theta^b`. By playing with the parameter
:math:`a`, the curve is scaled. By changing :math:`b`, the shape of the curve
itself is different.

This curve has a variety of useful applications:

1. For all :math:`b \gt 0`, :math:`r` equals 0 when :math:`\theta` is zero.
   This means that following the curve inwards always reaches the origin (which
   is where the target is in a target coordinate frame). Reaching the target is
   good.

2. Modifying :math:`a` only scales the curve. This means that if some point is
   placed on the cartesian plane and this curve is overlaid, scaling the curve
   by changing :math:`a` can cause the curve to touch a. Generating a path that
   includes the current position is good.

3. The curve is always decreasing for smaller values of :math:`\theta`. As
   :math:`\theta` approaches zero, the curve moves towards the center. Because
   :math:`\theta = 0` is aligned with the positive x axis, entry angle is
   always respected, which is good.

The value for :math:`a` must scale dynamically, but the value of :math:`b` is
left unrestricted. Autopilot chooses :math:`b` to be 1. There are two reasons for this:

a. The curve is generally smooth. The "eyeball test" determines that the curve
   looks generally good enough to be followed, even at high speeds. There are
   no "sharp" corners.

b. The arc length of the curve is easily found. Curves with :math:`b \ne 1` are
   much more difficult to solve for arc length. This is *very* important later.

Step 2: Scaling the curve
-------------------------

To determine the correct curve to use, the offset must be touching the curve.
Otherwise, the curve doesn't seem reasonable. To scale the curve, we must
convert the current position into polar coordinates. We will refer to the angle
part of the polar coordinate as :math:`\theta_1` and the radius will be
:math:`S`. Note that :math:`S` is the distance from the target to the current
position, or vice versa (it's the magnitude of displacement).

Once the current position is in polar coordinates, multiplying the curve
:math:`r = \theta` by the scale factor of :math:`S \over \theta_1`. This works
because when :math:`\theta = \theta_1`, the fraction :math:`\theta \over \theta_1`
cancels out and becomes 1, leaving :math:`S` - the correct radius for the curve at
that point.

Now the final equation for the curve is as follows:

:math:`r = {{ S \cdot \theta } \over \theta_1}`

.. note::
   This equation is **never** found in the code for Autopilot. This is just an
   explanation of *what* the curve is, and how it was developed.

Step 3: Finding direction
-------------------------

Autopilot doesn't care what the curve is. During one call to ``calculate()``,
the only part of the curve that matters right now is the direction that the
robot has to go in to follow the curve - the curve itself doesn't matter.

To find the direction (in cartesian X/Y coordinates for velocity) of the curve, the curve needs
to be converted to rectangular form with the following formulae:

:math:`x = r \cdot cos(\theta)`

:math:`y = r \cdot sin(\theta)`

Differentiating each term with respect to theta:

:math:`{dx \over d\theta} = r \cdot -sin(\theta) + {dr \over d\theta} \cdot cos(\theta)`

:math:`{dy \over d\theta} = r \cdot cos(\theta) + {dr \over d\theta} \cdot sin(\theta)`

To solve for :math:`dr \over d\theta`, we differentiate :math:`r = {S \cdot
\theta \over \theta_1}` to get :math:`{dr \over d\theta} = {S \over \theta_1}`.

Notice that :math:`r = {S \cdot \theta \over \theta_1}` can be rewritten as
:math:`r = {dr \over d\theta} \cdot \theta`. This means that we can pull the
term :math:`{dr \over d\theta}` out of the expressions for both :math:`dx \over
d\theta` and :math:`dy \over d\theta`:

:math:`{dx \over d\theta} = {dr \over d\theta} \cdot [\theta \cdot -sin(\theta) + cos(\theta)]`

:math:`{dy \over d\theta} = {dr \over d\theta} \cdot [\theta \cdot cos(\theta) + sin(\theta)]`

Because we want to find the direction of the curve at this point, we create the
following vector:

:math:`[{dx \over d\theta}, {dy \over d\theta}]`

But we only want direction, so we should normalize this vector to become a unit
vector (a vector with magnitude 1). This means that any constant that the
vector is multiplied by is "cancelled out". Because each component of the
vector is scaled by :math:`S \over \theta_1`, the entire vector is scaled by
:math:`S \over \theta_1`. But normalizing a vector ignores any scaling done, so
when we compute the actual velocity, we don't need to compute :math:`S \over
\theta_1` and can leave that part out.

Step 4: Calculate Speed
-----------------------

This section is divided into two parts, because it is quite long.

Pt. I
~~~~~

.. caution::
   This is a **long** section. Be prepared for lots of formulae and
   expressions.

Once we have a unit vector telling us in what direction we need to drive the
robot, it becomes important to know *how fast* we should go in that direction.

Because swerve drive uses an independent motor for drive and steer, it stands
to reason that changes in the direction of a path do not affect the robot's
speed. The only thing that affects the speed of the robot is the drive motors,
and the only thing that changing direction affects is the steer motors.

.. note::
   In this example, rotating is ignored. When the robot rotates, not all of the
   drive motors are going in the same direction, which is what creates the
   rotation in the chassis, so speed is limited.

The conclusion here is that we really only care about the distance that we
travel, so **we can express speed as a function of distance**.

The approach that Autopilot takes is as follows:

1. Calculate max theoretical velocity. This value is "theoretical" because it
   doesn't take into account the robot's current velocities. This value is also
   the maximum value that obeys the motion constraints, i.e. "max".

2. Adjust that velocity from the current velocity to obey acceleration limits (somewhat).

.. important:: If you're not already familiar with the three phases of an
   Autopilot flight, please read :doc:`about`

The actual implementation of this differs significantly from the implementation
used in WPILib's ``ProfiledPIDController``, which uses time to calculate the
current phase. Autopilot makes an effort to leave time out of the equation,
because it can often introduce far too much complexity when it doesn't need to.

The approach that Autopilot uses to calculate the theoretical max involves
*only* computing the ideal velocity as if the robot was in the landing phase.
Recall that the landing phase uses constant jerk. The first step in solving for
velocity with respect to distance is solving for velocity with respect to time,
:math:`v(t)`.

Now is probably a good time to take a moment and explain how we are going to
solve for :math:`v(x)`. If we integrate :math:`v(t)` to get :math:`x(t)`, and
then find its inverse, :math:`t(x)`, we can then plug in the result of
:math:`t(x)` into the velocity function to get the equation for the velocity
given some distance.

But the first step is finding :math:`v(t)`.

If you don't already know the physics equations for this, we will derive them
now using a Taylor series. But this isn't a very tricky Taylor series. In fact,
this is probably one of the easiest Taylor series computations ever. Here's why:

Firstly, the series is centered at :math:`t = 0`, and we know that at that
point, the robot's velocity should be 0. Because we also want zero acceleration
at the end of the path, we also can say that acceleration should be zero.
Finally, we know that we use constant jerk, so jerk is some constant :math:`j`.
Because jerk is constant with respect to time, we know that all derivates of
jerk are 0. Here's a table showing what we know:

.. list-table:: Derivates of Velocity
   :header-rows: 1

   * - :math:`n`
     - :math:`v^{(n)}(t)`

   * - :math:`0`
     - :math:`0`

   * - :math:`1`
     - :math:`0`

   * - :math:`2`
     - :math:`j`

   * - :math:`>2`
     - :math:`0`

Setting up a Taylor series here becomes trivial, because most terms are
multiplied by :math:`0`:

:math:`v(t) = 0 + 0 + {1 \over 2} jt^2 + 0 + ...`

:math:`v(t) = {1 \over 2} jt^2`

.. note::
   If :math:`t=0` represents the *end* of the path, does that mean that
   we're going to be expecting negative values for :math:`t`? I've decided that
   for simplicity, we'll actually say that :math:`t` actually represents the
   amount of time *until* landing. This doesn't mess with our equations, which
   is good.

Now we have to integrate this to solve for :math:`x(t)`, but this is pretty
simple:

:math:`\int{1 \over 2} jt^2 dt = {1 \over 3} \cdot {1 \over 2} jt^3 + C = {1 \over 6} jt^3 + C`

But now we have to define what the value of :math:`x(0)` should be (therefore
solving for :math:`C`). Using our target coordinate frame, we know that at
:math:`t=0`, our position in this coordinate frame should also be zero.
Therefore, :math:`C = 0`, leaving us with:

:math:`x(t) = {1 \over 6} jt^3`

We can now attempt to solve for this function's inverse, and we will get:

:math:`{6x \over j} = t^3`

:math:`t = ({6x \over j})^{1 \over 3}`

There we go! We've only got one step left: we plug in the expression for
:math:`t` we got here into our original equation for :math:`v(t)` to get a
function that tells us velocity as a function of distance, not time.

:math:`v(t) = v(({6x \over j})^{1 \over 3})`

:math:`= {1 \over 2} j [({6x \over j})^{1 \over 3}]^{2}`

:math:`= {1 \over 2} j ({6x \over j})^{2 \over 3}`

:math:`= {1 \over 2} (36jx^2)^{1 \over 3}`

:math:`= ({36 \over 8} j x^2)^{1 \over 3}`

Simplifying gives us the final equation:

:math:`v(x) = ({9 \over 2} j x^2)^{1 \over 3}`

This is **exactly** what we needed! This gives us the maximum velocity at some
distance to still be within the jerk constraint. But there's one more step to
be done: we still need to compensate for current speeds.

If the robot is at rest, we don't want to instantly command the max velocity.
We need to approach the goal velocity, which is where the acceleration
constraint comes into play.

.. important::
   For the rest of this section, I use "velocity" when I refer to
   "velocity in the goal direction of motion". These definitely are *not* the
   same thing, so it's important to note.

If we know the change in time among calls to Autopilot (robot periodic is 20
milliseconds), we can compute the maximum change in velocity that is acceptable
during this time by multiplying :math:`\Delta t` by our acceleration value.

This is the largest legal change in robot velocity if we always obey our
constraints. If the difference between the ideal velocity and the current velocity
is smaller than that change, then we can simply apply the ideal velocity.

Finally, the commanded velocity is capped at whatever the velocity constraint
is, if it exists.

Before I wrap up this section, it's important to note the times when Autopilot
doesn't obey its given constraints.

Firstly, if the current velocity is higher than the theoretical velocity, the
theoretical velocity is always applied, even if the change is greater than the
calculated max change. This is not a bug; this is a design choice. In a path,
smoothness is especially important *at the end* of the path. Because Autopilot
never demands a velocity higher than the ideal velocity, it is reasonable to
say that anytime when velocity is significantly higher than theoretical is due
to initial conditions. Rather than do its best while obeying constraints and
arrive at the target too fast (for end velocity of zero, nonzero end velocity
is bad), Autopilot will instead violate its constraints at the start as to
prevent issues later. In this situation, the tradeoff lies between jerky motion
at a point where that isn't critical to the final motion, or surprising the
user with unexpected behavior.

The second instance where Autopilot doesn't follow its own constraints is with
regards to motion that is not in the direction of the path. This motion can be
due to bad initial conditions, or - and more importantly - it can be due to a
discrete time step controller rather than a continuous motion. Autopilot doesn't
technically always follow its own path, but this is because from one cycle to
another, the robot should be travelling in a straight line - but this doesn't
follow with the path; therefore there will be a little error. Autopilot never
tells the robot to move in a different direction from the correct direction to
follow the path. Although this can very much risk sudden changes in velocity,
that is normally due to poor initial conditions for the path. Autopilot *could*
implement a different acceleration constraint to correct for velocity error,
but this adds another step that needs to be tuned. And what's worse is that for
the reasons mentioned above, this sort of correction is important and
improperly tuning it could cause Autopilot to tell the robot to oscillate or
exhibit weird behavior. This was another design decision - we want to make any
jerky motions at the start of an Autopilot action.
 
Pt. II
~~~~~~

We cannot assume that distance that we will need to travel is equal to the
magnitude of the displacement from the target (:math:`S` from earlier).
 
Notice that the length of the path that we have to travel is the length of the
curve from 0 to the current angle. This is called **arc length**, and we have
an integral expression to calculate it:

:math:`L = \int_0^{|\theta_1|}\sqrt{[r(\theta)]^2 + ({dr \over d\theta})^2}d\theta`

.. note::
   We take the absolute value of our angle because if we have an angle that's
   negative, we still want a positive length.

We already solved for :math:`{dr \over d\theta}` earlier when computing the
direction of velocity, so let's use it's value: :math:`S \over \theta_1`.

We substitute this expression in as well as the expression for
:math:`r(\theta)` and get:

:math:`L = \int_0^{|\theta_1|}\sqrt{({S \theta \over \theta_1})^2 + ({S \over
\theta_1})^2}d\theta`

:math:`= \int_0^{|\theta_1|}({S \over \theta_1})\sqrt{\theta^2 + 1}d\theta`

:math:`= ({S \over \theta_1})\int_0^{|\theta_1|}\sqrt{\theta^2 + 1}d\theta`

Next, a few things are going to happen: Firstly, we're going to forget (for
now) about the constant multiplier outside the integral. We'll return to it
later, but it's a trivial multiplication and not at all interesting. Secondly,
I'm going to replace all instances of :math:`\theta` with :math:`x`. Finally,
I'm going to also omit the bounds of integration. Again, they'll return later,
but they're not the special part here.

Now our integral to solve has become:

:math:`\int \sqrt {x^2 + 1}dx`

This integral is a fun nightmare to work out by hand, so feel free to skip if
it gets boring. You've been warned.

We can begin with a trig substitution:

:math:`\text{let } x = \tan(\theta)`

:math:`{dx \over d\theta} = \sec^2(\theta)`

:math:`dx = \sec^2(\theta)d\theta`

.. important::
   **Please** remember that this :math:`\theta` is **NOT** the same
   :math:`\theta` as before.

After this substitution, our integral becomes:

:math:`\int \sqrt{\tan^2(\theta) + 1} \cdot \sec^2(\theta)d\theta`

Recalling that :math:`\tan^2(a) + 1 = \sec^2(a)`:

:math:`\int \sqrt{\sec^2(\theta)} \cdot \sec^2(\theta)d\theta`

:math:`= \int \sec(\theta) \cdot \sec^2(\theta)d\theta`

Now, we can use integration by parts to "simplify" this expression:

.. list-table::
   :widths: 50 50
   :header-rows: 0

   * - :math:`u = \sec(\theta)`
     - :math:`v = \tan(\theta)`

   * - :math:`u' = \sec(\theta)\tan(\theta)`
     - :math:`v' = \sec^2(\theta)`

Thus, the integral becomes:

:math:`\sec(\theta)\tan(\theta) - \int \sec(\theta)\tan^2(\theta)d\theta`

:math:`= \sec(\theta)\tan(\theta) - \int \sec(\theta)[\sec^2(\theta) - 1]d\theta`

:math:`= \sec(\theta)\tan(\theta) - \int [\sec^3(\theta) - \sec(\theta)]d\theta`

:math:`= \sec(\theta)\tan(\theta) - \int \sec^3(\theta)d\theta + \int \sec(\theta)d\theta`

The integration of :math:`\sec(\theta)` is given to be :math:`\ln |\sec(\theta) +
\tan(\theta)| + C`. A proof of this is left to the reader.

Our integral transforms into:

:math:`\int \sec^3(\theta)d\theta = \sec(\theta)\tan(\theta) + \ln |\sec(\theta) +
\tan(\theta)| - \int \sec^3(\theta)d\theta`

Moving the integral term to the left side:

:math:`2\int \sec^3(\theta)d\theta = \sec(\theta)\tan(\theta) + \ln |\sec(\theta) + \tan(\theta)|`

Dividing each side by 2 solves the integral:

:math:`\int \sec^3(\theta)d\theta = {1 \over 2}[\sec(\theta)\tan(\theta) + \ln
|\sec(\theta) + \tan(\theta)|]`

But we're not done yet. We still need to move back into the :math:`x` world.

But this is what we know:

.. list-table::
   :header-rows: 0

   * - :math:`\tan(\theta)`
     - :math:`x`

   * - :math:`\sec(\theta)`
     - :math:`\sqrt{1 + x^2}`

We can substitute these values into the expression for the integral and get:

:math:`{1 \over 2}[x \sqrt{1+x^2} + \ln |x + \sqrt{1+x^2}|]`

Note that plugging :math:`x = 0` into this expression results in a value of
`0`. When we evaluate the integral from 0 to :math:`|\theta_1|`, we only need to
compute the value of this expression when we plug in :math:`x = |\theta_1|`.

But we're not quite done yet. We still need to remember that constant that we
multiplied the integral by: :math:`S \over \theta_1`.

Here's what happens when we plug that in:

:math:`{S \over 2\theta_1}[x \sqrt{1+x^2} + \ln |x + \sqrt{1+x^2}|]`

:math:`= {S \over 2\theta_1}x\sqrt{1+x^2} + {S \over 2\theta_1}\ln |x + \sqrt{1+x^2}|`

In the first term, something happens when we let :math:`x=\theta_1`. The fraction
:math:`\theta_1 \over \theta_1` reduces to just 1:

:math:`L = {S \over 2}\sqrt{1+\theta_1^2} + {S \over 2\theta_1}\ln |\theta_1 +
\sqrt{1+\theta_1^2}|`

This is the final formula that is found in the code for Autopilot, with one
exception: if :math:`theta_1 = 0`, the result is just :math:`S`. The reason for
this edge case is because if the angle is 0, the robot should only drive
straight in (and dividing by 0... computers don't like that).

This gives us a formula for the distance along the path that we have left to
travel. If we plug this into the formula for velocity given distance that we
found in pt. I, we are given a scalar by which we can multiply the vector we
got from Step 3, finally giving us a vector that represents the robot's
velocity.

Step 5: Wrapping it up
----------------------

At this point, the necessary calculations are in place; we know the correct
direction to drive in, and the correct speed to demand in that direction.

Finally, Autopilot returns this value to a global coordinate frame (remember,
this was all taking place in a target-centric coordinate frame?) and returns it
to the user.

Rotation
~~~~~~~~

Any rotation logic is much simpler, and does not merit its own section. This is
primarily due to the fact that Autopilot only returns a rotation setpoint, not
a rotational rate. Nevertheless, I will explain how Autopilot decides what
direction to request.

The first check Autopilot makes is whether the current target has a rotation
radius. If it doesn't, then there are no restraints on whether the target's
rotation is demanded. Therefore, the target's rotation is demanded.

If a rotation radius is in place, then Autopilot looks at the current distance
to the target. Is it smaller than the rotation radius? If so, then the target's
rotation is demanded. Otherwise, Autopilot simply returns the original rotation.

A quick note about entry angle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The most pedantic readers will notice one discrepancy between how entry angle
is described here and its behavior in code. This difference can be explained with
the following image of an Autopilot trajectory:

.. image:: entry-angle.png
   :width: 400

This graph is viewed from the target's coordinate frame, so the positive X axis
represents the direction the robot should come from. However, consider the case
when entry angle is zero. Then, this image looks exactly like the
field-relative view of this motion.

The technicallity here versus what is observed in practice with Autopilot is
the fact that here, the robot should approach from the positive X direction,
but in the real world, the robot approaches from the opposite direction. This
is not a bug. This is done to make it easier to imagine the correct entry angle
when writing code with Autopilot. Imagine an angle of 0. Does the angle point
to the left or to the right? Most will say that the angle starts at the origin
and points right. But an entry angle of zero, as shown in the image, actually
would cause the robot to approach *from* the right, i.e. would end going to the
left. This inverts the behavior that seems reasonable. Therefore, Autopilot
quietly flips the given entry angle so that it is the angle that the robot ends
travelling in.

This seems like the "least surprising" meaning of entry angle. Consider the
entry angle as if it was drawn as a vector pointing in that direction. The
arrow (if you picture it that way) that the vector points is the way that the
robot approaches from. It seems to be the more intuitive design.
