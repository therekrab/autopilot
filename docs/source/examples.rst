Examples
========

This page will walk through some common use cases with Autopilot.

Drive To Point
--------------

Let's create a regular command that uses Autopilot to drive to a point.

.. note:: I prefer not to subclass commands - ever. For that reason, this code
   can really go wherever you construct commands.

Firstly, we need an instance of Autopilot. This can be a constant, because it
only needs to be created once and can be used anywhere in the program,
repeatedly. It also never changes.

With this, we must setup all of our motion constraints.

.. code-block:: java

   private static final APConstraints kConstraints = new APConstraints()
       .withAcceleration(5.0)
       .withJerk(2.0);

   private static final APProfile kProfile = new APProfile(kConstraints)
       .withErrorXY(Centimeters.of(2))
       .withErrorTheta(Degrees.of(0.5))
       .withBeelineRadius(Centimeters.of(8));

   public static final Autopilot kAutopilot = new Autopilot(kProfile);

This is all the configuration we need to run Autopilot. Now, let's define a
target. This can be done on the fly, or it can be a constant, too. Let's do the
second option:

.. code-block:: java

   APTarget target = new APTarget(targetPose)
       .withEntryAngle(Rotation2d.kZero);

This target aligns to some pose, ``targetPose``, and enters coming from the
left (a rotation of zero is represented by an arrow pointing right - the
robot's end direction of motion.

To use Autopilot, we simply need to call the ``calculate()`` method as part of
the execution of a command.

.. code-block:: java

   Command alignCommand = drivetrain.run(() -> {
     Translation2d velocities = drivetrain.getFieldRelativeSpeeds();
     Pose2d pose = drivetrain.getCurrentPose();

     Tranform2d output = Constants.kAutopilot.calculate(pose, velocities, target);

     /* these speeds are field relative */
     double veloX = output.getX();
     double veloY = output.getY();
     Rotation2d headingReference = output.getRotation();

     /* This is where you should apply these speeds to the drivetrain */
   });

Autopilot is designed to be usable on a vast variety of drivetrains and
systems, so it only provides the setpoints - it's up to the user to apply these
speeds.

To add an end condition to the command, we can use the ``until()`` decorator:

.. code-block:: java

   Command alignCommand = Commands.run(/* snip */)
       .until(() -> Constants.kAutopilot.atTarget(drivetrain.getCurrentPose(), target));

And to make sure that the drivetrain comes to a complete stop when we arrive at
the target:

.. code-block:: java

   Command alignCommand = Commands.run(/* snip */)
       .until(/* snip */)
       .finallyDo(drivetrain::stop);

CTRE drivetrain example
~~~~~~~~~~~~~~~~~~~~~~~

With a CTRE drivetrain, we need to create a ``SwerveRequest`` to apply our
velocities.

.. note:: This part shouldn't be part of ``run()``, rather this should be
   instantiated outside the command.

.. code-block:: java

   private SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
       .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
       .withDriveRequestType(DriveRequestType.Velocity)
       .withHeadingPID(4, 0, 0); /* change theese values for your robot */

From here, we can use this request to apply field-relative velocities:

(this code goes inside the lambda for ``run()``:

.. code-block:: java

   /* snip */
   double veloX = output.getX()
   double veloY = output.getY();
   Rotation2d headingReference = output.getRotation();
 
   drivetrain.setControl(m_request
      .withVelocityX(veloX)
      .withVelocityY(veloY)
      .withTargetDirection(headingReference));

Recommendations for code structure
----------------------------------

Currently, this code works well. However, it's not obvious where the command
should be generated, because we didn't subclass ``Command`` directly. However,
notice that the command only uses the drivetrain. So let's move this command
into a command factory method on the drivetrain itself:

.. code-block:: java

   public Command align(APTarget target) {
     return this.run(() -> {
       Translation2d velocities = this.getFieldRelativeSpeeds();
       Pose2d pose = this.getCurrentPose();

       Tranform2d output = Constants.kAutopilot.calculate(pose, velocities, target);

       /* these speeds are field relative */
       double veloX = output.getX();
       double veloY = output.getY();
       Rotation2d headingReference = output.getRotation();

       this.setControl(m_fieldRelativeRequest
           .withVelocityX(veloX)
           .withVelocityY(veloY)
           .withTargetDirection(headingReference));
     })
         .until(() -> Constants.kAutopilot.atTarget(this.getCurrentPose(), target)
         .finallyDo(this::stop);
   }

This lets us construct larger commands later and just call
``drivetrain.align()`` with a target, and the command is rebuilt each time.

Subclassing ``Command`` Example
-------------------------------

.. warning:: Creating commands like this is not recommended. They get verbose
   quickly and it's harder to find a good spot in code to add command
   decorators and group behavior together. But I'll show it anyways in case you
   want it.

Here's a complete version of a class, ``AlignCommand`` that does the same thing
as our other example (using CTRE drivetrain, for example, but you can use any
type of drivetrain):

.. code-block:: java

   public class AlignCommand extends Command {
     private final APTargeet m_target;
     private final Drivetrain m_drivetrain;

     private final SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
         .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
         .withDriveRequestType(DriveRequestType.Velocity)
         .withHeadingPID(4, 0, 0); /* tune this for your robot! */


     public AlignCommand(APTarget target, Drivetrain drivetrain) {
       m_target = target;
       m_drivetrain = drivetrain;
       addRequirements(drivetrain);
     }

     @Override
     public void initialize() {
       /* no-op */
     }

     @Override
     public void execute() {
       Translation2d velocities = m_drivetrain.getFieldRelativeSpeeds();
       Pose2d pose = m_drivetrain.getCurrentPose();

       Transform2d out = Constants.kAutopilot.calculate(pose, velocity, m_target);

       double veloX = out.getX();
       double veloY = out.getY();
       Rotation2d headingReference = out.getRotation();

       m_drivetrain.setControl(m_request
           .withVelcoityX(veloX)
           .withVelocityY(veloY)
           .withTargetDirection(headingReference));
     }

     @Override
     public boolean isFinished() {
       return Constants.kAutopilot.atTarget(m_drivetrain,getCurrentPose(), m_target);
     }

     @Override
     public void end(boolean interrupted) {
       m_drivetrain.stop();
     }
   }
