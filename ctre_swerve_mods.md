# Modifications to CTRE Swerve

We made the following modifications to the generated CTRE swerve project to enable using Choreo for autos.

CommandSwerveDrivetrain.java -- add the following plus necessary imports

```java
//
// in class definition portion, add
//

/* Swerve request to apply during field-centric path following */
private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();
private final PIDController m_pathXController = new PIDController(10,0,0);
private final PIDController m_pathYController = new PIDController(10,0,0);
private final PIDController m_pathThetaController = new PIDController(7,0,0);

//
// then add these three methods at the bottom of the file
//

/*
    * Creates a new Choreo AutoFactory for this drivetrain
    *
    * @return AutoFactory
    */
public AutoFactory createAutoFactory() {
    return createAutoFactory((sample, isStart) -> {});
}

/*
    * Creates a new AutoFactory with the given trajectory logger
    *
    * @param trajLogger Logger for this trajectory
    * @return AutoFactory instance
    */
public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajLogger) {
    return new AutoFactory(
        this::getPose,
        this::resetPose,
        this::followPath,
        true,
        this,
        trajLogger
        );
}

/*
    * Returns the robot's current pose from the drivetrain's state
    */
public Pose2d getPose() {
    Pose2d pose = getState().Pose;
    return pose;
}


/*
    * Follows the given field-centric path sample with PID
    *
    * @param sample Sample along the path to follow
    */
public void followPath(SwerveSample sample) {
    m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);
    var pose = getState().Pose;
    var targetSpeeds = sample.getChassisSpeeds();
    targetSpeeds.vxMetersPerSecond += m_pathXController.calculate(
        pose.getX(), sample.x
    );

    targetSpeeds.vyMetersPerSecond += m_pathYController.calculate(
        pose.getY(), sample.y
    );

    targetSpeeds.omegaRadiansPerSecond += m_pathThetaController.calculate(
        pose.getRotation().getRadians(), sample.heading
    );

    setControl(
        m_pathApplyFieldSpeeds.withSpeeds(targetSpeeds)
            .withWheelForceFeedforwardsX(sample.moduleForcesX())
            .withWheelForceFeedforwardsY(sample.moduleForcesY())
    );
}
```

Then, in Autos.java, you can do:

```java
//
// in the constructor, add
//

autoFactory = drivetrain.createAutoFactory();

//
// then define each auto routine like this:
//

public Command example() {
    // create a new Routine with the name "LeaveHome" - which should match the
    // trajectory you'll create next
    AutoRoutine routine = autoFactory.newRoutine("LeaveHome");
    // Read in the trajectory named "LeaveHome" - this has to be the case-sensitive
    // name from Choreo
    AutoTrajectory exampleTraj = routine.trajectory("LeaveHome");
    // when the routine is active (auto is enabled) run a sequence of commands - print a message, reset odometry,
    // then drive our trajectory
    routine.active().onTrue(Commands.sequence(Commands.print("running example auto"), exampleTraj.resetOdometry(),
            exampleTraj.cmd()));
    // finally, when that trajectory is done, stop the bot using the routine from Routines.java
    exampleTraj.done().onTrue(Commands.sequence(Commands.print("Auto Completed"), routines.stopBot()));
    return routine.cmd();
}

```
