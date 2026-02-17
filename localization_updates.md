## Add vision measurments to robot odometry

Update Robot.java's robotPeriodic() function with the following. This will help update the robot's notion of where it is on the field by using triangulation with the known locations of April Tags.

```java
// add this in Robot.java
@Override
public void robotPeriodic() {
  // uses the getPose() function we added to the Drivetrain
  LimelightHelpers.SetRobotOrientation("limelight", drivetrain.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
  LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
  if(mt2.tagCount > 0) {
    drivetrain.addVisionMeasurement(
        mt2.pose,
        mt2.timestampSeconds);
    }
}
```

## Getting the distance to the hub using field localization

Update the TurretControl.java class's getDistance() function or create a new function:

```java
// put at the top of the file in class def'n and constructor
private final AprilTagFieldLayout aprilTags;
aprilTags = AprilTagFields.k2026RebuiltWelded.loadAprilTagLayoutField();
aprilTags.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

// We need a function to get the bot's pose (it's notion of position & orientation on the field)
private Pose2D getCurrentBotPose() {
    // There are multiple ways to do this. We should experiment to see which works best
    // Method 1: Using the Limelight's Megatag2 pose estimator
    PoseEstimate botPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    Pose2d currentBotPose = botPoseEstimate.pose.toPose2d();
    // there are other properties of that PoseEstimate, see
    // https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib#poseestimate

    // Method 2: Using LimelightHelpers.java's get pose function
    // public Pose2d getRobotPose_FieldSpace2D() {
    //     return toPose2D(robotPose_FieldSpace);
    // }
    // currentBotPose = getRobotPose_FieldSpace2D();

    // Method 3: Using CTRE's swerve drivetrain's get pose function
    // var state = drivetrain.getState();
    // // pull out the pose estimate and chassis speeds
    // Pose2d currentBotPose = state.Pose;
    // ChassisSpeeds speeds = state.Speeds;

    return currentBotPose;
}

// Method 1 where we grab the ID of the tag that the Limelight sees
public Distance getDistanceToVisibleTag() {
    // really large distance so if a tag isn't visible we get an unrealistic distance value
    double distance = 999;
    Pose2D currentBotPose = getCurrentBotPose();
    // get the ID of the AprilTag the Limelight currently sees
    int aprilTagID = LimelightHelpers.getFiducialID();
    // get the pose of that tag
    Optional<Pose3d> tagPose = aprilTags.getTagPose(aprilTagID); // where id is the AprilTag ID number
    if (!tagPose.isEmpty()) {
        distance = currentBotPose.getTranslation()
            .getDistance(tagPose.get().getTranslation().toTranslation2d()); // meters
    }
    return Distance.ofBaseUnits(distance, Units.meters);
}
// Method 2 where we specify which tag ID to use
public Distance getDistanceToTag(int aprilTagID) {
    // really large distance so if a tag isn't visible we get an unrealistic distance value
    double distance = 999;
    Pose2D currentBotPose = getCurrentBotPose();
    // get the pose of the tag
    Optional<Pose3d> tagPose = aprilTags.getTagPose(aprilTagID); // where id is the AprilTag ID number
    if (!tagPose.isEmpty()) {
        distance = currentBotPose.getTranslation()
            .getDistance(tagPose.get().getTranslation().toTranslation2d()); // meters
    }
    return Distance.ofBaseUnits(distance, Units.meters);
}

// Method 3: we can describe where the hubs are relative to the field and then
// figure our distances to the correct hub.
//
// see https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
// page 5 for field dimensions (those are in inches so convert to meters) to get the position
// relative to the blue alliance side of the field, which is what we want to use
public static final Translation2d kBlueHub = new Translation2d(0.03, 5.547869); // meters
public static final Translation2d kRedHub = new Translation2d(0.03, 5.547869); // meters

// then use this function to get the distance to the hub
public Distance getDistanceToHub(Alliance alliance) {
    double distance = 0;
    Pose2D currentBotPose = getCurrentBotPose();
    if (alliance == 'blue') {
        distance = currentBotPose.getTranslation().getDistance(kBlueHub);
    } else {
        distance = currentBotPose.getTranslation().getDistance(kRedHub);
    }
    return distance
}
```

## Interpolating distance using an InterpolatingDoubleTreeMap

An InterpolatingDoubleTreeMap is a data structure that will help us interpolate between values. This will be useful for setting the shooter RPMs for a calcualted distance. We'll figure out the right RPM to use at a few set distances. Then the InterpolatingDoubleTreeMap will let us "look up" a value between the known ranges. For example, if we know the RPMs to use at 5, 10, and 15 meters away. We can use the InterpolatingDoubleTreeMap to figure out the RPMs to use at 7 or 12 meters.

```java
private static final InterpolatingDoubleTreeMap kRegression = new InterpolatingDoubleTreeMap();

    static {
        // first value is distance to target in meters
        // second parameter is the corresponding shooter RPMs

        // closest we can shoot
        kRegression.put(1.5, 2000);
        // we'll want a few known values to give the interpolator
        // something to work with
        kRegression.put(10.0, 3000);
        // farthest we can shoot
        kRegression.put(15.0, 4000);

    }
```

Then, to use it, we do this:

```java
kRegression.get(getDistanceToTheHub());
```
