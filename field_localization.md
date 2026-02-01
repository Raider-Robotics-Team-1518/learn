# Field localization

## Field coordinates

When on the Blue alliance, the 0,0 coordinate (the origin) of the field is, looking at the field from the blue side, the right side, closest corner. Positive X is away from the blue side. Positive Y is to the left. There are two ways of handling being on the Red alliance. See [FRC's field coordinates doc](https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#dealing-with-red-or-blue-alliance).

The Limelight code below assumes the "always blue" technique, where 0,0 is always on the blue side of the field. We need to confirm that the CTR-generated code uses the same technique.

## Pose2D and Pose3D objects

A pose is an object representing the coordinates of something in field-relative space. Pose2D includes the x, y coordinates as well as a rotation. These are relative to the field origin (see above). A Pose3Dd also includes the Z-axis. Instances of both objects have various methods and properties that are useful to us. For example, they both have a `getTranslation()` method and the Translation2d/Translation3d object that is returned has a `getDistance()` method that we can use to calculate the distance in meters between two poses.

## Updating odometry

The CTRE-generated code includes the functionality to automatically update the bot's odometry. We can & should use their `addVisionMeasurment()` function to refine that odometry using pose information from the Limelight. We would do that in the `robotPeriodic()` function so that it runs in both auto and teleop modes.

```java

@Override
public void robotPeriodic() {
  // uses the getPose() function we added to Autos.java in 2026Rebuilt
  // needs a reference to the gyro, m_gyro
  boolean doRejectUpdate = false;
  LimelightHelpers.SetRobotOrientation("limelight", getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
  LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
  if(Math.abs(m_gyro.getRate()) > 720) {
    // if our angular velocity is greater than 720 degrees per second, ignore vision updates
    doRejectUpdate = true;
  }
  if(mt2.tagCount == 0) {
    doRejectUpdate = true;
  }
  if(!doRejectUpdate) {
    m_poseEstimator.addVisionMeasurement(
        mt2.pose,
        mt2.timestampSeconds);
    }
}
```

See [the Limelight megatag example](https://github.com/LimelightVision/limelight-examples/blob/main/java-wpilib/swerve-megatag-odometry/src/main/java/frc/robot/Drivetrain.java#L87) and the Limelight docs for more on MegaTag2.

## Getting the robot's location

When we need to, we can grab the robot's pose to determine its location on the field (and orientation). We can do that by calling a Limelight library function, or by calling a method of the pose estimator created in CTR's Drivetrain.java file.

```java
// Limelight technique
// This returns a variable of type LimelightHelpers.PoseEstimate which
// has a .pose property that is a 2D pose of the bot
PoseEstimate botPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
Pose3d currentBotPose3d = botPoseEstimate.pose;
Pose2d currentBotPose2d = currentBotPose3d.toPose2d();
// there are other properties of that PoseEstimate, see
// https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib#poseestimate

// in LimelightHelpers.java, there's also this function & others
// I believe the megatag2 method is more accurate
public Pose2d getRobotPose_FieldSpace2D() {
    return toPose2D(robotPose_FieldSpace);
}
currentBotPose = getRobotPose_FieldSpace2D();

// Using CTRE's generated swerve project, we can get the bot's pose using:
var state = drivetrain.getState();
// pull out the pose estimate and chassis speeds
Pose2d currentBotPose = state.Pose;
ChassisSpeeds speeds = state.Speeds;

```

## Calculating distance to an April tag

```java
// From GRR's Crescendo project

aprilTags = AprilTagFields.k2026RebuiltWelded.loadAprilTagLayoutField();
aprilTags.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

Optional<Pose3d> tagPose = aprilTags.getTagPose(id); // where id is the AprilTag ID number
if (!tagPose.isEmpty()) {
    double distance = currentBotPose.getTranslation()
        .getDistance(tagPose.get().getTranslation().toTranslation2d()); // meters I believe
}

// You can also define a spot on the field for an element
// from GRR's Crescendo Constants.java file
public static final Translation2d kBlueSpeaker = new Translation2d(0.03, 5.547869); // meters I believe

// they also defined some constants for the field size (in meters) that they used to calculate
// the position of some field elements
public static final double kLength = 16.541; // need to be updated for current year's field size
public static final double kWidth = 8.211; // same

```

As best as I can tell, there's no way to determine the rotation between two poses. For example, you have a bot pose and an AprilTag pose. You cannot calculate what angle the robot (or a turret) would need to rotate to point at the tag. We'll have to determine that using the Limelight.

---

Additional Notes:

- GRR's Reefscape code
  - lib/math/FieldInfo.java file is a utility class for getting field information, including setting AprilTag locations.
  - lib/math/Math2.java has some utilities, like "near()" that we could possibly use
- GRR's Crescendo code
  - Look at robot/subsytems/swerve.java for uses of getDistance() which is a built-in function of the Translation2d class for figuring out the distance between two poses. So, you get the robot's pose from (in their case) the swerve drive system and then calculate distance to a specific pose, such as the position of a field element or AprilTag. Uses a Translation2d but a pose is typically a Translation3D which will have a method called toTranslation2d() to convert.
  - Also in swerve.java, look at the use of AprilTagFields on ~line 215 and also getTagPose() on line 242. WPILib includes a class with the april tag locations on the field
  - See robot/subsystems/Pivot.java and the use of InterpolatingDoubleTreeMap to "look up" an angle based on distance. You define some known points (this distance means position at that angle). Then this lookup interpolates between the known angles based on the distance. We'd have to manually tune those known points. Doing it this way might be easier than coming up with our own formula or making manual guesses in code.
