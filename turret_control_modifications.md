What I think we need to do to go back to using the TurretControl class, which had nice auto-tracking features that the TurretSubsystem does not have.

1. Move the TurretControl.java class from the Documents folder back into the project into the java/frc/robot/subsystems folder.

2. Add a new MANUAL state to the enum in Constants.java

```java
public enum turretStates {
        DEFAULT, HOME, TRACKING, SEARCHING, MANUAL
    }
```

3. In the TurretControl class, in the periodic() function, modify the switch/case statement to add handling for the MANUAL state

```java
case MANUAL:
    // do nothing here, we'll be calling setVelocity from RobotContainer
    // when in MANUAL mode
    break;

```

4. Add these two functions in the TurretControl class

```java
/*
* Manually drives the turret at the speed specified
* @param speed Motor rotations per second
*/
private void setVelocityInRotations(speed) {
    // we might need to account for the 90:1 gearbox here
    turretMotor.setControl(velocityControl.withVelocity(speed));
}

/*
* Manually drives the turret at the speed specified
* @param speed Motor degrees per second
*/
private void setVelocityInDegrees(speed) {
    // we might need to account for the 90:1 gearbox here
    turretMotor.setControl(velocityControl.withVelocity(Units.DegreesPerSecond.of(speed)));
}
```

5. Update RobotContainer to import the TurretControl class, declare it, and instantiate it.

```java
// in the imports section
import frc.robot.subsystems.TurretControl

// in the class definition section
public static TurretControl turretSubsystem;

// then in the constructor, change how turretSubsystem is instantiated
public RobotContainer() {
    ...
    turretSubsystem = new TurretControl();

```

6. Modify RobotContainer.java to use these two blocks for the codriver trigger control rather than what's there now. (Comment out the existing lines in case this doesn't work.) We'll probably have to play with the `* 60` multiplier to dial in the speed.

```java
codriver.leftTrigger(0.1).whileTrue(
        Commands.startEnd(Commands.sequence(
                () -> turretSystem.setState(Constants.turretStates.MANUAL),
                () -> turretSystem.setVelocityInDegrees(-codriver.getLeftTriggerAxis() * 60)),
                () -> turretSystem.setVelocityInDegrees(0)));

codriver.rightTrigger(0.1).whileTrue(
        Commands.startEnd(Commands.sequence(
                () -> turretSystem.setState(Constants.turretStates.MANUAL),
                () -> turretSystem.setVelocityInDegrees(codriver.getRightTriggerAxis() * 60)),
                () -> turretSystem.setVelocityInDegrees(0)));

```
