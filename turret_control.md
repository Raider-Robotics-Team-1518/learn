A TurretSubsystem class, taken from https://youtu.be/2G25fvJnC00

```java
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.Units;
import frc.robot.LimelightHelpers;

import frc.robot.Constants;

public class TurretControl extends SubsystemBase {

    private final SparkMax turretMotor;
    private final RelativeEncoder turretEncoder;
    SparkClosedLoopController m_controller;

    // This enum should be moved to its own class and imported
    // and used here so that it's easily available in the
    // RobotContainer.java file

    private Constants.turretStates currentState;
    private Angle tx, ty;
    private Boolean tv;
    private Boolean searchDirectionRight;

    public TurretControl() {
        currentState = Constants.turretStates.DEFAULT;
        turretMotor = new SparkMax(Constants.Motors.turretMotorID, MotorType.kBrushless);
        turretEncoder = turretMotor.getAlternateEncoder();
        m_controller = turretMotor.getClosedLoopController();

        // Reset encoder position to zero
        turretEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        super.periodic();

        /*
         * Can set via code which AprilTags to look for using
         * LimelightHelpers.SetFiducialIDFiltersOverride("limelight", [1, 2, 3]);
         * We could switch based on the Alliance we're on
         */

        tx = Units.Degrees.of(LimelightHelpers.getTX("limelight"));
        ty = Units.Degrees.of(LimelightHelpers.getTY("limelight"));
        tv = LimelightHelpers.getTV("limelight");

        // This method will be called once per scheduler run
        // DEFAULT, HOME, TRACKING, SEARCHING, MANUAL
        switch (currentState) {
            case HOME:
                goHome();
                break;
            case TRACKING:
                trackTarget();
                if (!isTargetVisible()) {
                    currentState = Constants.turretStates.SEARCHING;
                }
                break;
            case SEARCHING:
                findTarget();
                if (isTargetVisible()) {
                    currentState = Constants.turretStates.TRACKING;
                }
                break;
            case MANUAL:
            // Do nothing here, we'll be calling setVelocity from RobotContainer
            // when in MANUAl mode
                break;
            default:
                stopMotor();
        }
    }

    public Command setState(Constants.turretStates state) {
        // the `this` is supply the requirements, in place of doing
        // and addRequirements() in a normal command
        return new InstantCommand(() -> currentState = state, this);
    }

    private Distance getDistance() {
        // TARGET_HEIGHT needs to be defined as a Distance (in Inch)
        // Distance TARGET_HEIGHT = Distance.ofBaseUnits(36, Inch)
        Distance opposite = Constants.Dimensions.targetHeight.minus(Constants.Dimensions.limelightHeight);
        // ty is from the limelight helper getTy()
        // import edu.wpi.first.units.measure.Angle;
        // import edu.wpi.first.units.Units.Degree;
        // Angle LIMELIGHT_MOUNTING_ANGLE = Degree.of(15);
        Angle theta = ty.plus(Constants.Dimensions.limelightMountingAngle);
        // import edu.wpi.first.units.Units.Radians;
        double distance = opposite.in(Units.Meter) / Math.tan(theta.in(Units.Radians));
        return Distance.ofBaseUnits(distance, Units.Meter);
    }

    public Angle getTargetAngle() {
        // get the angle to the target relative to where the turret
        // is currently pointing. This is how far we'll have to turn
        // We'd get this from the Limelight's tx value
        Distance distance = getDistance();
        Distance oppposite = distance.times(Math.tan(tx.in(Units.Radians))).plus(Constants.Dimensions.limelightXOffset);
        Distance adjacent = distance.plus(Constants.Dimensions.limelightYOffset);
        return Units.Radians.of(Math.atan(oppposite.in(Units.Meter) / adjacent.in(Units.Meter)));
    }

    public Angle getTurretFacingAngle() {
        // get the current angle the turret is turned to, would be
        // based on the encoder's absolute position with zero being
        // straight ahead
        return Angle.ofBaseUnits(turretEncoder.getPosition(), Rotations);
    }

    public boolean isTargetVisible() {
        return tv;
    }

    private void trackTarget() {
        // uses PID to drive tx to 0
        Angle currentAngle = getTurretFacingAngle();
        Angle offsetAngle = getTargetAngle();
        Angle targetAngle = currentAngle.plus(offsetAngle);
        Angle clampedAngle = Units.Rotations.of(Math.max(-0.48, Math.min(0.48, targetAngle.in(Rotations))));
        // Set the setpoint of the PID controller in raw position mode
        m_controller.setSetpoint(clampedAngle.baseUnitMagnitude(), ControlType.kPosition);
    }

    private void findTarget() {
        double currentRotation = getTurretFacingAngle().in(Units.Rotations);
        double searchSpeed = 0.26; // radians/sec
        if (currentRotation >= 0.47) {
            searchDirectionRight = false;
        } else if (currentRotation <= 0.49) {
            searchDirectionRight = true;
        }

        double finalVelocity = searchDirectionRight ? searchSpeed : -searchSpeed;
        m_controller.setSetpoint(finalVelocity, ControlType.kVelocity);
                                                                                                         // deg/sec
    }

    public void goHome() {
        m_controller.setSetpoint(0, ControlType.kPosition);
    }

    public void stopMotor() {
        turretMotor.stopMotor();
        // turretMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    /*
    * Manually drives the turret at the speed specified
    * @param speed Trigger input in range from +- 0 to 1
     */
    public void driveTurret (double speed) {
        int motorSpeed = (int) (11710 * 0.75 * speed);
        m_controller.setSetpoint (motorSpeed, ControlType.kVelocity);
    }
}

```

Then RobotContainer.java

```java
public class RobotContainer()
{
    final TurretSubsystem turretSubsystem;
    final CommandXboxController controller;

    public RobotContainer() {
        controller = new CommandXboxController(0);
        turretSubsystem = new TurretSubsystem();
    }

    public void configureBindings() {
        controller.x().onTrue(turretSubsystem.setState(turretSubsystem.turretStates.TRACKING));

        codriver.leftTrigger(0.1).whileTrue(
                Commands.startEnd(() -> {
                    turretSubsystem.setState(Constants.turretStates.MANUAL);
                    turretSubsystem.driveTurret (-codriver.getLeftTriggerAxis());
                },
                () -> turretSubsystem.driveTurret(0)));

        codriver.rightTrigger(0.1).whileTrue(
                Commands.startEnd(() -> {
                    turretSubsystem.setState(Constants.turretStates.MANUAL);
                    turretSubsystem.driveTurret(codriver.getRightTriggerAxis() * 60);
                },
                () -> turretSubsystem.driveTurret(0)));


}
}

```
