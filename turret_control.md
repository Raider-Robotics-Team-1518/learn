A TurretSubsystem class, taken from https://youtu.be/2G25fvJnC00

```java
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Units.*;

import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

    private final TalonFX turretMotor;
    private final CANcoder turretEncoder;

    // This enum should be moved to its own class and imported
    // and used here so that it's easily available in the
    // RobotContainer.java file
    public enum turretStates {
        DEFAULT, HOME, TRACKING, SEARCHING
    }

    private turretStates currentState;
    private Angle tx, ty;
    private Boolean tv;
    private Boolean searchDirectionRight;

    public TurretSubsystem() {
        currentState = turretStates.DEFAULT;
        turretMotor = new TalonFX(Constants.turretMotorID);
        turretEncoder = new CANcoder(Constants.turretEncoderID);
        FeedbackConfigs motor_feedback_config = new FeedbackConfigs();
        motor_feedback_config.SensorToMechanismRatio = 1 / 1.2; // should define that in constants
        motor_feedback_config.RotorToSensorRatio = 60; // should define that in constants
        motor_feedback_config.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        motor_feedback_config.FeedbackRemoteSensorID = turretEncoder.getDeviceID();
        SoftwareLimitSwitchConfigs motor_software_limit_switch_config = new SoftwareLimitSwitchConfigs();
        // Soft limits protects cables from being pulled
        motor_software_limit_switch_config.ForwardSoftLimitEnable = true;
        motor_software_limit_switch_config.ReverseSoftLimitEnable = true;
        // limits to approx +/- 175 degrees, these should be constants
        motor_software_limit_switch_config.ForwardSoftLimitThreshold = 0.48;
        motor_software_limit_switch_config.ReverseSoftLimitThreshold = -0.48;

        TalonFXConfiguration motor_config = new TalonFXConfiguration();
        motor_config.SoftwareLimitSwitch = motor_software_limit_switch_config;
        motor_config.withFeedback(motor_feedback_config);
        // configure our PID values, these should be Tunables or at least constants
        motor_config.Slot0.kP = 2.4;
        motor_config.Slot0.kI = 0;
        motor_config.Slot0.kD = 0.1;

        turretMotor.getConfigurator().apply(motor_config);

    }

    @Override
    public void periodic() {
        super.periodic();

        // need to import Degrees...I think it's in edu.wpi.first.units.measure
        tx = Units.Degrees.of(LimelightHelpers.getTX("limelight"));
        ty = Units.Degrees.of(LimelightHelpers.getTY("limelight"));
        tv = LimelightHelpers.getTV("limelight");

        // This method will be called once per scheduler run
        // DEFAULT, HOME, TRACKING, SEARCHING
        switch (currentState) {
            case HOME:
                goHome();
                break;
            case TRACKING:
                trackTarget();
                if (!isTargetVisible()) {
                    currentState = turretStates.SEARCHING;
                }
                break;
            case SEARCHING:
                findTarget();
                if (isTargetVisible()) {
                    currentState = turretStates.TRACKING;
                }
                break;
            default:
                stopMotor();
        }
    }

    public Command setState(turretStates state) {
        // the `this` is supply the requirements, in place of doing
        // and addRequirements() in a normal command
        return new InstantCommand(() -> currentState = state, this);
    }

    private Distance getDistance() {
        // TARGET_HEIGHT needs to be defined as a Distance (in Inch)
        // Distance TARGET_HEIGHT = Distance.ofBaseUnits(36, Inch)
        Distance opposite = Constants.TARGET_HEIGHT.minus(Constants.LIMELIGHT_HEIGHT);
        // ty is from the limelight helper getTy()
        // import edu.wpi.first.units.measure.Angle;
        // import edu.wpi.first.units.Units.Degree;
        // Angle LIMELIGHT_MOUNTING_ANGLE = Degree.of(15);
        Angle theta = ty.plus(Constants.LIMELIGHT_MOUNTING_ANGLE);
        // import edu.wpi.first.units.Units.Radians;
        double distance = opposite.in(Units.Meter) / Math.tan(theta.in(Units.Radians));
        return Distance.ofBaseUnits(distance, Units.Meter);
    }

    public Angle getTargetAngle() {
        // get the angle to the target relative to where the turret
        // is currently pointing. This is how far we'll have to turn
        // We'd get this from the Limelight's tx value
        Distance distance = getDistance();
        Distance oppposite = distance.times(Math.tan(tx.in(Units.Radians))).plus(Constants.CAMERA_OFFSET_X);
        Distance adjacent = distance.plus(Constants.CAMERA_OFFSET_Y);
        return Units.Radians.of(Math.atan(oppposite.in(Units.Meter) / adjacent.in(Units.Meter)));
    }

    public Angle getTurretFacingAngle() {
        // get the current angle the turret is turned to, would be
        // based on the encoder's absolute position with zero being
        // straight ahead
        return turretEncoder.getAbsolutePosition().getValue();
    }

    public boolean isTargetVisible() {
        return tv;
    }

    private void trackTarget() {
        // uses PID to drive tx to 0
        Angle currentAngle = getTurretFacingAngle();
        Angle offsetAngle = getTargetAngle();
        Angle targetAngle = currentAngle.plus(offsetAngle);
        Angle clampedAngle = Units.Rotations.of(Math.max(-0.48, Math.min(0.48, targetAngle.inRotations())));
        turretMotor.setControl(new PositionVoltage(0).withPosition(clampedAngle));
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
        turretMotor.setControl(new VelocityVoltage(0).withVelocity(Units.RadiansPerSecond.of(finalVelocity))); // about 45
                                                                                                         // deg/sec
    }

    public void goHome() {
        turretMotor.setControl(new PositionVoltage(0).withPosition(0));
    }

    public void stopMotor() {
        turretMotor.stopMotor();
        turretMotor.setNeutralMode(NeutralModeValue.Brake);
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
    }
}

```
