```java

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
    private enum turretStates {
        IDLE, TRACKING_TARGET, FINDING_TARGET
    }
    private currentState = turretStates.IDLE;

  public TurretSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch(currentState) {
        case turretStates.IDLE:
            stopMotor();
            break;
        case turretStates.TRACKING_TARGET:
            trackTarget()
            break;
        case turretStates.FINDING_TARGET:
            findTarget();
            break;
        default:
            stopMotor();
    }
  }

    private void trackTarget() {
        // uses PID to drive tx to 0
    }

    private void findTarget() {
        // slowly drive back and forth looking for a target
    }


    public double getTurretAngle() {
        // get the current angle the turret is turned to, would be
        // based on the encoder's absolute position with zero being
        // straight ahead
    }
    public double getTargetAngle() {
        // get the angle to the target relative to where the turret
        // is currently pointing. This is how far we'll have to turn
        // We'd get this from the Limelight's tx value
    }

    public boolean isTargetVisible() {
        return LimelightHelpers.tv;
    }

    public void stopMotor() {}
}

```
