# PID Control

This will be light on theory and instead focus on implementation. For background and theory of PID control, see the [FRC PID control overview](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html). Another [good document about PID comes from Team 2928](https://2928-frc-programmer-training.readthedocs.io/en/latest/Romi/Control/romiPID/) (it's about the Romi but most of it applies to the RoboRio as well). Finally, RevRobotics has [useful docs](https://docs.revrobotics.com/revlib/spark/closed-loop) focusing on using their motors/controllers for PID control.

## Definitions

You will need a little background including understanding some PID terminology.

- P &mdash; Proportional factor, roughly speaking how quickly the position error is driven to zero -- how fast it approaches the target setpoint.
- I &mdash; Integral factor, generally, for FRC leave this set to zero. Roughly speaking, how quickly the velocity error is driven to zero.
- D &mdash; Derivative factor, roughly speaking this is how quickly the accumulated error over time is driven to zero. This factor helps reduce oscillations around the setpoint. A well-tuned P value is better than adding a D factor if you can.
- Setpoint &mdash; A PID controller's setpoint is the desired target value for a process variable that the controller aims to maintain, such as a given encoder value like total rotations.
- Process value &mdash; The current value of the value being controlled, such as the motor's current number of rotations.
- Error &mdash; How far off from the setpoint is the current process value
- Open loop control &mdash; A form of control where there is no feedback from the system on the current state of the system. Driving a motor for 3 seconds without checking for encoder values is an example of open loop control.
- Closed loop control &mdash; A form of control where the current value of the motor is read and fed back into the calculations. PID is a form of closed loop control.

Basically, you give a PID Controller a target value, such as an angle, speed, or distance traveled, and a way to calculate the current value of that target. The PID Controller then automatically drives the motor or subsystem until it has reached that target value.

The advantage of using a PID Controller to do this it that it will automatically handle overshooting, undershooting, or oscillating around the target value, as well as holding to that target value. You will need to tune the values of P, I, and D to control the accuracy of that function.

## Example 1

Example from TanX from their learning sessions in 2025. This one shows PID control over a motor, driving it to the angle specified by the joystick.

```java
package frc.robot;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This sample program shows how to control a motor using a joystick. In the operator control part
 * of the program, the joystick is read and the value is written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and motor controller inputs also range from -1 to 1
 * making it easy to work together.
 *
 * <p>In addition, the encoder value of an encoder connected to ports 0 and 1 is consistently sent
 * to the Dashboard.
 */
public class Robot extends TimedRobot {
  private static final double MAX_SPEED = 0.25;
  private static final double MIN_SPEED = -0.25;

  // Specify how far the wheel turns per rotations of the motor. This will be
  // dependent on the gearing installed in the motor assembly. The value shown
  // here was correct for TanX's robot but wouldn't work directly for another bot.
  private static final double WHEEL_ROTATIONS_PER_MOTOR_ROTATION = (1.0 / (150.0 / 7.0));

  private static final int kMotorPort = 15;
  private static final int kJoystickPort = 0;

  private final SparkMax m_motor;
  private final SparkClosedLoopController m_motorController;
  private final SparkFlexConfig m_motorConfig = new SparkFlexConfig();
  private final ClosedLoopConfig m_closedLoopConfig = new ClosedLoopConfig();
  private final XboxController m_joystick;
  private final RelativeEncoder m_encoder;

  // starting values for P, I, and D. These will be tunable from the dashboard.
  private double p = 1.0;
  private double i = 0.0;
  private double d = 0.0;

  private final PIDController m_pidController = new PIDController(p, i, d);

  /** Called once at the beginning of the robot program. */
  public Robot() {
    m_motor = new SparkMax(kMotorPort, MotorType.kBrushless);
    m_motorController = m_motor.getClosedLoopController();
    m_joystick = new XboxController(kJoystickPort);
    m_encoder = m_motor.getEncoder();

    m_pidController.setTolerance(0.05);
    // configure our closed-loop controller with the starting PID values
    m_closedLoopConfig.pid(p, i, d);
    m_motorConfig.apply(m_closedLoopConfig);
    // configure the motor with the motor config object and some motor-specific base parameters
    m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putNumber("P", p);
    SmartDashboard.putNumber("I", i);
    SmartDashboard.putNumber("D", d);
  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {
    // the code here in robotPeriodic is all about outputting and getting current values
    // for various parameters, including the P, I, and D values, which can be set from
    // the dashboard
    SmartDashboard.putNumber("Encoder", m_encoder.getPosition());
    SmartDashboard.putNumber("Joystick", m_joystick.getLeftY());
    SmartDashboard.putNumber("Motor", m_motor.getAppliedOutput());
    SmartDashboard.putNumber("Stick Angle (Radians)", calculateStickAngle());
    SmartDashboard.putNumber("Motor Setpoint", m_pidController.getSetpoint());
    SmartDashboard.putNumber("Wheel Angle (Radians)",Rotation2d.fromRotations(m_encoder.getPosition() * WHEEL_ROTATIONS_PER_MOTOR_ROTATION).getRadians());

    // Get PID values from SmartDashboard
    double np = SmartDashboard.getNumber("P", 0.0);
    double ni = SmartDashboard.getNumber("I", 0.0);
    double nd = SmartDashboard.getNumber("D", 0.0);

    // Update PID controller values if new values have been entered in the dashboard
    boolean setConfig = false;
    if (p != np) {
      p = np;
      m_pidController.setP(p);
      m_closedLoopConfig.p(p);
      setConfig = true;
    }
    if (i != ni) {
      i = ni;
      m_pidController.setI(i);
      m_closedLoopConfig.i(i);
      setConfig = true;
    }
    if (d != nd) {
      d = nd;
      m_pidController.setD(d);
      m_closedLoopConfig.d(d);
      setConfig = true;
    }

    if (setConfig) {
      m_motorConfig.apply(m_closedLoopConfig);
      m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  }

  /** The teleop periodic function is called every control packet in teleop. */
  @Override
  public void teleopPeriodic() {
    // Get the joystick angle and convert it to a setpoint for the motor
    Rotation2d rotation = Rotation2d.fromDegrees(calculateStickAngle());

    double motorSetpoint = rotation.getRotations() / WHEEL_ROTATIONS_PER_MOTOR_ROTATION;

    // Set the motor setpoint based on the joystick angle
    m_pidController.setSetpoint(motorSetpoint, ControlType.kPosition);

    // Calculate the PID output and clamp it to the motor speed limits
    double output = m_pidController.calculate(m_encoder.getPosition());
    // `clamp()` is a function that forces a value to between the min
    // and max values specified. Here, output will never be less than
    // MIN_SPEED or greater than MAX_SPEED
    output = MathUtil.clamp(output, MIN_SPEED, MAX_SPEED);

    // Set the motor speed if we haven't hit the target setpoint
    // This continues to drive the motor till we hit our target
    if (!m_pidController.atSetpoint())  {
      m_motor.set(output);
    }
}

  private double calculateStickAngle() {
    double x = m_joystick.getLeftX();
    double y = m_joystick.getLeftY();

    double angle = Math.atan2(y, x);
    //angle += Math.PI / 2; // Adjust for the joystick orientation

    return angle;
  }
}
```

## Example 2

This example is taken [from the FRC docs](https://docs.wpilib.org/en/stable/docs/software/commandbased/pid-subsystems-commands.html). It demonstrates PID control of a subsystem, in this case a shooter with two motors. This returns a Command that you'd attach to a button (when pressed, run this command).

```java
package edu.wpi.first.wpilibj.examples.rapidreactcommandbot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.examples.rapidreactcommandbot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final PWMSparkMax m_shooterMotor = new PWMSparkMax(ShooterConstants.kShooterMotorPort);
  private final PWMSparkMax m_feederMotor = new PWMSparkMax(ShooterConstants.kFeederMotorPort);
  private final Encoder m_shooterEncoder =
      new Encoder(
          ShooterConstants.kEncoderPorts[0],
          ShooterConstants.kEncoderPorts[1],
          ShooterConstants.kEncoderReversed);
  private final SimpleMotorFeedforward m_shooterFeedforward =
      new SimpleMotorFeedforward(
          ShooterConstants.kSVolts, ShooterConstants.kVVoltSecondsPerRotation);
  private final PIDController m_shooterFeedback = new PIDController(ShooterConstants.kP, 0.0, 0.0);

  /** The shooter subsystem for the robot. */
  public Shooter() {
    m_shooterFeedback.setTolerance(ShooterConstants.kShooterToleranceRPS);
    m_shooterEncoder.setDistancePerPulse(ShooterConstants.kEncoderDistancePerPulse);

    // Set default command to turn off both the shooter and feeder motors, and then idle
    setDefaultCommand(
        runOnce(
                () -> {
                  m_shooterMotor.disable();
                  m_feederMotor.disable();
                })
            .andThen(run(() -> {}))
            .withName("Idle"));
  }

  /**
   * Returns a command to shoot the balls currently stored in the robot. Spins the shooter flywheel
   * up to the specified setpoint, and then runs the feeder motor.
   *
   * @param setpointRotationsPerSecond The desired shooter velocity
   */
  public Command shootCommand(double setpointRotationsPerSecond) {
    return parallel(
            // Run the shooter flywheel at the desired setpoint using feedforward and feedback
            run(
                () -> {
                  m_shooterMotor.set(
                      m_shooterFeedforward.calculate(setpointRotationsPerSecond)
                          + m_shooterFeedback.calculate(
                              m_shooterEncoder.getRate(), setpointRotationsPerSecond));
                }),

            // Wait until the shooter has reached the setpoint, and then run the feeder
            waitUntil(m_shooterFeedback::atSetpoint).andThen(() -> m_feederMotor.set(1)))
        .withName("Shoot");
  }
}
```

## Example 3

See our [2025 ElevatorSubsystem.java subsystem](https://github.com/Raider-Robotics-Team-1518/ReefScape_grr/blob/main/src/main/java/org/team1518/robot/subsystems/ElevatorSubsystem.java) for a simple application of PID control of an elevator. We didn't include any handling of gravity, which we should have. The elevator would slowly creep down until it got well below the set point then would bump itself back up. With a gravity factor in there (see below) the motor controller would have applied a small voltage to drive the motor enough to keep the elevator from dropping.

## Tuning

- [Tuning PID](https://docs.wpilib.org/en/stable/docs/software/dashboards/shuffleboard/advanced-usage/shuffleboard-tuning-pid.html) from FRC docs
- [Rev Robotics tuning guide](https://docs.revrobotics.com/revlib/spark/closed-loop/getting-started-with-pid-tuning)

Rev Robotics [has a doc on extra "feedforward" tuning parameters](https://docs.revrobotics.com/revlib/spark/closed-loop/feed-forward-control) that their controllers support (e.g. SPARK Max). These help overcome things like friction in a motor, the force of gravity on an elevator, etc. For example, the kG feedforward parameter will set the gravity gain to help hold an elevator in position after it has reached its setpoint. This is important info to tune and take into consideration in the code.

CTR has similar information on [their TalonFX page](https://pro.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/closed-loop-requests.html).
