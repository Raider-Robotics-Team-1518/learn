# DRAFT Tuning Swerve PID

PID constants for drive and turn are in the generated/TunerConstants.java file right at the beginning of the TunerConstants class defintion. There are pre-defined values (from CTRE) there now. Record what they are at the beginning in case you want to put them back to that state.

## Tuning the drive and angular speed

In RobotContainer.java, there are two variables you can adjust for max drive and angular speed. These are `MaxSpeed` and `maxAngularRate`. Both are defined near the top of the file. Change the multiplier there to decrease the speeds.

You should also set max acceleration in Choreo to adjust speed in autonomous mode.

## Drive PID tuning

Run through this first with the bot on blocks. Then do it again on the ground.

1. In Choreo, create an auto routine that drives the robot forward a specific distance, for example 6 feet.
1. Build and push code
1. Run the auto, the goal is that bot drives exactly the distance you specified and then stops. Most likely the first time it won't do that. If it undershoots, increase P. If it overshoots, decrease P.
1. Build, push, and repeat until the robot drives acceptably close to the goal.
1. If it oscillates around the set point, increase D. Repeat your tests.

## Turn PID tuning

Do this with the bot on blocks first, then repeat on the ground.

1. In Choreo, create an auto routine that turns the robot to a specified angle, for example 45 degrees.
1. Build and push code.
1. Run the auto. The goal is that the wheels turn to the angle you specified and stop. If they overshoot, decrease P. If it undershoots, increase P.
1. Repeat till it reliably hits the target angle.
1. If the wheels oscillate around the set point, increase D. Repeat your tests.

## References

These are good to read over!

- https://www.frc5712.com/swerve-calibration
- https://www.frc5712.com/pid-control
- https://sites.google.com/view/team930programmingdoc/control-systems/swerve-drive
- https://docs.yagsl.com/configuring-yagsl/how-to-tune-pidf
- https://pro.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/closed-loop-requests.html
