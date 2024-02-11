package frc.robot.utils;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/***
 * @author Noah Simon
 * @author Raadwan Masum
 * @author Rohin Sood
 *         Default command to control the SwervedriveSubsystem with joysticks
 */

public class PIDTuner {
  static boolean shouldRun = false;

  public PIDTuner() {
  }

  public static double update(double currentVal, double interval, Command command, double error,
      double okErr) {
    boolean inc = false, dec = false;

    if (okErr < 0) {
      inc = true;
    } else if (error > okErr) {
      dec = true;
    } else {
      return currentVal;
    }

    double newVal = currentVal;
    if (dec) {
      newVal = currentVal - interval;
    } else if (inc) {
      newVal = currentVal + interval;
    }

    SmartDashboard.putNumber("PIDTuner-newVal", newVal);

    shouldRun = !shouldRun;
    Trigger newTrig = new Trigger(() -> shouldRun);

    newTrig.onTrue(command);
    command.andThen(() -> {
      Robot.swerve.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    });

    // Robot.swerve.resetOdometry(new Pose2d(new Translation2d(0, 0), new
    // Rotation2d(0)));

    return newVal;
  }
}