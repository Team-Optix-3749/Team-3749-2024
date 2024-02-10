package frc.robot.utils;

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
  private static CommandPS5Controller controller = new CommandPS5Controller(4);
  
  public PIDTuner() {
  }

  public static double update(double currentVal, double interval, Command command) {
    boolean dec = controller.getHID().getL1Button();
    boolean inc = controller.getHID().getR1Button();
    
    double newVal = currentVal;
    if (dec) {
      newVal = currentVal - interval;
    } else if (inc) {
      newVal = currentVal + interval;
    }

    SmartDashboard.putNumber("PIDTuner-newVal", newVal);

    Trigger runCMD = controller.triangle();
    runCMD.onTrue(command);

    Robot.swerve.resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));

    return newVal;
  }
}