package frc.robot.commands.swerve;

import java.util.function.Supplier;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.Constants;
import frc.robot.utils.JoystickIO;
import frc.robot.utils.PIDTuner;
import frc.robot.utils.Xbox;
import frc.robot.utils.Constants.ControllerConstants;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.Sim.PIDValues;;

/***
 * @author Noah Simon
 * @author Raadwan Masum
 * @author Rohin Sood
 *         Default command to control the SwervedriveSubsystem with joysticks
 */

public class Teleop extends Command {
  private final Swerve swerve;
  private final Supplier<Double> xSpdFunction, ySpdFunction, xTurningSpdFunction, yTurningSpdFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  private final PIDController pid_turnController = new PIDController(PIDValues.kP_teleopTurn, 0,
      PIDValues.kD_teleopTurn);

  public Teleop(
      Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> xTurningSpdFunction,
      Supplier<Double> yTurningSpdFunction) {
    this.swerve = Robot.swerve;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.xTurningSpdFunction = xTurningSpdFunction;
    this.yTurningSpdFunction = yTurningSpdFunction;

    // This should be max Acceleration! I think.
    this.xLimiter = new SlewRateLimiter(DriveConstants.maxAccelerationMetersPerSecondSquared);
    this.yLimiter = new SlewRateLimiter(DriveConstants.maxAccelerationMetersPerSecondSquared);
    this.turningLimiter = new SlewRateLimiter(DriveConstants.maxAngularAccelerationRadiansPerSecondSquared);
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    pid_turnController.enableContinuousInput(0, 2 * Math.PI);
  }

  @Override
  public void execute() {
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = xTurningSpdFunction.get();

    xSpeed = Math.abs(xSpeed) > ControllerConstants.deadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > ControllerConstants.deadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > ControllerConstants.deadband ? turningSpeed : 0.0;

    xSpeed = xLimiter.calculate(xSpeed * DriveConstants.maxSpeedMetersPerSecond);
    ySpeed = yLimiter.calculate(ySpeed * DriveConstants.maxSpeedMetersPerSecond);

    turningSpeed = turningLimiter.calculate(turningSpeed * DriveConstants.maxAngularSpeedRadiansPerSecond);

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed, ySpeed, turningSpeed, swerve.getRotation2d());

    // set chassis speeds
    swerve.setChassisSpeeds(chassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}