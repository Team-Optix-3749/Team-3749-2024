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

public class SwerveTeleopCommand extends Command {

  private final Swerve swerve;
  private final Supplier<Double> xSpdFunction, ySpdFunction, xTurningSpdFunction, yTurningSpdFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  private final PIDController pid_turnController = new PIDController(PIDValues.kP_teleopTurn, 0,
      PIDValues.kD_teleopTurn);

  public SwerveTeleopCommand(
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
    this.turningLimiter = new SlewRateLimiter(DriveConstants.maxAngularAccelerationMetersPerSecondSquared);
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
    double turningSpeed = 0;

    double xTurnPos = xTurningSpdFunction.get();
    double yTurnPos = yTurningSpdFunction.get();

    xSpeed = Math.abs(xSpeed) > ControllerConstants.deadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > ControllerConstants.deadband ? ySpeed : 0.0;
    xTurnPos = Math.abs(xTurnPos) > ControllerConstants.deadband ? xTurnPos : 0.0;
    yTurnPos = Math.abs(yTurnPos) > ControllerConstants.deadband ? yTurnPos : 0.0;

    xSpeed = xLimiter.calculate(xSpeed * DriveConstants.maxSpeedMetersPerSecond);
    ySpeed = yLimiter.calculate(ySpeed * DriveConstants.maxSpeedMetersPerSecond);

    // do some cool magical trig to find theta and convert it to unsigned rad
    double currentRotationRad = swerve.getRotation2d().getRadians();
    double desiredRotationRad = Math.atan2(xTurnPos, yTurnPos);
    desiredRotationRad = (desiredRotationRad + (4 * Math.PI)) % (2 * Math.PI);

    turningSpeed = turningLimiter
        .calculate(pid_turnController.calculate(currentRotationRad, desiredRotationRad));

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed, ySpeed, turningSpeed, swerve.getRotation2d());

    // if (Robot.isSimulation()) {
    // PIDValues.kP_teleopTurn = PIDTuner.update(PIDValues.kP_teleopTurn, 0.005,
    // new TurnToAngle(new Rotation2d(Math.PI)));
    // } else {
    // throw new Error("TeleopCommand -- DID NOT REMOVE PID TUNER");
    // }

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