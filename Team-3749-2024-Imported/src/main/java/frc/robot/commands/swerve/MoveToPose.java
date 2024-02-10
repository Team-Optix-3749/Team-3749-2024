package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.Sim.PIDValues;
import frc.robot.Robot;

/***
 * @param Pose2d targetPose
 */
public class MoveToPose extends Command {
  private final Swerve swerve = Robot.swerve;
  private Pose2d targetPose = swerve.getPose();

  private final PIDController pid_driveXController = new PIDController(PIDValues.kP_MiscDrive, 0,
      PIDValues.kD_MiscDrive);
  private final PIDController pid_driveYController = new PIDController(PIDValues.kP_MiscDrive, 0,
      PIDValues.kD_MiscDrive);
  private final PIDController pid_turnController = new PIDController(PIDValues.kP_MiscTurn, 0,
      PIDValues.kD_MiscTurn);

  private final SlewRateLimiter slewLimit_x = new SlewRateLimiter(DriveConstants.maxAccelerationMetersPerSecondSquared);
  private final SlewRateLimiter slewLimit_y = new SlewRateLimiter(DriveConstants.maxAccelerationMetersPerSecondSquared);
  private final SlewRateLimiter slewLimit_turn = new SlewRateLimiter(
      DriveConstants.maxAngularAccelerationMetersPerSecondSquared);

  public MoveToPose(Pose2d targetPose) {
    this.targetPose = targetPose;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    pid_turnController.enableContinuousInput(0, 2 * Math.PI);

    // set tolerances to prevent endless command
    pid_turnController.setTolerance(Constants.DriveConstants.toleranceRad_Misc);
    pid_driveXController.setTolerance(Constants.DriveConstants.toleranceM_Misc);
    pid_driveYController.setTolerance(Constants.DriveConstants.toleranceM_Misc);

    // set target pose so command finishes when robot is within tolerances (see
    // isFinished())
    pid_driveXController.setSetpoint(targetPose.getX());
    pid_driveYController.setSetpoint(targetPose.getY());
  }

  @Override
  public void execute() {
    // get current position on field
    Pose2d currentPose = swerve.getPose();

    double driveXPos = currentPose.getX();
    double driveYPos = currentPose.getY();

    // calculate speeds
    double driveXVeloM = pid_driveXController.calculate(driveXPos, targetPose.getX());
    double driveYVeloM = pid_driveYController.calculate(driveYPos, targetPose.getY());
    double turnVelo = pid_turnController.calculate(currentPose.getRotation().getRadians(),
        targetPose.getRotation().getRadians());

    driveXVeloM = slewLimit_y.calculate(driveXVeloM * DriveConstants.maxSpeedMetersPerSecond);
    driveYVeloM = slewLimit_x.calculate(driveYVeloM * DriveConstants.maxSpeedMetersPerSecond);
    turnVelo = slewLimit_turn.calculate(turnVelo * DriveConstants.maxAngularSpeedMetersPerSecond);

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(driveXVeloM,
        driveYVeloM, turnVelo, swerve.getRotation2d());

    swerve.setChassisSpeeds(chassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
    SmartDashboard.putNumber("ended", 0);
  }

  @Override
  public boolean isFinished() {
    return pid_driveXController.atSetpoint() && pid_driveYController.atSetpoint() && pid_turnController.atSetpoint();
  }

  public static Transform2d translationToTransform(double x, double y) {
    return new Transform2d(new Translation2d(x, y), new Rotation2d());
  }
}