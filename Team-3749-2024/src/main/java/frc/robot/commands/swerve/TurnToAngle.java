package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.Sim.PIDValues;

public class TurnToAngle extends Command {
  private final Swerve swerve;

  private Rotation2d desiredRotation = new Rotation2d(0);
  private boolean preserveVelocity = false;

  private final PIDController pid_turnController = new PIDController(PIDValues.kP_TurnToAngle, 0,
      PIDValues.kD_TurnToAngle);
  private final SlewRateLimiter slewLimit_turn = new SlewRateLimiter(
      DriveConstants.maxAngularAccelerationRadiansPerSecondSquared);

  /***
   * @param Rotation2d desiredRotation
   */
  public TurnToAngle(Rotation2d desiredRotation) {
    this.swerve = Robot.swerve;
    this.desiredRotation = desiredRotation;
    addRequirements(swerve);
  }

  /***
   * @param Rotation2d desiredRotation
   * @param boolean    preserveVelocity
   */
  public TurnToAngle(Rotation2d desiredRotation, boolean preserveVelocity) {
    this.swerve = Robot.swerve;
    this.desiredRotation = desiredRotation;
    this.preserveVelocity = preserveVelocity;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    pid_turnController.setSetpoint(desiredRotation.getRadians());
    pid_turnController.setTolerance(Constants.DriveConstants.toleranceRad_Misc);
    pid_turnController.enableContinuousInput(0, 2 * Math.PI);
  }

  @Override
  public void execute() {
    // calculate the turning speed and clamp the output to the maximum speed that
    // the motor can turn
    double turnVelo = pid_turnController.calculate(swerve.getRotation2d().getRadians(),
        desiredRotation.getRadians());
    turnVelo = slewLimit_turn.calculate(turnVelo * DriveConstants.maxAngularSpeedRadiansPerSecond);

    SmartDashboard.putNumber("cmdcurRot", swerve.getRotation2d().getRadians());
    SmartDashboard.putNumber("cmddesRot", desiredRotation.getRadians());

    ChassisSpeeds newChassisSpeeds;
    if (preserveVelocity) {
      ChassisSpeeds currentChassisSpeeds = swerve.getChassisSpeeds();
      newChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          currentChassisSpeeds.vxMetersPerSecond,
          currentChassisSpeeds.vyMetersPerSecond, turnVelo,
          swerve.getRotation2d());
    } else {
      newChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, turnVelo,
          swerve.getRotation2d());
    }

    swerve.setChassisSpeeds(newChassisSpeeds);
    swerve.logDesiredOdometry(new Pose2d(swerve.getPose().getX(), swerve.getPose().getY(), desiredRotation));
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return pid_turnController.atSetpoint();
  }
}