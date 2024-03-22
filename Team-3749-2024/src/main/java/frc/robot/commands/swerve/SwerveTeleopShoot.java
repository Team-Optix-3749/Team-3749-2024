package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ShootKinematics;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.utils.AutoConstants;
import frc.robot.utils.MiscConstants;
import frc.robot.utils.MiscConstants.*;
import java.sql.Driver;
import java.util.function.Supplier;

/***
 * @author Noah Simon
 * @author Raadwan Masum
 * @author Rohin Sood
 *         Default command to control the SwervedriveSubsystem with joysticks
 */

public class SwerveTeleopShoot extends Command {

  private final Swerve swerve;
  private final Supplier<Double> xSpdFunction, ySpdFunction, xTurningSpdFunction;
  private final PIDController turnPID = new PIDController(5, 0, 0.15);

  public SwerveTeleopShoot(
      Supplier<Double> xSpdFunction,
      Supplier<Double> ySpdFunction,
      Supplier<Double> xTurningSpdFunction) {
    this.swerve = Robot.swerve;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.xTurningSpdFunction = xTurningSpdFunction;

    addRequirements(swerve);
    turnPID.enableContinuousInput(0, 2 * Math.PI);
    turnPID.setTolerance(Units.degreesToRadians(1));

  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double xMagnitude = xSpdFunction.get();
    double yMagnitude = ySpdFunction.get();

    double linearMagnitude = Math.hypot(xMagnitude, yMagnitude);
    Rotation2d linearDirection = new Rotation2d(xMagnitude, yMagnitude);

    // deadbands
    linearMagnitude = MathUtil.applyDeadband(linearMagnitude, ControllerConstants.deadband);

    // squaring the inputs for smoother driving at low speeds
    linearMagnitude = Math.copySign(linearMagnitude * linearMagnitude, linearMagnitude);

    double maxSpeed;
    if (DriverStation.isTeleopEnabled()) {
      maxSpeed = SwerveConstants.DriveConstants.teleopMaxSpeedMetersPerSecond;
    }
    double driveSpeedMPS = linearMagnitude * DriveConstants.maxSpeedMetersPerSecond;

    // Calcaulate new linear components
    double xSpeed = driveSpeedMPS * Math.cos(linearDirection.getRadians());
    double ySpeed = driveSpeedMPS * Math.sin(linearDirection.getRadians());
    ChassisSpeeds chassisSpeeds;

    // for the entirety of comp, this block of code meant nothing
    double currentRotationRad = Robot.swerve.getRotation2d().getRadians();
    double desiredRotationRad = ShootKinematics.getRobotRotation(Robot.swerve.getPose()).getRadians();
    while (desiredRotationRad < 0){
        desiredRotationRad+= 2 * Math.PI;
    }
    while (desiredRotationRad >= 2 * Math.PI){
        desiredRotationRad-= 2 * Math.PI;
    }
    while (currentRotationRad < 0){
        currentRotationRad+= 2 * Math.PI;
    }
    while (currentRotationRad >= 2 * Math.PI){
        currentRotationRad-= 2 * Math.PI;
    }
    
    double turningSpeed = turnPID.calculate(currentRotationRad, desiredRotationRad);
    SmartDashboard.putNumber("desired rot", desiredRotationRad);
        SmartDashboard.putNumber("turn sped", turningSpeed);
    SmartDashboard.putNumber("error", turnPID.getPositionError());
    if (turnPID.atSetpoint()){
        turningSpeed = 0;
    }
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        ySpeed,
        xSpeed,
        turningSpeed,
        swerve.getRotation2d());

    if (MiscConstants.isRedAlliance()) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          -ySpeed,
          -xSpeed,
          turningSpeed,
          swerve.getRotation2d());
    }

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
