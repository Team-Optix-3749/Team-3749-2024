// package frc.robot.commands.swerve;

// import java.util.function.Supplier;

// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Robot;
// import frc.robot.subsystems.swerve.Swerve;
// import frc.robot.utils.Constants.ControllerConstants;
// import frc.robot.utils.Constants.DriveConstants;

// /***
//  * @author Noah Simon
//  * @author Raadwan Masum
//  * @author Rohin Sood
//  *         Default command to control the SwervedriveSubsystem with joysticks
//  */

// public class SwerveTeleopCommand extends Command {

//     private final Swerve swerve;
//     private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
//     private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;


//     public SwerveTeleopCommand(
//             Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction) {
//         this.swerve = Robot.swerve;
//         this.xSpdFunction = xSpdFunction;
//         this.ySpdFunction = ySpdFunction;
//         this.turningSpdFunction = turningSpdFunction;

//         // This should be max Acceleration! I think.
//         this.xLimiter = new SlewRateLimiter(DriveConstants.maxSpeedMetersPerSecond);
//         this.yLimiter = new SlewRateLimiter(DriveConstants.maxSpeedMetersPerSecond);
//         this.turningLimiter = new SlewRateLimiter(DriveConstants.maxSpeedMetersPerSecond);
//         addRequirements(swerve);
//     }

//     @Override
//     public void initialize() {

//     }

//     @Override
//     public void execute() {

//         // 1. Get real-time joystick inputs
//         double xSpeed = xSpdFunction.get();
//         double ySpeed = ySpdFunction.get();
//         double turningSpeed = turningSpdFunction.get();
//         // 2. Apply deadband
//         xSpeed = Math.abs(xSpeed) > ControllerConstants.deadband? xSpeed : 0.0;
//         ySpeed = Math.abs(ySpeed) >  ControllerConstants.deadband? ySpeed : 0.0;
//         turningSpeed = Math.abs(turningSpeed) >  ControllerConstants.deadband ? turningSpeed : 0.0;

//         // 3. Make the driving smoother with consistant accelerations
//         xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.maxSpeedMetersPerSecond;
//         ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.maxSpeedMetersPerSecond;
//         turningSpeed = turningLimiter.calculate(turningSpeed)
//                 * DriveConstants.realMaxAngularSpeedRadiansPerSecond;

//         // 4. Construct desired chassis speeds relative to the field
//         ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
//                 xSpeed, ySpeed, turningSpeed, swerve.getRotation2d());

//         // set chassis speeds
//         swerve.setChassisSpeeds(chassisSpeeds);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         swerve.stopModules();
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }