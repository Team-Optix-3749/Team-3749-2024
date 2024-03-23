package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
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

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/***
 * @author Noah Simon
 * @author Raadwan Masum
 * @author Rohin Sood
 *         Default command to control the SwervedriveSubsystem with joysticks
 */

public class AlignToAmp extends Command {

    private final Swerve swerve;

    private final PIDController xController = new PIDController(2, 0, 0);
    private final PIDController yController = new PIDController(2, 0, 0);
    private final PIDController turnController = new PIDController(5, 0, 0.15);


    public AlignToAmp() {
        this.swerve = Robot.swerve;

        addRequirements(swerve);
        turnController.enableContinuousInput(0, 2 * Math.PI);
        turnController.setTolerance((Units.degreesToRadians(1.5)));
        xController.setTolerance(0.05);
        yController.setTolerance(0.05);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {


        // for the entirety of comp, this block of code meant nothing
        Pose2d currentPose = Robot.swerve.getPose();
        
        double currentRotationRad = currentPose.getRotation().getRadians();
        double desiredRotationRad = -Math.PI/2;
        while (desiredRotationRad < 0) {
            desiredRotationRad += 2 * Math.PI;
        }
        while (desiredRotationRad >= 2 * Math.PI) {
            desiredRotationRad -= 2 * Math.PI;
        }
        while (currentRotationRad < 0) {
            currentRotationRad += 2 * Math.PI;
        }
        while (currentRotationRad >= 2 * Math.PI) {
            currentRotationRad -= 2 * Math.PI;
        }

        Translation2d desiredCords = new Translation2d(1.76,7.73);
    
        if (MiscConstants.isRedAlliance()){
            desiredCords = GeometryUtil.flipFieldPosition(desiredCords);
        } 

        double turningSpeed = turnController.calculate(currentRotationRad, desiredRotationRad);
        double xSpeed = xController.calculate(currentPose.getX(), desiredCords.getX());
        double ySpeed = yController.calculate(currentPose.getY(), desiredCords.getY());

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed,
            ySpeed,
            turningSpeed,
            swerve.getRotation2d());        // set chassis speeds
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
