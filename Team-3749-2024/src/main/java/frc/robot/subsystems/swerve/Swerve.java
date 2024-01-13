// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.GyroIO.GyroData;
import frc.robot.subsystems.swerve.SwerveModuleIO.ModuleData;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.RobotType;

/***
 * @author Noah Simon
 * @author Rohin Sood
 * @author Raadwan Masum
 * @author Harkirat
 * 
 *         Subsystem class for swerve drive, used to manage four swerve modules
 *         and set their states. Also includes a pose estimator, gyro, and
 *         logging information
 */
public class Swerve extends SubsystemBase {
    private SwerveModule[] modules = new SwerveModule[4];

    private GyroIO gyro;
    private GyroData gyroData;

    // equivilant to a odometer, but also intakes vision
    private SwerveDrivePoseEstimator swerveDrivePoseEstimator;

    public Swerve() {
        for (int i = 0; i < 4; i++) {
            modules[i] = new SwerveModule(i);
        }
        
        if (Constants.ROBOT_TYPE == RobotType.SIM) {
            gyro = new GyroIO() {
            };

            gyroData = new GyroData();

        } else {
            // real swerve module instatiation here

        }

        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(Constants.DriveConstants.driveKinematics,
                new Rotation2d(0),
                new SwerveModulePosition[] { modules[0].getPosition(), modules[1].getPosition(),
                        modules[2].getPosition(), modules[3].getPosition() },
                new Pose2d(new Translation2d(0, 0), new Rotation2d(0, 0)));

    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.driveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        setModuleStates(moduleStates);
    }

    public ChassisSpeeds getChassisSpeeds() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return DriveConstants.driveKinematics.toChassisSpeeds(states);
    }

    public void resetGyro() {
        gyro.resetGyro();
        System.out.println("RESET");
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(gyroData.yaw / 180 * Math.PI);
    }

    public Pose2d getPose() {

        Pose2d estimatedPose = swerveDrivePoseEstimator.getEstimatedPosition();    
            
        return new Pose2d(estimatedPose.getTranslation(), getRotation2d());
    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return swerveDrivePoseEstimator;
    }

    // Not sure that this works properly
    public void resetOdometry(Pose2d pose) {

        swerveDrivePoseEstimator.resetPosition(getRotation2d(),
                new SwerveModulePosition[] { modules[0].getPosition(), modules[1].getPosition(),
                        modules[2].getPosition(), modules[3].getPosition() },
                pose);
        
    }

    public void updateOdometry() {
        // if (limelight.getEstimatedGlobalPose(swerveDrivePoseEstimator.getEstimatedPosition()).isPresent()){
        //     swerveDrivePoseEstimator.addVisionMeasurement(limelight.getEstimatedGlobalPose(swerveDrivePoseEstimator.getEstimatedPosition()).get().estimatedPose.toPose2d(), limelight.getTimeRunning());
        // }
        swerveDrivePoseEstimator.update(getRotation2d(),
                new SwerveModulePosition[] { modules[0].getPosition(), modules[1].getPosition(),
                        modules[2].getPosition(), modules[3].getPosition() });
    }

    public void stopModules() {
        for (SwerveModule module : modules) {
            module.stop();
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.maxSpeedMetersPerSecond);
        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(desiredStates[i]);
        }
    }

    public double getVerticalTilt() {
        return gyroData.pitch;
    }

    @Override
    public void periodic() {
        updateOdometry();

        for (int i = 0; i < 4; i++) {
            modules[i].periodic();
        }

        SmartDashboard.putNumberArray("Odometry",
                new double[] { getPose().getX(), getPose().getY(), getPose().getRotation().getDegrees() });


        double[] realStates = {
                modules[0].getState().angle.getDegrees(),
                modules[0].getState().speedMetersPerSecond,
                modules[1].getState().angle.getDegrees(),
                modules[1].getState().speedMetersPerSecond,                
                modules[2].getState().angle.getDegrees(),
                modules[2].getState().speedMetersPerSecond,                
                modules[3].getState().angle.getDegrees(),
                modules[3].getState().speedMetersPerSecond
            };


        double[] theoreticalStates = {
                modules[0].getDesiredState().angle.getDegrees(),
                modules[0].getDesiredState().speedMetersPerSecond,
                modules[1].getDesiredState().angle.getDegrees(),
                modules[1].getDesiredState().speedMetersPerSecond,                
                modules[2].getDesiredState().angle.getDegrees(),
                modules[2].getDesiredState().speedMetersPerSecond,                
                modules[3].getDesiredState().angle.getDegrees(),
                modules[3].getDesiredState().speedMetersPerSecond   
            };

        SmartDashboard.putNumberArray("Theoretical States", theoreticalStates);
        SmartDashboard.putNumberArray("Real Staets", realStates);
        SmartDashboard.putNumber("Yaw", getRotation2d().getDegrees());
        SmartDashboard.putNumber("Pitch", getVerticalTilt());
        SmartDashboard.putNumber("Robot Pose X", getPose().getX());
        SmartDashboard.putNumber("Robot Pose Y", getPose().getY());
    }
}