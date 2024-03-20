package frc.robot.utils;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;

public final class AutoConstants {

        public static double kP_PathPlannerDrive = 7; // 3
        public static double kD_PathPlannerDrive = 0.25; // 0.5

        public static double kP_PathPlannerTurn = 4; // 4.75
        public static double kD_PathPlannerTurn = 0.1; // 0.2

        public static PIDConstants drivePIDConstants = new PIDConstants(
                        kP_PathPlannerDrive,
                        0.0,
                        kD_PathPlannerDrive);
        public static PIDConstants turnPIDConstants = new PIDConstants(
                        kP_PathPlannerTurn,
                        0,
                        kD_PathPlannerTurn);

        public static HolonomicPathFollowerConfig cfgHolonomicFollower = new HolonomicPathFollowerConfig(
                        // in your Constants class
                        drivePIDConstants,
                        turnPIDConstants,
                        DriveConstants.maxSpeedMetersPerSecond, // Max module speed, in m/s
                        Math.sqrt(2 * (DriveConstants.trackWidth * DriveConstants.trackWidth)), // Drivetrain
                        // radius
                        new ReplanningConfig() // Default path replanning config. See the API for the
        // options here
        );

        public static PathConstraints defaultPathConstraints = new PathConstraints(
                        DriveConstants.maxSpeedMetersPerSecond,
                        DriveConstants.maxAccelerationMetersPerSecondSquared,
                        DriveConstants.maxAngularSpeedRadiansPerSecond,
                        DriveConstants.maxAngularAccelerationRadiansPerSecondSquared);

}