package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class Constants {

    public static enum RobotType {
        REAL,
        SIM
      }
    
    public static final RobotType ROBOT_TYPE = Robot.isReal() ? RobotType.REAL : RobotType.SIM;

    public static final class Elevator {
        public static final int elevatorMotorOneID = 1;
        public static final int elevatorMotorTwoID = 2;
    }

    public static final class Sim{
        public static final double loopPeriodSec = 0.02;
    }


    public static final class ModuleConstants {
        public static final double wheelDiameterMeters = Units.inchesToMeters(3.5);
        public static final double driveMotorGearRatio = 1.0 / 6.75;
        public static final double turningMotorGearRatio = 1.0 / 12.8;
        public static final double kPTurningReal = 2.25;
        public static final double kPDrivingReal = 0.0;
        public static final double kVDrivingReal = 1.5;
        public static final double kSDrivingReal = 0.0;

        public static final double kPTurningSim = 4;
        public static final double kVDrivingSim = 3.19;
        public static final double kSDrivingSim = 0.0;
        public static final double kPDrivingSim = 0.0;


  
    }

    public static final class DriveConstants {
        // Distance between right and left wheels
        public static final double trackWidth = Units.inchesToMeters(17.5);
        // Distance between front and back wheels
        public static final double wheelBase = Units.inchesToMeters(17.5);
        public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2, trackWidth / 2), // front left
                new Translation2d(wheelBase / 2, -trackWidth / 2), // front right
                new Translation2d(-wheelBase / 2, trackWidth / 2), // back left
                new Translation2d(-wheelBase / 2, -trackWidth / 2)); // back right

        public static final int[] driveMotorPorts = { 1, 3, 7, 5 }; // FL, FR, BL, BR
        public static final int[] turningMotorPorts = { 2, 4, 8, 6 }; // FL, FR, BL, BR

        public static final boolean[] turningEncoderReversed = { false, false, false, false };
        public static final boolean[] driveEncoderReversed = { true, false, true, false };

        public static final int[] absoluteEncoderPorts = { 9, 10, 11, 12 };

        public static final boolean[] driveAbsoluteEncoderReversed = { false, false, false, false };

        public static final double[] driveAbsoluteEncoderOffsetDeg = {130.34, 107.75, 61.70, 168.75};

        public static final double realMaxSpeedMetersPerSecond = 5;
        public static final double realMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
        public static final double simMaxSpeedMetersPerSecond = 2.655;
        public static final double simMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
        public static final double maxSpeedMetersPerSecond = Robot.isReal() ? DriveConstants.realMaxSpeedMetersPerSecond: DriveConstants.simMaxSpeedMetersPerSecond;


    }

    public static final class ControllerConstants {
        public static final double deadband = 0.1;
    }


    public static final class AutoConstants {
        public static final Map<String, Command> eventMap = new HashMap<>();

        public static final SendableChooser<Command> autoChooser = new SendableChooser<>();

    }

    public static class VisionConstants {
        // See
        // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
        // page 208
        public static final double targetWidth = Units.inchesToMeters(41.30) - Units.inchesToMeters(6.70); // meters

        // See
        // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
        // page 197
        public static final double targetHeight = Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); // meters

        public static final Transform3d cam_to_robot = new Transform3d(
                new Translation3d(0, 0, -Units.inchesToMeters(15.25)), new Rotation3d());

        public static final Transform3d robot_to_cam = cam_to_robot.inverse();

        public static final int reflective_tape_pipeline_index = 0;
        public static final int apriltag_pipeline_index = 1;

        public static final double camera_height = Units.inchesToMeters(20); // meters
        public static final double camera_yaw = 0;
        public static final double camera_pitch = 0;
        public static final double camera_roll = 0;

        // msg from Noah: I forget what these do
        public static final double retro_cam_offset = 0.56;
        public static final double apriltag_cam_offset = 3.1;

    }

}