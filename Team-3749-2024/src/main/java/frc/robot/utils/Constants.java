package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class Constants {
        /***
     * 
     * @param margin how close the values need to be to return true. Use a positive
     *               number
     * @param a      the first number
     * @param b      the second number
     * @return true if it is within the margin, false if not
     */
    public static boolean withinMargin(double margin, double a, double b) {
        if (a + margin >= b && a - margin <= b) {
            return true;
        }
        return false;
    }

    /***
     * 
     * @param margin how close the values need to be to return true. Use a positive
     *               number
     * @param a      the first translation
     * @param b      the second translation
     * @return true if it is within the margin, false if not
     */
    public static boolean withinMargin(double margin, Translation2d a, Translation2d b) {
        // if X is within margin
        if (a.getX() + margin >= b.getX() && a.getX() - margin <= b.getX()) {
            // if Y is within margin
            if (a.getY() + margin >= b.getY() && a.getY() - margin <= b.getY()) {

                return true;
            }
        }
        return false;
    }

    public static enum RobotType {
        REAL,
        SIM
      }
    
    public static final RobotType ROBOT_TYPE = Robot.isReal() ? RobotType.REAL : RobotType.SIM;


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
        public static enum Node {
            CONE(0), CUBE(Units.inchesToMeters(14.25)), MID_CONE(24), TOP_CONE(43);

            public double height_meters;

            Node(double height_meters) {
                this.height_meters = height_meters;
            }
        };
        public static enum Cam {
            LEFT(0), RIGHT(1),BACK(2);
            public int camNum;
            Cam(int camNum){
                this.camNum = camNum;
            }
        }
        //REPLACE WITH THE ACTUAL VALUES THESE ARE PLACEHOLDER
        public static final Transform3d CAM_LEFT_TO_ROBOT = new Transform3d(
                new Translation3d(Units.inchesToMeters(11), 0, -Units.inchesToMeters(15.25)), new Rotation3d());
        public static final Transform3d ROBOT_LEFT_TO_CAM = CAM_LEFT_TO_ROBOT.inverse();
        public static final Transform3d SIM_LEFT_ROBOT_TO_CAM = new Transform3d(1, 0, 0, new Rotation3d());
        public static final Transform2d STDV_LEFT = new Transform2d(0.5, 0.5, new Rotation2d(5));
        //REPLACE WITH THE ACTUAL VALUES THESE ARE PLACEHOLDER
        public static final Transform3d CAM_RIGHT_TO_ROBOT = new Transform3d(
                new Translation3d(-Units.inchesToMeters(11), 0, -Units.inchesToMeters(15.25)), new Rotation3d());
        public static final Transform3d ROBOT_RIGHT_TO_CAM = CAM_LEFT_TO_ROBOT.inverse();
        public static final Transform3d SIM_RIGHT_ROBOT_TO_CAM = new Transform3d(1, 0, 0, new Rotation3d());
        public static final Transform2d STDV_RIGHT = new Transform2d(0.5, 0.5, new Rotation2d(5));


        public static final int REFLECTIVE_PIPELINE_INDEX = 0;
        public static final int APRILTAG_PIPELINE_INDEX = 1;

        public static final double CAM_HEIGHT = Units.inchesToMeters(20); // meters
        public static final double SIM_CAM_HEIGHT = 1;
        public static final double CAM_YAW = 0;
        public static final double CAM_PITCH = 0;
        public static final double CAM_ROLL = 0;

        public static final Matrix<N3, N1>   VISION_MEASUREMENT_STANDARD_DEVIATIONS = 
            MatBuilder.fill(Nat.N3(), Nat.N1(), 1.0, 1.0, 1.0 * Math.PI);
        public static final int DISTANCE_WEIGHT = 7;
        public static final double POSE_AMBIGUITY_MULTIPLIER = 0.2;
        public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
        public static final double NOISY_DISTANCE_METERS = 2.5;
        public static final int TAG_PRESENCE_WEIGHT = 10;

        public static enum Pipelines {
            APRILTAG(1),
            CUBE(0);
            
            public int index;

            Pipelines(int index) {
                this.index = index;
            }
        }

        public static enum Node {
            CONE(0), CUBE(Units.inchesToMeters(14.25)), MID_CONE(24), TOP_CONE(43);

            public double height_meters;

            Node(double height_meters) {
                this.height_meters = height_meters;
            }
        };

    }

    
}