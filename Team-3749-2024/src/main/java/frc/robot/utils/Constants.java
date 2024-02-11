package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import com.pathplanner.lib.util.PIDConstants;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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

    public static final class Sim {
        public static final double loopPeriodSec = 0.02;

        public static final class PIDValues {
            // will eventally be easier to change values from here than poke around through
            // files
            public static double kP_teleopTurn = 1.3;
            public static double kD_teleopTurn = 0.0;

            public static double kP_MiscDrive = 0.42;
            public static double kD_MiscDrive = 0.02;
            public static double kP_MiscTurn = 0.15;
            public static double kD_MiscTurn = 0.003;

            public static double kP_TurnToAngle = 0.15;
            public static double kD_TurnToAngle = 0.008;
        };

    }

    public static enum RobotState {
        STOW(0, 0),
        SPEAKER(30, 0),
        AMP(80, 0);

        public double armPosRad;
        public double shintakePosRad;

        private RobotState(double armPosRad, double shintakePosRad) {
            this.armPosRad = armPosRad;
            this.shintakePosRad = shintakePosRad;
        }
    }

    public static final class ArmConstants {
        private static final PIDConstants simPID = new PIDConstants(0, 0, 0); // 2.2,0,0
        private static final PIDConstants realPID = new PIDConstants(0, 0, 0);

        public static final PIDConstants PID = Robot.isReal() ? realPID : simPID;

        public static final int leftID = 0;
        public static final int rightID = 1;
        // inverse gear ratio * 1min/60sec * 2PI to get rad/sec
        public static final double relativeEncoderVelocityConversionFactor = 1 / 150 * 1 / 60 * Math.PI * 2;
        public static final int encoderID = 2;
        public static final double encoderOffsetRad = 0;

        // Control - PID, FF, and Trapezoidal Constraints

        private static final double simkS = 0.0;
        private static final double simkG = 0.203;// stick arm at 0 degrees, tune till it doesnt move

        private static final double simkV = 6.616; // max volts - kG / max velocity
        private static final double simkA = 0; // (max volts - kG - vel@maxacceleration*kV )/max acceleration

        private static final double realkS = 0;
        private static final double realkG = 0;
        private static final double realkV = 0;
        private static final double realkA = 0;

        public static final double kS = Robot.isReal() ? realkS : simkS;
        public static final double kG = Robot.isReal() ? realkG : simkG;
        public static final double kV = Robot.isReal() ? realkV : simkV;
        public static final double kA = Robot.isReal() ? realkA : simkA;

        // private static final Constraints simConstraints = new Constraints(2.36,
        // 71.58);
        private static final Constraints simConstraints = new Constraints(1.783, 89.175);

        private static final Constraints realConstraints = new Constraints(Math.PI, 2 * Math.PI);
        public static final Constraints constraints = Robot.isReal() ? realConstraints : simConstraints;

        // Field Parameters
        public static final double armHeightOffset = 0.0; // how high up the arm is, NOTE: need to find
        public static final double armLength = 0.93;
        public static final double shooterVelocity = 10.0; // NOTE: likely will vary, might need to pass as parameter

        // NOTE: Not percise
        // Field Parameters
        public static final double speakerHeight = 2.05;
        public static final double minDistance = 0.9;

        // Calcuation stuff
        public static final double distMargin = 0.5;
        public static final double maxAngle = 42.109;
        public static final double maxAngleRad = Math.toRadians(maxAngle);
    }

    public static final class ModuleConstants {
        public static final double wheelDiameterMeters = Units.inchesToMeters(4);
        public static final double driveMotorGearRatio = 6.75;
        public static final double turnMotorGearRatio = 12.8;
        public static final double kPTurningReal = 2.25;
        public static final double kPDrivingReal = 0.0;
        public static final double kVDrivingReal = 2.5;
        public static final double kSDrivingReal = 0.0;

        public static final double kPTurningSim = 4;
        public static final double kVDrivingSim = 3.19;
        public static final double kSDrivingSim = 0.0;
        public static final double kPDrivingSim = 0.0;

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
            LEFT(0), RIGHT(1), BACK(2);

            public int camNum;

            Cam(int camNum) {
                this.camNum = camNum;
            }
        }

        // REPLACE WITH THE ACTUAL VALUES THESE ARE PLACEHOLDER
        public static final Transform3d CAM_LEFT_TO_ROBOT = new Transform3d(
                new Translation3d(Units.inchesToMeters(11), 0, -Units.inchesToMeters(15.25)), new Rotation3d());
        public static final Transform3d ROBOT_LEFT_TO_CAM = CAM_LEFT_TO_ROBOT.inverse();
        public static final Transform3d SIM_LEFT_ROBOT_TO_CAM = new Transform3d(1, 0, 0, new Rotation3d());
        public static final Transform2d STDV_LEFT = new Transform2d(0.5, 0.5, new Rotation2d(5));
        // REPLACE WITH THE ACTUAL VALUES THESE ARE PLACEHOLDER
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

        public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS = MatBuilder.fill(Nat.N3(), Nat.N1(),
                1.0, 1.0, 1.0 * Math.PI);
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

        public static final boolean[] driveEncoderReversed = { true, false, true, false };
        public static final boolean[] turningEncoderReversed = { false, false, false, false };

        public static final int[] absoluteEncoderPorts = { 9, 10, 11, 12 };

        public static final boolean[] driveAbsoluteEncoderReversed = { false, false, false, false };

        public static final double[] driveAbsoluteEncoderOffsetDeg = { 130.34, 107.75, 61.70, 168.75 };

        private static final double realMaxSpeedMetersPerSecond = 3.707;
        private static final double realMaxAngularSpeedRadiansPerSecond = 11.795;
        private static final double realMaxAccelerationMetersPerSecondSquared = 7.800;
        private static final double realMaxAngularAccelerationRadiansPerSecondSquared = 30.273;

        public static final int driveMotorStallLimit = 30;
        public static final int driveMotorFreeLimit = 60;
        public static final int turnMotorStallLimit = 30;
        public static final int turnMotorFreeLimit = 60;

        private static final double simMaxSpeedMetersPerSecond = 3.707;
        private static final double simMaxAngularSpeedRadiansPerSecond = 11.795;
        private static final double simMaxAccelerationMetersPerSecondSquared = 2.5;
        private static final double simMaxAngularAccelerationRadiansPerSecondSquared = 30.273;

        // private static final double simMaxSpeedMetersPerSecond = 2.65;
        // private static final double simMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
        // private static final double simMaxAccelerationMetersPerSecondSquared = 2.5;
        // private static final double simMaxAngularAccelerationRadiansPerSecondSquared
        // = 2 * 2 * Math.PI;

        private static final double simMaxMotorVoltage = 12.0;
        private static final double realMaxMotorVoltage = 12.0;

        public static final double maxMotorVolts = Robot.isReal() ? DriveConstants.realMaxMotorVoltage
                : DriveConstants.simMaxMotorVoltage;

        public static final double maxSpeedMetersPerSecond = Robot.isReal()
                ? DriveConstants.realMaxSpeedMetersPerSecond
                : DriveConstants.simMaxSpeedMetersPerSecond;

        public static final double maxAngularSpeedRadiansPerSecond = Robot.isReal()
                ? DriveConstants.realMaxAngularSpeedRadiansPerSecond
                : DriveConstants.simMaxAngularSpeedRadiansPerSecond;

        public static final double maxAccelerationMetersPerSecondSquared = Robot.isReal()
                ? DriveConstants.realMaxAccelerationMetersPerSecondSquared
                : DriveConstants.simMaxAccelerationMetersPerSecondSquared;

        public static final double maxAngularAccelerationRadiansPerSecondSquared = Robot.isReal()
                ? DriveConstants.realMaxAngularAccelerationRadiansPerSecondSquared
                : DriveConstants.simMaxAngularAccelerationRadiansPerSecondSquared;

        public static final double toleranceM_Misc = 0.02;
        public static final double toleranceRad_Misc = Math.PI / 750;

        public static final Pose2d fieldStartingPose = new Pose2d(1, 1, Rotation2d.fromDegrees(0));
    }

    public static final class AutoConstants {
        public static double kP_PathPlannerDrive = 8.8;
        public static double kD_PathPlannerDrive = 0;

        public static double kP_PathPlannerTurn = 0.9;
        public static double kD_PathPlannerTurn = 0;

        public static PIDConstants drivePIDConstants = new PIDConstants(kP_PathPlannerDrive, 0,
                kD_PathPlannerDrive);
        public static PIDConstants turnPIDConstants = new PIDConstants(kP_PathPlannerTurn, 0,
                kD_PathPlannerTurn);

        public static HolonomicPathFollowerConfig cfgHolonomicFollower = new HolonomicPathFollowerConfig(
                // in your Constants class
                drivePIDConstants,
                turnPIDConstants,
                DriveConstants.maxSpeedMetersPerSecond, // Max module speed, in m/s
                Math.sqrt(2 * (DriveConstants.trackWidth * DriveConstants.trackWidth)), // Drivetrain radius
                new ReplanningConfig() // Default path replanning config. See the API for the
        // options here
        );

        public static PathConstraints defaultPathConstraints = new PathConstraints(
                DriveConstants.maxSpeedMetersPerSecond,
                DriveConstants.maxAccelerationMetersPerSecondSquared,
                DriveConstants.maxAngularSpeedRadiansPerSecond,
                DriveConstants.maxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class ControllerConstants {
        public static final double deadband = 0.1;
    }
}