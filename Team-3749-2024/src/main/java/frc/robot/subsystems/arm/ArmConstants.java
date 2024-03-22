package frc.robot.subsystems.arm;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;

public final class ArmConstants {

        public static final int leftID = 15;
        public static final int rightID = 16;
        // inverse gear ratio * 1min/60sec * 2PI to get rad/sec
        public static final double gearRatio = 200.0;

        public static final double sprocketRatio = 64.0 / 24.0; 
        public static final int encoderID = 7;
        public static final double encoderOffsetRad = Units.degreesToRadians(122.62);

        public static final double stowPositionRad = Units.degreesToRadians(1);
        public static final double climbPositionRad = Units.degreesToRadians(102.5);
        public static final double subwooferPositionRad = Units.degreesToRadians(13.5);
        public static final double ampPositionRad = Units.degreesToRadians(114);
        public static final double podiumPositionRad = Units.degreesToRadians(35);
        public static final double groundIntakepositionRad = Units.degreesToRadians(4.5);

        public enum ArmStates {
                
                STOW,
                AMP,
                SHOOT,
                CLIMB,
                SUBWOOFER,
                PODIUM,
                GROUND_INTAKE,
                AIMBOT,
                IN_TRANIST;
        }


        // Control - PID, FF, and Trapezoidal Constraints
        private static final PIDConstants simPID = new PIDConstants(0, 0, 0); // 2.2,0,0
        private static final PIDConstants realStowedPID = new PIDConstants(0.55, 0, 0.05);
        public static final PIDConstants stowedPID = Robot.isReal() ? realStowedPID : simPID;
       
        private static final PIDConstants realDeployedPID = new PIDConstants(0.55, 0, 0.05);
        public static final PIDConstants deployedPID = Robot.isReal() ? realDeployedPID : new PIDConstants(0);

        private static final double simkS = 0.0;
        private static final double simkG = 0.203; // stick arm at 0 degrees, tune till it doesnt move

        private static final double simkV = 6.616; // max volts - kG / max velocity
        private static final double simkA = 0; // (max volts - kG - vel@maxacceleration*kV )/max acceleration

        private static final double realStowedkS = 0.205;
        private static final double realStowedkG = 0.375; // 0.58 - 0.17
        private static final double realStowedkV = 3.9;
        private static final double realStowedkA = 0.165;

        public static final double stowedkS = Robot.isReal() ? realStowedkS : simkS;
        public static final double stowedkG = Robot.isReal() ? realStowedkG : simkG;
        public static final double stowedkV = Robot.isReal() ? realStowedkV : simkV;
        public static final double stowedkA = Robot.isReal() ? realStowedkA : simkA;

        private static final double realDeployedkS = 0.315;
        private static final double realDeployedkG = 0.585; // 0.90 - 0.28
        private static final double realDeployedkV = 3.9;
        private static final double realDeployedkA = 0.165;

        public static final double deployedkS = Robot.isReal() ? realDeployedkS : simkS;
        public static final double deployedkG = Robot.isReal() ? realDeployedkG : simkG;
        public static final double deployedkV = Robot.isReal() ? realDeployedkV : simkV;
        public static final double deployedkA = Robot.isReal() ? realDeployedkA : simkA;

        // private static final Constraints simConstraints = new Constraints(2.36,
        // 71.58);
        private static final Constraints simStowedConstraints = new Constraints(
                        1.783,
                        89.175);

        private static final Constraints realStowedConstraints = new Constraints(        
                        2.662,
                        5.5);
        public static final Constraints stowedConstraints = Robot.isReal()                
                        ? realStowedConstraints
                        : simStowedConstraints;

        private static final Constraints realDeployedConstraints = new Constraints(        
                        2.442,
                        5.5

                        );
        public static final Constraints                 
        deployedConstraints = Robot.isReal()
                        ? realDeployedConstraints
                        : new Constraints(0, 0);
                


        // Field Parameters
        public static final double armHeightOffset = Units.inchesToMeters(12.75);
        public static final double armLengthOffset = -0.2286;
        public static final double armLength = 0.61;
        public static final double shooterVelocity = 12.5; // NOTE: likely will vary, might need to pass as
                                                           // parameter

        // Field Parameters
        public static final double speakerHeight = Units.inchesToMeters(78.13);; // likely thing you'll need to tune
        public static final double minDistance = 0.9; // / NOTE: Not percise, please check

        // Calcuation stuff
        // Max angle??? (ask Bailey)
        public static final double distMargin = 0.25; // Half a meter is kind of a lot, don't you think?
        public static final double maxAngle = 42.109;
        public static final double maxAngleRad = Math.toRadians(maxAngle);

        // Important Field Coordinates (everything converted from inches to meters)
        // NOTE: may need to adjust slightly to make sure code works properly (some
        // inpercision in measurements)
        public static final Translation2d redSpeakerPosition = new Translation2d(
                        Units.inchesToMeters(653.2),
                        Units.inchesToMeters(218.64)); // rounded need to change
        public static final Translation2d blueSpeakerPosition = new Translation2d(
                        0,
                        Units.inchesToMeters(218.64)); // rounded need to change

        public static final double stageMargin = 20; // margin in inches
        public static final Translation2d[] redStagePoints = {
                        new Translation2d(
                                        Units.inchesToMeters(125.01 + stageMargin),
                                        Units.inchesToMeters(161.64)),
                        new Translation2d(
                                        Units.inchesToMeters(231.2 + stageMargin),
                                        Units.inchesToMeters(104.64 + stageMargin)),
                        new Translation2d(
                                        Units.inchesToMeters(231.2 + stageMargin),
                                        Units.inchesToMeters(218.64 + stageMargin))
        };
        public static final Translation2d[] blueStagePoints = {
                        new Translation2d(
                                        653.2 - redStagePoints[0].getX(),
                                        redStagePoints[0].getY()),
                        new Translation2d(
                                        653.2 - redStagePoints[1].getX(),
                                        redStagePoints[1].getY()),
                        new Translation2d(
                                        653.2 - redStagePoints[2].getX(),
                                        redStagePoints[2].getY())
        };
}
