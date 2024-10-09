package frc.robot.subsystems.wrist;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;

public final class WristConstants {

        public static final int wristId = 17;
        public static final double gearRatio = 1;

        public static final double almostDeployedRad = Units.degreesToRadians(110); // 141
        public static final double stowGoalRad = Units.degreesToRadians(2);
        public static final double fullDeployedRad = Units.degreesToRadians(160);
        public static final double passingRad = Units.degreesToRadians(45);

        public static final double subwooferRad = fullDeployedRad;
        public static final double podiumRad = fullDeployedRad;

        public enum WristStates {
                IN_TRANIST,
                ALMOST_DEPLOYED,
                FULL_DEPLOYED,
                SUBWOOFER,
                PODIUM,
                STOW,
                PASS;
        }

        public static final double wristOffsetRad = Units.degreesToRadians(73);
        public static final boolean wristReversed = true;

        private static final PIDConstants simPID = new PIDConstants(35, 0, 1);

        private static final PIDConstants realPID = new PIDConstants(0.9, 0.0, 0); // 0.35

        public static final PIDConstants PID = Robot.isReal() ? realPID : simPID;

        private static final Constraints simConstraint = new Constraints(
                        Math.PI,
                        2 * Math.PI); // we stealing from arm with
        // this one
        private static final Constraints realConstraint = new Constraints(
                        2.25 * Math.PI,
                        1.75 * Math.PI);

        public static final Constraints trapezoidConstraint = Robot.isReal()
                        ? realConstraint
                        : simConstraint;

        // thanks arm (robbery)
        public static final double simkS = 0.0;
        public static final double simkG = .33775;
        public static final double simkV = 6.616;
        public static final double simkA = 0;

        public static final double realkS = 0.0;
        public static final double realkVForward = 3.95;// 1.6 // radians
        public static final double realkVBackward = 2.8; // radians

        public static final double kYIntercept = 0.0654;
        public static final double kBar = 0.635;
        public static final double kBarSquared = -0.898;
        public static final double kBarCubed = 0.237;

}
