package frc.robot.subsystems.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {

    public static enum Cam {
        LEFT(0), RIGHT(1), BACK(2);

        public int camNum;

        Cam(int camNum) {
            this.camNum = camNum;
        }
    }

    // +X is forward, +Y is left, +Z is up
    public static final Transform3d ROBOT_TO_LEFT_CAM = new Transform3d(
            new Translation3d(Units.inchesToMeters(-10.588), Units.inchesToMeters(10.161),
                    Units.inchesToMeters(10.605)),
            new Rotation3d(0, 55, 158));
    public static final Transform3d LEFT_CAM_TO_ROBOT = ROBOT_TO_LEFT_CAM.inverse();



    public static final Transform3d ROBOT_TO_RIGHT_CAM = new Transform3d(
            new Translation3d(Units.inchesToMeters(-10.510), Units.inchesToMeters(-10.182),
                    Units.inchesToMeters(10.598)),
            new Rotation3d(0, 55, 200));
    public static final Transform3d RIGHT_CAM_TO_ROBOT = ROBOT_TO_RIGHT_CAM.inverse();

    public static final Transform3d SIM_LEFT_ROBOT_TO_CAM = new Transform3d(1, 0, 0, new Rotation3d());
    public static final Transform3d SIM_RIGHT_ROBOT_TO_CAM = new Transform3d(1, 0, 0, new Rotation3d());

    public static final double CAM_HEIGHT = Units.inchesToMeters(20); // meters
    public static final double SIM_CAM_HEIGHT = 1;
    public static final double CAM_YAW = 0;
    public static final double CAM_PITCH = 0;
    public static final double CAM_ROLL = 0;

    public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS = MatBuilder.fill(Nat.N3(),
            Nat.N1(),
            1.0, 1.0, 1.0 * Math.PI);
    public static final int DISTANCE_WEIGHT = 7;
    public static final double POSE_AMBIGUITY_MULTIPLIER = 0.2;
    public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
    public static final double NOISY_DISTANCE_METERS = 2.5;
    public static final int TAG_PRESENCE_WEIGHT = 10;

    public static enum Pipelines {
        APRILTAG(0);

        public int index;

        Pipelines(int index) {
            this.index = index;
        }
    }
}