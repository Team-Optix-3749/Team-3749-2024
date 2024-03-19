// Importing necessary libraries
package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.vision.VisionConstants.Cam;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.LimelightHelpers.LimelightPose;

/**
 * Encapsulated PhotonCamera object used in posed estimation and alignment
 * 
 * @author Rohin Sood
 * @author Jadon Lee
 */
public class Limelight extends SubsystemBase {
    // Position
    public Pose2d estimatedPose2dLeft = new Pose2d(0, 0, new Rotation2d());
    public Pose2d estimatedPose2dRight = new Pose2d(0, 0, new Rotation2d());

    public boolean targeting = false;
    // PhotonCamera instance
    private final PhotonCamera cameraLeft;
    private final PhotonCamera cameraRight;
    // private final PhotonCamera cameraBack = new PhotonCamera("limelight2");

    private AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonPoseEstimator photonPoseEstimatorLeft;
    private PhotonPoseEstimator photonPoseEstimatorRight;

    // Timer for tracking how long the Limelight subsystem has been running
    // Constructor
    public Limelight() {

        cameraLeft = new PhotonCamera("LL2");
        cameraRight = new PhotonCamera("LL3");
        try {
            // Loading AprilTag field layout
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

            // Initializing PhotonPoseEstimator based on robot type
            if (Robot.isSimulation()) {
                photonPoseEstimatorLeft = new PhotonPoseEstimator(aprilTagFieldLayout,
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        cameraLeft, VisionConstants.SIM_LEFT_ROBOT_TO_CAM);
                photonPoseEstimatorRight = new PhotonPoseEstimator(aprilTagFieldLayout,
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        cameraRight, VisionConstants.SIM_RIGHT_ROBOT_TO_CAM);
            } else {
                photonPoseEstimatorLeft = new PhotonPoseEstimator(aprilTagFieldLayout,
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        cameraLeft, VisionConstants.ROBOT_TO_LEFT_CAM);

                photonPoseEstimatorRight = new PhotonPoseEstimator(aprilTagFieldLayout,
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        cameraRight, VisionConstants.ROBOT_TO_RIGHT_CAM);
            }
            photonPoseEstimatorLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            photonPoseEstimatorRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        } catch (Exception e) {
            // Handling exceptions during initialization
            System.out.println(e);
        }

    }

    // Overridden periodic method for logging during each robot loop iteration
    @Override
    public void periodic() {
        manualPose();

        // System.out.println("right detect");

    }

    public void manualPose() {

        PhotonPipelineResult latestResultLeft = cameraLeft.getLatestResult();
        SmartDashboard.putBoolean(" left present", latestResultLeft.hasTargets());

        if (latestResultLeft.hasTargets()) {
            MultiTargetPNPResult multiResultLeft = latestResultLeft.getMultiTagResult();

            if (multiResultLeft.estimatedPose.isPresent) {
                try {
                    SmartDashboard.putNumber("left ambiguity", multiResultLeft.estimatedPose.ambiguity);

                    if (multiResultLeft.estimatedPose.ambiguity <= 0.2) {
                        Transform3d estimatedTransform = multiResultLeft.estimatedPose.best;
                        estimatedPose2dLeft = new Pose2d(estimatedTransform.getX(), estimatedTransform.getY(),
                                estimatedTransform.getRotation().toRotation2d()).transformBy(
                                        new Transform2d(VisionConstants.LEFT_CAM_TO_ROBOT.getX(),
                                                VisionConstants.LEFT_CAM_TO_ROBOT.getY(),
                                                VisionConstants.LEFT_CAM_TO_ROBOT.getRotation().toRotation2d()));
                                                
                        estimatedPose2dLeft = applyTransformation(
                                new Pose2d(estimatedTransform.getX(), estimatedTransform.getY(),
                                        estimatedTransform.getRotation().toRotation2d()),
                                new Transform2d(VisionConstants.LEFT_CAM_TO_ROBOT.getX(),
                                        VisionConstants.LEFT_CAM_TO_ROBOT.getY(),
                                        VisionConstants.LEFT_CAM_TO_ROBOT.getRotation().toRotation2d()));


                        // update swerve pose estimator
                        Robot.swerve.visionUpdateOdometry(
                                new LimelightHelpers.LimelightPose(estimatedPose2dLeft,
                                        latestResultLeft.getTimestampSeconds()));
                        // Logging Limelight odometry information to SmartDashboard
                        SmartDashboard.putNumberArray("Left Limelight Odometry",
                                new double[] { estimatedPose2dLeft.getX(),
                                        estimatedPose2dLeft.getY(),
                                        estimatedPose2dLeft.getRotation().getRadians() });
                    }
                } catch (Exception e) {
                    SmartDashboard.putString("Left Error", e.toString());

                }
            } else {

                double imageCaptureTime = latestResultLeft.getTimestampSeconds();
                PhotonTrackedTarget bestTarget = latestResultLeft.getBestTarget();

                if (bestTarget.getPoseAmbiguity() <= 0.2) {

                    int targetID = bestTarget.getFiducialId();
                    Optional<Pose3d> targetPoseOptional = aprilTagFieldLayout.getTagPose(targetID);

                    if (targetPoseOptional.isPresent()) {
                        Pose3d targetPose = targetPoseOptional.get();
                        SmartDashboard.putNumberArray("targetPose", new double[] { targetPose.getX(), targetPose.getY(),
                                targetPose.getRotation().toRotation2d().getDegrees() });

                        Pose2d camPose = targetPose.transformBy(bestTarget.getBestCameraToTarget().inverse())
                                .toPose2d();

                        SmartDashboard.putNumberArray("camPose", new double[] { camPose.getX(), camPose.getY(),
                                camPose.getRotation().getDegrees() });

                        SmartDashboard.putNumberArray("origin", new double[] { 0, 0, 0 });
                        SmartDashboard.putNumberArray("cam offset",
                                new double[] { VisionConstants.LEFT_CAM_TO_ROBOT.getX(),
                                        VisionConstants.LEFT_CAM_TO_ROBOT.getY(),
                                        VisionConstants.LEFT_CAM_TO_ROBOT.getRotation().toRotation2d().getDegrees() });

                        Pose2d robotPose = camPose.transformBy(VisionConstants.LEFT_CAM_TO_ROBOT2D);
                        robotPose = applyTransformation(camPose, VisionConstants.LEFT_CAM_TO_ROBOT2D);

                        // Robot.swerve.visionUpdateOdometry(
                        // new LimelightHelpers.LimelightPose(robotPoseEstimate, imageCaptureTime));
                        SmartDashboard.putNumberArray("Left Limelight Odometry",
                                new double[] { robotPose.getX(),
                                        robotPose.getY(),
                                        robotPose.getRotation()
                                                .getDegrees() });

                        // SmartDashboard.putNumberArray("Left Limelight Odometry",
                        // new double[] { robotPoseEstimate.getX(),
                        // robotPoseEstimate.getY(),
                        // robotPoseEstimate.getRotation().getDegrees() });

                    }
                }
            }
        }
        PhotonPipelineResult latestResultRight = cameraRight.getLatestResult();
        SmartDashboard.putBoolean(" Right present", latestResultRight.hasTargets());

        if (latestResultRight.hasTargets()) {
            MultiTargetPNPResult multiResultRight = latestResultRight.getMultiTagResult();

            if (multiResultRight.estimatedPose.isPresent) {
                try {
                    SmartDashboard.putNumber("Right ambiguity", multiResultRight.estimatedPose.ambiguity);

                    if (multiResultRight.estimatedPose.ambiguity <= 0.2) {
                        Transform3d estimatedTransform = multiResultRight.estimatedPose.best;
                        estimatedPose2dRight = new Pose2d(estimatedTransform.getX(), estimatedTransform.getY(),
                                estimatedTransform.getRotation().toRotation2d()).transformBy(
                                        new Transform2d(VisionConstants.RIGHT_CAM_TO_ROBOT.getX(),
                                                VisionConstants.RIGHT_CAM_TO_ROBOT.getY(),
                                                VisionConstants.RIGHT_CAM_TO_ROBOT.getRotation().toRotation2d()));

                        // update swerve pose estimator
                        Robot.swerve.visionUpdateOdometry(
                                new LimelightHelpers.LimelightPose(estimatedPose2dRight,
                                        latestResultRight.getTimestampSeconds()));
                        // Logging Limelight odometry information to SmartDashboard
                        SmartDashboard.putNumberArray("Right Limelight Odometry",
                                new double[] { estimatedPose2dRight.getX(),
                                        estimatedPose2dRight.getY(),
                                        estimatedPose2dRight.getRotation().getRadians() });
                    }
                } catch (Exception e) {
                    SmartDashboard.putString("Right Error", e.toString());

                }
            } else {

                double imageCaptureTime = latestResultRight.getTimestampSeconds();
                PhotonTrackedTarget bestTarget = latestResultRight.getBestTarget();

                if (bestTarget.getPoseAmbiguity() <= 0.2) {

                    int targetID = bestTarget.getFiducialId();
                    Optional<Pose3d> targetPoseOptional = aprilTagFieldLayout.getTagPose(targetID);

                    if (targetPoseOptional.isPresent()) {
                        Pose3d targetPose = targetPoseOptional.get();
                        SmartDashboard.putNumberArray("targetPose", new double[] { targetPose.getX(), targetPose.getY(),
                                targetPose.getRotation().toRotation2d().getDegrees() });

                        Pose2d camPose = targetPose.transformBy(bestTarget.getBestCameraToTarget().inverse())
                                .toPose2d();

                        SmartDashboard.putNumberArray("camPose", new double[] { camPose.getX(), camPose.getY(),
                                camPose.getRotation().getDegrees() });

                        SmartDashboard.putNumberArray("origin", new double[] { 0, 0, 0 });
                        SmartDashboard.putNumberArray("cam offset",
                                new double[] { VisionConstants.RIGHT_CAM_TO_ROBOT.getX(),
                                        VisionConstants.RIGHT_CAM_TO_ROBOT.getY(),
                                        VisionConstants.RIGHT_CAM_TO_ROBOT.getRotation().toRotation2d().getDegrees() });

                        // Robot.swerve.visionUpdateOdometry(
                        // new LimelightHelpers.LimelightPose(robotPoseEstimate, imageCaptureTime));
                        SmartDashboard.putNumberArray("Left Limelight Odometry",
                                new double[] { camPose.transformBy(VisionConstants.LEFT_CAM_TO_ROBOT2D).getX(),
                                        camPose.transformBy(VisionConstants.LEFT_CAM_TO_ROBOT2D).getY(),
                                        camPose.transformBy(VisionConstants.LEFT_CAM_TO_ROBOT2D).getRotation()
                                                .getDegrees() });

                        // SmartDashboard.putNumberArray("Left Limelight Odometry",
                        // new double[] { robotPoseEstimate.getX(),
                        // robotPoseEstimate.getY(),
                        // robotPoseEstimate.getRotation().getDegrees() });

                    }
                }
            }
        }

    }

    // Thanks to FRC Team 5712
    public Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation) {
        double smallestDistance = Double.POSITIVE_INFINITY;
        for (var target : estimation.targetsUsed) {
            var t3d = target.getBestCameraToTarget();
            var distance = Math.sqrt(t3d.getX() * t3d.getX() + t3d.getY() * t3d.getY() + t3d.getZ() * t3d.getZ());
            if (distance < smallestDistance)
                smallestDistance = distance;
        }
        double poseAmbiguityFactor = estimation.targetsUsed.size() != 1
                ? 1
                : Math.max(
                        1,
                        (estimation.targetsUsed.get(0).getPoseAmbiguity()
                                + VisionConstants.POSE_AMBIGUITY_SHIFTER)
                                * VisionConstants.POSE_AMBIGUITY_MULTIPLIER);
        double confidenceMultiplier = Math.max(
                1,
                (Math.max(
                        1,
                        Math.max(0, smallestDistance - VisionConstants.NOISY_DISTANCE_METERS)
                                * VisionConstants.DISTANCE_WEIGHT)
                        * poseAmbiguityFactor)
                        / (1
                                + ((estimation.targetsUsed.size() - 1)
                                        * VisionConstants.TAG_PRESENCE_WEIGHT)));

        return VisionConstants.VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(confidenceMultiplier);
    }

    public static Pose2d applyTransformation(Pose2d startingPose,
            Transform2d transformation) {
        // Apply rotation to the translation

        double newX = startingPose.getX() + -transformation.getX() * Math.cos(transformation.getRotation().getRadians())
                + transformation.getY() * Math.sin(transformation.getRotation().getRadians());
        double newY = startingPose.getY() - transformation.getX() * Math.sin(transformation.getRotation().getRadians())
                - transformation.getY() * Math.cos(transformation.getRotation().getRadians());

        // Apply rotation
        double newTheta = startingPose.getRotation().getRadians() + transformation.getRotation().getRadians();

        // Ensure theta remains within the range [0, 2π)
        newTheta = normalizeAngle(newTheta);

        return new Pose2d(newX, newY, new Rotation2d(newTheta));
    }

    /**
     * Normalizes an angle to the range [0, 2π)
     * 
     * @param angle The angle in radians to normalize
     * @return The normalized angle
     */
    private static double normalizeAngle(double angle) {
        double twoPi = 2 * Math.PI;
        return ((angle % twoPi) + twoPi) % twoPi;
    }
}
