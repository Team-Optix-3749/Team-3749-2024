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

    // NetworkTables entries for controlling LEDs
    private final NetworkTable photonTableLeft = NetworkTableInstance.getDefault().getTable("photonvisionLeft");
    private final NetworkTableEntry ledModeLeft = photonTableLeft.getEntry("ledMode");
    private final NetworkTableEntry ledModeStateLeft = photonTableLeft.getEntry("ledModeState");
    private final NetworkTableEntry ledModeRequestLeft = photonTableLeft.getEntry("ledModeRequest");
    private final NetworkTable photonTableRight = NetworkTableInstance.getDefault().getTable("photonvisionRight");
    private final NetworkTableEntry ledModeRight = photonTableRight.getEntry("ledMode");
    private final NetworkTableEntry ledModeStateRight = photonTableRight.getEntry("ledModeState");
    private final NetworkTableEntry ledModeRequestRight = photonTableRight.getEntry("ledModeRequest");
    private final NetworkTable photonTableBack = NetworkTableInstance.getDefault().getTable("photonvisionRight");
    private final NetworkTableEntry ledModeBack = photonTableBack.getEntry("ledMode");
    private final NetworkTableEntry ledModeStateBack = photonTableBack.getEntry("ledModeState");
    private final NetworkTableEntry ledModeRequestBack = photonTableBack.getEntry("ledModeRequest");

    // ShuffleData for logging pipeline index
    private final ShuffleData<Integer> pipeline = new ShuffleData<Integer>("Limelight",
            "New Pipeline", -1000);

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

        // Setting LED to Off and starting the timer
        setLED(VisionLEDMode.kOff, Cam.LEFT);
        setLED(VisionLEDMode.kOff, Cam.RIGHT);
        setLED(VisionLEDMode.kOff, Cam.BACK);

    }

    // Method to get the latest PhotonPipelineResult from the camera
    public PhotonPipelineResult getLatestResult(Cam camPose) {
        switch (camPose) {
            case LEFT:
                return cameraLeft.getLatestResult();
            case RIGHT:
                return cameraRight.getLatestResult();
            // case BACK:
            // return cameraBack.getLatestResult();
            default:
                return null;

        }

    }

    // Method to check if a target is present in the latest result
    public boolean hasTarget(PhotonPipelineResult result) {
        SmartDashboard.putBoolean("Has Target", result.hasTargets());
        return result.hasTargets();
    }

    // Method to get a list of tracked targets from the latest result
    public List<PhotonTrackedTarget> getTargets(PhotonPipelineResult result) {
        return result.getTargets();
    }

    // Method to get the best tracked target from the latest result
    public PhotonTrackedTarget getBestTarget(PhotonPipelineResult result) {
        return result.getBestTarget();
    }

    public PhotonPoseEstimator getPoseEstimator() {
        return photonPoseEstimatorLeft;
    }

    // Method to get the yaw (rotation) of a tracked target
    public Rotation2d getYaw(PhotonTrackedTarget target) {
        return new Rotation2d(Math.toRadians(target.getYaw()));
    }

    // Method to get the pitch (elevation) of a tracked target
    public double getPitch(PhotonTrackedTarget target) {
        return target.getPitch();
    }

    // Method to get the area of a tracked target
    public double getArea(PhotonTrackedTarget target) {
        return target.getArea();
    }

    // Method to get the skew of a tracked target
    public double getSkew(PhotonTrackedTarget target) {
        return target.getSkew();
    }

    // Method to get the bounding corners of a tracked target
    public List<TargetCorner> getBoundingCorners(PhotonTrackedTarget target) {
        return target.getDetectedCorners();
    }

    // Method to get the fiducial ID of a tracked target
    public int getTargetId(PhotonTrackedTarget target) {
        return target.getFiducialId();
    }

    // Method to get the pose ambiguity of a tracked target
    public double getPoseAbmiguity(PhotonTrackedTarget target) {
        return target.getPoseAmbiguity();
    }

    // Getter for AprilTagFieldLayout
    public AprilTagFieldLayout getAprilTagFieldLayout() {
        return aprilTagFieldLayout;
    }

    // Getter for the current camera pipeline index
    public int getPipeline(Cam camPose) {
        switch (camPose) {
            case LEFT:
                return cameraLeft.getPipelineIndex();
            case RIGHT:
                return cameraRight.getPipelineIndex();
            // case BACK:
            // return cameraBack.getPipelineIndex();
        }
        return 0;

    }

    // Setter for the camera pipeline index
    public void setPipeline(int index, Cam camPose) {
        switch (camPose) {
            case LEFT:
                cameraLeft.setPipelineIndex(index);
            case RIGHT:
                cameraRight.setPipelineIndex(index);
                // case BACK:
                // cameraBack.setPipelineIndex(index);
        }
    }

    // Method to set the LED mode for the Limelight
    public void setLED(VisionLEDMode ledMode, Cam camPose) {
        switch (camPose) {
            case LEFT:
                switch (ledMode) {
                    case kOn:
                        this.ledModeLeft.setInteger(1);
                        ledModeStateLeft.setInteger(1);
                        ledModeRequestLeft.setInteger(1);
                        break;
                    case kOff:
                        this.ledModeLeft.setInteger(0);
                        ledModeStateLeft.setInteger(0);
                        ledModeRequestLeft.setInteger(0);
                        break;
                    case kBlink:
                        this.ledModeLeft.setInteger(2);
                        ledModeStateLeft.setInteger(2);
                        ledModeRequestLeft.setInteger(2);
                        break;
                    default:
                        this.ledModeLeft.setInteger(-1);
                        ledModeStateLeft.setInteger(-1);
                        ledModeRequestLeft.setInteger(-1);
                        break;
                }
                cameraLeft.setLED(ledMode);
            case RIGHT:
                switch (ledMode) {
                    case kOn:
                        this.ledModeRight.setInteger(1);
                        ledModeStateRight.setInteger(1);
                        ledModeRequestRight.setInteger(1);
                        break;
                    case kOff:
                        this.ledModeRight.setInteger(0);
                        ledModeStateRight.setInteger(0);
                        ledModeRequestRight.setInteger(0);
                        break;
                    case kBlink:
                        this.ledModeRight.setInteger(2);
                        ledModeStateRight.setInteger(2);
                        ledModeRequestRight.setInteger(2);
                        break;
                    default:
                        this.ledModeRight.setInteger(-1);
                        ledModeStateRight.setInteger(-1);
                        ledModeRequestRight.setInteger(-1);
                        break;
                }
                cameraRight.setLED(ledMode);
                // case BACK:
                // switch (ledMode) {
                // case kOn:
                // this.ledModeBack.setInteger(1);
                // ledModeStateBack.setInteger(1);
                // ledModeRequestBack.setInteger(1);
                // break;
                // case kOff:
                // this.ledModeBack.setInteger(0);
                // ledModeStateBack.setInteger(0);
                // ledModeRequestBack.setInteger(0);
                // break;
                // case kBlink:
                // this.ledModeBack.setInteger(2);
                // ledModeStateBack.setInteger(2);
                // ledModeRequestBack.setInteger(2);
                // break;
                // default:
                // this.ledModeBack.setInteger(-1);
                // ledModeStateBack.setInteger(-1);
                // ledModeRequestBack.setInteger(-1);
                // break;
                // }
                // cameraBack.setLED(ledMode);
        }

    }

    // Method to get the estimated global pose using the PhotonPoseEstimator
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose, Cam camNum) {
        switch (camNum) {
            case LEFT:
                photonPoseEstimatorLeft.setReferencePose(prevEstimatedRobotPose);
                return photonPoseEstimatorLeft.update();
            case RIGHT:
                photonPoseEstimatorRight.setReferencePose(prevEstimatedRobotPose);
                return photonPoseEstimatorRight.update();
            default:
                return null;
        }

    }

    // Method for logging information
    public void logging() {
        pipeline.set(getPipeline(Cam.LEFT));
    }

    // Overridden periodic method for logging during each robot loop iteration
    @Override
    public void periodic() {
        logging();
        // manualPose();

        Optional<EstimatedRobotPose> estimatedPoseLeft = photonPoseEstimatorLeft.update();
        SmartDashboard.putBoolean("left photon present",
                estimatedPoseLeft.isPresent());
        if (estimatedPoseLeft.isPresent()) {

            double timestamp = estimatedPoseLeft.get().timestampSeconds;
            Pose2d pose = estimatedPoseLeft.get().estimatedPose.toPose2d();

            Robot.swerve.visionUpdateOdometry(new LimelightPose(pose, timestamp));
            SmartDashboard.putNumberArray("Left Limelight estimator Odometry",
                    new double[] { estimatedPoseLeft.get().estimatedPose.getX(),
                            estimatedPoseLeft.get().estimatedPose.getY(),
                            estimatedPoseLeft.get().estimatedPose.toPose2d().getRotation().getDegrees()
                    });
        }

        System.out.println("right detect");

    }

    public void manualPose() {

        PhotonPipelineResult latestResultLeft = getLatestResult(Cam.LEFT);
        SmartDashboard.putBoolean(" left present", latestResultLeft.hasTargets());

        if (latestResultLeft.hasTargets()) {
            MultiTargetPNPResult multiResultLeft = latestResultLeft.getMultiTagResult();

            if (multiResultLeft.estimatedPose.isPresent) {
                try {
                    SmartDashboard.putNumber("left ambiguity", multiResultLeft.estimatedPose.ambiguity);

                    if (multiResultLeft.estimatedPose.ambiguity <= 0.2) {
                        // Pose3d estimatedPose = VisionConstants.LEFT_CAM_TO_ROBOT
                        //         .transformBy(multiResultLeft.estimatedPose.best);
                        // estimatedPose2dLeft = estimatedPose.toPose2d();

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

                    // int targetID = bestTarget.getFiducialId();
                    // Optional<Pose3d> targetPoseOptional =
                    // aprilTagFieldLayout.getTagPose(targetID);
                    // if (targetPoseOptional.isPresent()) {
                    // Pose3d targetPose = targetPoseOptional.get();
                    // Pose3d camPose =
                    // targetPose.transformBy(bestTarget.getBestCameraToTarget().inverse());
                    // Pose2d robotPoseEstimate =
                    // camPose.transformBy(VisionConstants.LEFT_CAM_TO_ROBOT).toPose2d();
                    // Robot.swerve.visionUpdateOdometry(
                    // new LimelightHelpers.LimelightPose(robotPoseEstimate, imageCaptureTime));
                    // SmartDashboard.putNumberArray("Left Limelight Odometry",
                    // new double[] { robotPoseEstimate.getX(),
                    // robotPoseEstimate.getY(),
                    // robotPoseEstimate.getRotation().getRadians() });

                    // }
                }
            }
        }

        PhotonPipelineResult latestResultRight = getLatestResult(Cam.RIGHT);
        SmartDashboard.putBoolean(" right present", latestResultRight.hasTargets());

        if (latestResultRight.hasTargets()) {

            MultiTargetPNPResult multiResultRight = latestResultRight.getMultiTagResult();
            if (multiResultRight.estimatedPose.isPresent) {
                // Logging information to SmartDashboard
                // Getting the estimated global pose using Limelight and SwerveDrive pose
                SmartDashboard.putBoolean("Running Limelight Right", true);
                try {
                    SmartDashboard.putNumber(" right ambiguity", multiResultRight.estimatedPose.ambiguity);

                    if (multiResultRight.estimatedPose.ambiguity <= 0.2) {

                        estimatedPose2dRight = new Pose2d(multiResultRight.estimatedPose.best.getX(),
                                multiResultRight.estimatedPose.best.getY(),
                                multiResultRight.estimatedPose.best.getRotation().toRotation2d());
                        estimatedPose2dRight
                                .transformBy(new Transform2d(VisionConstants.RIGHT_CAM_TO_ROBOT.getX(),
                                        VisionConstants.RIGHT_CAM_TO_ROBOT.getY(),
                                        VisionConstants.RIGHT_CAM_TO_ROBOT.getRotation().toRotation2d()));
                        // update swerve pose esimtator
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
                        Pose3d camPose = targetPose.transformBy(bestTarget.getBestCameraToTarget().inverse());
                        Pose2d robotPoseEstimate = camPose.transformBy(VisionConstants.RIGHT_CAM_TO_ROBOT).toPose2d();
                        Robot.swerve.visionUpdateOdometry(
                                new LimelightHelpers.LimelightPose(robotPoseEstimate, imageCaptureTime));
                        // Logging Limelight odometry information to SmartDashboard
                        SmartDashboard.putNumberArray("Right Limelight Odometry",
                                new double[] { robotPoseEstimate.getX(),
                                        robotPoseEstimate.getY(),
                                        robotPoseEstimate.getRotation().getRadians() });
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
}