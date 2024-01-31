// Importing necessary libraries
package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.Constants.RobotType;
import frc.robot.utils.Constants.VisionConstants.Cam;
import frc.robot.utils.Constants.VisionConstants.Node;

/**
 * Encapsulated PhotonCamera object used in posed estimation and alignment
 * 
 * @author Rohin Sood
 * @author Jadon Lee
 */
public class Limelight extends SubsystemBase {
    //Position
    public Pose2d estimatedPose2dLeft = new Pose2d(0,0,new Rotation2d());
    public Pose2d estimatedPose2dRight = new Pose2d(0,0,new Rotation2d());

    public boolean targeting = false;
    // PhotonCamera instance
    private final PhotonCamera cameraLeft = new PhotonCamera("limelightLeft");
    private final PhotonCamera cameraRight = new PhotonCamera("limelightRight");
    private final PhotonCamera cameraBack = new PhotonCamera("limelightBack");


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
    "Pipeline", -1000);

    // Timer for tracking how long the Limelight subsystem has been running
    private final Timer timer;

    // Constructor
    public Limelight() {
        try {
            // Loading AprilTag field layout
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            
            // Initializing PhotonPoseEstimator based on robot type
            if (Constants.ROBOT_TYPE == RobotType.SIM){
                photonPoseEstimatorLeft = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    cameraLeft, Constants.VisionConstants.SIM_LEFT_ROBOT_TO_CAM);
                photonPoseEstimatorRight = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    cameraRight, Constants.VisionConstants.SIM_RIGHT_ROBOT_TO_CAM);
            }
            else{
                photonPoseEstimatorLeft = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    cameraLeft, Constants.VisionConstants.ROBOT_LEFT_TO_CAM);
                photonPoseEstimatorRight = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    cameraRight, Constants.VisionConstants.ROBOT_RIGHT_TO_CAM);
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

        timer = new Timer();
        timer.start();
    }

    // Method to get the latest PhotonPipelineResult from the camera
    public PhotonPipelineResult getLatestResult(Cam camPose) {
        switch (camPose){
            case LEFT:
                return cameraLeft.getLatestResult();
            case RIGHT:
                return cameraRight.getLatestResult();
            case BACK:
                return cameraBack.getLatestResult();
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
    public PhotonPoseEstimator getPoseEstimator(){
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

    // Method to calculate the distance to a target based on its pitch and camera parameters
    public double getDistance(PhotonTrackedTarget target, Node node) {
        if (Constants.ROBOT_TYPE == RobotType.SIM){
            return PhotonUtils.calculateDistanceToTargetMeters(
                    Constants.VisionConstants.SIM_CAM_HEIGHT,
                    node.height_meters,
                    Constants.VisionConstants.CAM_PITCH,
                    Units.degreesToRadians(getPitch(target)));
        }
        else{
            return PhotonUtils.calculateDistanceToTargetMeters(
                    Constants.VisionConstants.CAM_HEIGHT,
                    node.height_meters,
                    Constants.VisionConstants.CAM_PITCH,
                    Units.degreesToRadians(getPitch(target)));
        }
    }

    // Method to estimate the translation from the camera to a target
    public Translation2d getTranslation2d(PhotonTrackedTarget target, Node node) {
        return PhotonUtils.estimateCameraToTargetTranslation(
                getDistance(target, node), getYaw(target));
    }

    // Getter for AprilTagFieldLayout
    public AprilTagFieldLayout getAprilTagFieldLayout() {
        return aprilTagFieldLayout;
    }

    // Getter for the current camera pipeline index
    public int getPipeline(Cam camPose) {
        switch (camPose){
            case LEFT:
                return cameraLeft.getPipelineIndex();
            case RIGHT:
                return cameraRight.getPipelineIndex();
            case BACK:
                return cameraBack.getPipelineIndex();
        }
        return 0;
        
    }

    // Setter for the camera pipeline index
    public void setPipeline(int index, Cam camPose) {
        switch (camPose){
            case LEFT:
                cameraLeft.setPipelineIndex(index);
            case RIGHT:
                cameraRight.setPipelineIndex(index);
            case BACK:
                cameraBack.setPipelineIndex(index);
        }
    }

    // Method to set the LED mode for the Limelight
    public void setLED(VisionLEDMode ledMode, Cam camPose) {
        switch (camPose){
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
            case BACK:
                switch (ledMode) {
                    case kOn:
                        this.ledModeBack.setInteger(1);
                        ledModeStateBack.setInteger(1);
                        ledModeRequestBack.setInteger(1);
                        break;
                    case kOff:
                        this.ledModeBack.setInteger(0);
                        ledModeStateBack.setInteger(0);
                        ledModeRequestBack.setInteger(0);
                        break;
                    case kBlink:
                        this.ledModeBack.setInteger(2);
                        ledModeStateBack.setInteger(2);
                        ledModeRequestBack.setInteger(2);
                        break;
                    default:
                        this.ledModeBack.setInteger(-1);
                        ledModeStateBack.setInteger(-1);
                        ledModeRequestBack.setInteger(-1);
                        break;
                }
                cameraBack.setLED(ledMode);
        }

    }

    // Method to get the estimated global pose using the PhotonPoseEstimator
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose, Cam camNum) {
        switch (camNum){
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

    // Method to get the time the Limelight subsystem has been running
    public double getTimeRunning(){
        return timer.get();
    }

    // Method for logging information
    public void logging() {
        pipeline.set(getPipeline(Cam.LEFT));
    }
    
    // Overridden periodic method for logging during each robot loop iteration
    @Override
    public void periodic() {
        logging();
    }
    //Thanks to FRC Team 5712
    public Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation) {
        double smallestDistance = Double.POSITIVE_INFINITY;
        for (var target : estimation.targetsUsed) {
            var t3d = target.getBestCameraToTarget();
            var distance = Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
            if (distance < smallestDistance)
                smallestDistance = distance;
        }
        double poseAmbiguityFactor = estimation.targetsUsed.size() != 1
            ? 1
            : Math.max(
                1,
                (estimation.targetsUsed.get(0).getPoseAmbiguity()
                    + Constants.VisionConstants.POSE_AMBIGUITY_SHIFTER)
                    * Constants.VisionConstants.POSE_AMBIGUITY_MULTIPLIER);
        double confidenceMultiplier = Math.max(
            1,
            (Math.max(
                1,
                Math.max(0, smallestDistance - Constants.VisionConstants.NOISY_DISTANCE_METERS)
                    * Constants.VisionConstants.DISTANCE_WEIGHT)
                * poseAmbiguityFactor)
                / (1
                    + ((estimation.targetsUsed.size() - 1) * Constants.VisionConstants.TAG_PRESENCE_WEIGHT)));

        return Constants.VisionConstants.VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(confidenceMultiplier);
    }
}
