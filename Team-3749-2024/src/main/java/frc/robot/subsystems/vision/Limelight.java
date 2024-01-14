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
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.utils.Constants.VisionConstants.Node;

/**
 * Encapsulated PhotonCamera object used in posed estimation and alignment
 * 
 * @author Rohin Sood
 */
public class Limelight extends SubsystemBase {
    //Position
    public Pose2d estimatedPose2d = new Pose2d(0,0,new Rotation2d());
    public boolean targeting = false;
    // PhotonCamera instance
    private final PhotonCamera camera = new PhotonCamera("limelight");
    private AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonPoseEstimator photonPoseEstimator;

    // NetworkTables entries for controlling LEDs
    private final NetworkTable photonTable = NetworkTableInstance.getDefault().getTable("photonvision");
    private final NetworkTableEntry ledMode = photonTable.getEntry("ledMode");
    private final NetworkTableEntry ledModeState = photonTable.getEntry("ledModeState");
    private final NetworkTableEntry ledModeRequest = photonTable.getEntry("ledModeRequest");

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
                photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    camera, Constants.VisionConstants.sim_robot_to_cam);
            }
            else{
                photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    camera, Constants.VisionConstants.robot_to_cam);
            }
            
        } catch (Exception e) {
            // Handling exceptions during initialization
            System.out.println(e);
        }

        // Setting LED to Off and starting the timer
        setLED(VisionLEDMode.kOff);
        timer = new Timer();
        timer.start();
    }

    // Method to get the latest PhotonPipelineResult from the camera
    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
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
        return photonPoseEstimator;
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
                    Constants.VisionConstants.sim_camera_height,
                    node.height_meters,
                    Constants.VisionConstants.camera_pitch,
                    Units.degreesToRadians(getPitch(target)));
        }
        else{
            return PhotonUtils.calculateDistanceToTargetMeters(
                    Constants.VisionConstants.camera_height,
                    node.height_meters,
                    Constants.VisionConstants.camera_pitch,
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
    public int getPipeline() {
        return camera.getPipelineIndex();
    }

    // Setter for the camera pipeline index
    public void setPipeline(int index) {
        camera.setPipelineIndex(index);
    }

    // Method to set the LED mode for the Limelight
    public void setLED(VisionLEDMode ledMode) {
        switch (ledMode) {
            case kOn:
                this.ledMode.setInteger(1);
                ledModeState.setInteger(1);
                ledModeRequest.setInteger(1);
                break;
            case kOff:
                this.ledMode.setInteger(0);
                ledModeState.setInteger(0);
                ledModeRequest.setInteger(0);
                break;
            case kBlink:
                this.ledMode.setInteger(2);
                ledModeState.setInteger(2);
                ledModeRequest.setInteger(2);
                break;
            default:
                this.ledMode.setInteger(-1);
                ledModeState.setInteger(-1);
                ledModeRequest.setInteger(-1);
                break;
        }
        camera.setLED(ledMode);
    }

    // Method to get the estimated global pose using the PhotonPoseEstimator
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    // Method to get the time the Limelight subsystem has been running
    public double getTimeRunning(){
        return timer.get();
    }

    // Method for logging information
    public void logging() {
        pipeline.set(getPipeline());
    }

    // Overridden periodic method for logging during each robot loop iteration
    @Override
    public void periodic() {
        logging();
    }
}
