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

    private final PhotonCamera camera = new PhotonCamera("limelight");
    private AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonPoseEstimator photonPoseEstimator;

    private final NetworkTable photonTable = NetworkTableInstance.getDefault().getTable("photonvision");
    private final NetworkTableEntry ledMode = photonTable.getEntry("ledMode");
    private final NetworkTableEntry ledModeState = photonTable.getEntry("ledModeState");
    private final NetworkTableEntry ledModeRequest = photonTable.getEntry("ledModeRequest");

    // private final ShuffleData<Boolean> targetFound = new ShuffleData<Boolean>("Limelight", "Target Found", false);
    
    // private final ShuffleData<Double> targetPitch = new ShuffleData<Double>("Limelight", "Target Pitch", 0.0);
    // private final ShuffleData<Double> targetYaw = new ShuffleData<Double>("Limelight", "Target Yaw", 0.0);
    private final ShuffleData<Integer> pipeline = new ShuffleData<Integer>("Limelight",
    "Pipeline", -1000);

    // private final ShuffleData<Integer> aprilTagID = new ShuffleData<Integer>("Limelight", "Fiducial ID", -1000);
    // private final ShuffleData<Double> aprilTagX = new ShuffleData<Double>("Limelight", "AprilTag Y", -1000.0);
    // private final ShuffleData<Double> aprilTagY = new ShuffleData<Double>("Limelight", "AprilTag X", -1000.0);

    // private final ShuffleData<Double> targetTransX = new ShuffleData<Double>("Limelight", "Target Trans X", -1000.0);
    // private final ShuffleData<Double> targetTransY = new ShuffleData<Double>("Limelight", "Target Trans Y", -1000.0);
    private final Timer timer;
    public Limelight() {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            if (Constants.ROBOT_TYPE == RobotType.SIM){
                photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    camera, Constants.VisionConstants.sim_robot_to_cam);
            }
            else{
                photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    camera, Constants.VisionConstants.robot_to_cam);
            }
            
        } catch (Exception e) {
            System.out.println(e);
        }

        setLED(VisionLEDMode.kOff);
        timer = new Timer();
        timer.start();
    }

    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    public boolean hasTarget(PhotonPipelineResult result) {
        SmartDashboard.putBoolean("Has Target", result.hasTargets());
        return result.hasTargets();
    }

    public List<PhotonTrackedTarget> getTargets(PhotonPipelineResult result) {
        return result.getTargets();
    }

    public PhotonTrackedTarget getBestTarget(PhotonPipelineResult result) {
        return result.getBestTarget();
    }

    public Rotation2d getYaw(PhotonTrackedTarget target) {
        return new Rotation2d(Math.toRadians(target.getYaw()));
    }

    public double getPitch(PhotonTrackedTarget target) {
        return target.getPitch();
    }

    public double getArea(PhotonTrackedTarget target) {
        return target.getArea();
    }

    public double getSkew(PhotonTrackedTarget target) {
        return target.getSkew();
    }

    public List<TargetCorner> getBoundingCorners(PhotonTrackedTarget target) {
        return target.getDetectedCorners();
    }

    public int getTargetId(PhotonTrackedTarget target) {
        return target.getFiducialId();
    }

    public double getPoseAbmiguity(PhotonTrackedTarget target) {
        return target.getPoseAmbiguity();
    }

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

    public Translation2d getTranslation2d(PhotonTrackedTarget target, Node node) {
        return PhotonUtils.estimateCameraToTargetTranslation(
                getDistance(target, node), getYaw(target));
    }

    public AprilTagFieldLayout getAprilTagFieldLayout() {
        return aprilTagFieldLayout;
    }

    public int getPipeline() {
        return camera.getPipelineIndex();
    }

    public void setPipeline(int index) {
        camera.setPipelineIndex(index);
    }

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

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
    
    public double getTimeRunning(){
        return timer.get();
    }

    public void logging() {
        
        pipeline.set(getPipeline());

    }

    @Override
    public void periodic() {

        logging();
    }

}