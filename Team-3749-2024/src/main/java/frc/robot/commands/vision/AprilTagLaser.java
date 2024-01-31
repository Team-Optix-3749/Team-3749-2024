// Importing necessary libraries
package frc.robot.commands.vision;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.utils.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


// https://docs.google.com/drawings/d/14wvoQdwZa72nxycyzGatSuS-62I1iBWHSTa2HcFA90o/edit
//THIS IS THE CHECKERBOARD LINK

// Command class for vision-based simulation using Limelight and SwerveDrive
public class AprilTagLaser extends Command {

    // Reference  Swerve subsystems
    private Swerve swerve;
    private Pose3d[] visionTargets;

    // Constructor for PhotonSim command
    public AprilTagLaser(){
        visionTargets = new Pose3d[]{};
        // Initializing Limelight and Swerve subsystem references from Robot class
        this.swerve = Robot.swerve;

        // Setting Limelight pipeline to AprilTag for pose estimation
        Robot.limelight.setPipeline(Constants.VisionConstants.Pipelines.APRILTAG.index);
    }
    @Override
    public void initialize(){
        Robot.limelight.targeting = true;
    }
    // Execute method called during command execution
    @Override
    public void execute(){
        // Checking if Limelight has a target in the latest result
        if (Robot.limelight.getLatestResult().getMultiTagResult().estimatedPose.isPresent){
            // Logging information to SmartDashboard

            
            // Getting the estimated global pose using Limelight and SwerveDrive pose
            try{
                for (PhotonTrackedTarget target : Robot.limelight.getLatestResult().getTargets()){
                    // target.
                }
                Robot.limelight.estimatedPose2d = Robot.limelight.getEstimatedGlobalPose(swerve.getPose()).get().estimatedPose.toPose2d();
                // Logging Limelight odometry information to SmartDashboard
                SmartDashboard.putNumberArray("April Tag Points", );
            }
            catch (Exception e){

            }
    }
    @Override
    public void end(boolean interrupted){
        Robot.limelight.targeting = false;
    }
}
