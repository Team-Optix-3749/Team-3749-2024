// Importing necessary libraries
package frc.robot.commands.vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.utils.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;




// Command class for vision-based simulation using Limelight and SwerveDrive
public class PhotonSim extends Command {

    // Reference  Swerve subsystems
    private Swerve swerve;

    // Constructor for PhotonSim command
    public PhotonSim(){
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
                Robot.limelight.estimatedPose2d = Robot.limelight.getEstimatedGlobalPose(swerve.getPose()).get().estimatedPose.toPose2d();
                // Logging Limelight odometry information to SmartDashboard
                SmartDashboard.putNumberArray("Limelight Odometry", new double[] { Robot.limelight.estimatedPose2d.getX(), Robot.limelight.estimatedPose2d.getY(), Robot.limelight.estimatedPose2d.getRotation().getRadians() });
            }
            catch (Exception e){

            }
         }
    }
    @Override
    public void end(boolean interrupted){
        Robot.limelight.targeting = false;
    }
}
