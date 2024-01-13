// Importing necessary libraries
package frc.robot.commands.vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.utils.Constants;

// Command class for vision-based simulation using Limelight and SwerveDrive
public class PhotonSim extends Command {

    // Reference to Limelight and Swerve subsystems
    private Limelight limelight;
    private Swerve swerve;

    // Pose2d to store the estimated global pose
    Pose2d estimatedPose2d;

    // Constructor for PhotonSim command
    public PhotonSim(){
        // Initializing Limelight and Swerve subsystem references from Robot class
        this.limelight = Robot.limelight;
        this.swerve = Robot.swerve;

        // Setting Limelight pipeline to AprilTag for pose estimation
        limelight.setPipeline(Constants.VisionConstants.Pipelines.APRILTAG.index);
    }

    // Execute method called during command execution
    @Override
    public void execute(){
        // Checking if Limelight has a target in the latest result
        if (limelight.getEstimatedGlobalPose(swerve.getPose()).isPresent()){
            // Logging information to SmartDashboard
            SmartDashboard.putNumber("AHHH", 0);

            // Getting the estimated global pose using Limelight and SwerveDrive pose
            estimatedPose2d = limelight.getEstimatedGlobalPose(swerve.getPose()).get().estimatedPose.toPose2d();

            // Logging Limelight odometry information to SmartDashboard
            SmartDashboard.putNumberArray("Limelight Odometry", new double[] { estimatedPose2d.getX(), estimatedPose2d.getY(), estimatedPose2d.getRotation().getDegrees() });
        }
    }
}
