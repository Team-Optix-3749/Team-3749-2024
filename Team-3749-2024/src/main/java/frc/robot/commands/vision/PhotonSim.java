// Importing necessary libraries
package frc.robot.commands.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
// import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.VisionConstants.Cam;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


// https://docs.google.com/drawings/d/14wvoQdwZa72nxycyzGatSuS-62I1iBWHSTa2HcFA90o/edit
//THIS IS THE CHECKERBOARD LINK
/**
 * Command class for vision-based simulation using Limelight and SwerveDrive
 * 
 * @author Jadon Lee
 */
public class PhotonSim extends Command {

    // Reference  Swerve subsystems
    // private Swerve swerve;

    // Constructor for PhotonSim command
    public PhotonSim(){
        // Initializing Limelight and Swerve subsystem references from Robot class
        // this.swerve = Robot.swerve;

        // Setting Limelight pipeline to AprilTag for pose estimation
        Robot.limelight.setPipeline(Constants.VisionConstants.Pipelines.APRILTAG.index, Cam.LEFT);
        // addRequirements(swerve);
    }
    @Override
    public void initialize(){
        Robot.limelight.targeting = true;
    }
    // Execute method called during command execution
    @Override
    public void execute(){
        // Checking if Limelight has a target in the latest result
        // Get Latest Result should pull from the current timestamp tho it has trouble own slower computers when working with sim
        if (Robot.limelight.getLatestResult(Cam.LEFT).getMultiTagResult().estimatedPose.isPresent){
            // Logging information to SmartDashboard
            // Getting the estimated global pose using Limelight and SwerveDrive pose
            try{
                SmartDashboard.putNumber("Yaw",Robot.limelight.getLatestResult(Cam.LEFT).getBestTarget().getYaw());
                // Robot.limelight.estimatedPose2dLeft = Robot.limelight.getEstimatedGlobalPose(swerve.getPose(), Cam.LEFT).get().estimatedPose.toPose2d();
                // Logging Limelight odometry information to SmartDashboard
                SmartDashboard.putNumberArray("Limelight Odometry", new double[] { Robot.limelight.estimatedPose2dLeft.getX(), Robot.limelight.estimatedPose2dLeft.getY(), Robot.limelight.estimatedPose2dLeft.getRotation().getRadians() });
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
