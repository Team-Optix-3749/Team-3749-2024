package frc.robot.commands.vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.utils.Constants;

public class PhotonSim extends Command {

    private Limelight limelight;
    private Swerve swerve;
    Pose2d estimatedPose2d;

    public PhotonSim(){
        this.limelight = Robot.limelight;
        this.swerve = Robot.swerve;
        limelight.setPipeline(Constants.VisionConstants.Pipelines.APRILTAG.index);

    }
    @Override
    public void execute(){
        if (limelight.hasTarget(limelight.getLatestResult())){
            SmartDashboard.putNumber("AHHH", 0);
            estimatedPose2d = limelight.getEstimatedGlobalPose(swerve.getPose()).get().estimatedPose.toPose2d();
            SmartDashboard.putNumberArray("Limelight Odometry", new double[] { estimatedPose2d.getX(), estimatedPose2d.getY(), estimatedPose2d.getRotation().getDegrees() });
        }
    }
}
