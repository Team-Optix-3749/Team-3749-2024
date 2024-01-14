package frc.robot.commands.swerve;

import java.util.*;
import java.util.function.*;

import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.util.*;
import com.choreo.lib.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.Sim.PIDValues;

public class PPUtils {
  private static Swerve swerve = Robot.swerve;
  public static Consumer<Pose2d> pathTargetPose = pose -> swerve.logDesiredOdometry(pose);

  static SendableChooser<Command> autoChooser;
  static SendableChooser<Alliance> allianceChooser = new SendableChooser<>();
  static boolean isFirstPath = true;

  public static void initPPUtils() {
    PathPlannerLogging.setLogTargetPoseCallback(pathTargetPose);

    allianceChooser.addOption("Blue Alliance", Alliance.Blue);
    allianceChooser.addOption("Red Alliance", Alliance.Red);
    allianceChooser.setDefaultOption("Blue Alliance", Alliance.Blue);

    AutoBuilder.configureHolonomic(
        swerve::getPose,
        swerve::resetOdometry,
        swerve::getChassisSpeeds,
        swerve::setChassisSpeeds,
        Constants.PathPlannerConstants.cfgHolonomicFollower,
        () -> {
          Alliance robotAlliance = allianceChooser.getSelected();
          robotAlliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : robotAlliance;

          if (robotAlliance == Alliance.Red) {
            return true;
          } else {
            return false;
          }
        },
        swerve);

    autoChooser = AutoBuilder.buildAutoChooser("TestAuto");

    SmartDashboard.putData("Choose Auto", autoChooser);
    SmartDashboard.putData("Choose Alliance", allianceChooser);
  }

  public static void initPathCommands(HashMap<String, Command> commandList) {
    commandList.forEach((String cmdName, Command cmd) -> {
      NamedCommands.registerCommand(cmdName, cmd);
    });
  }

  public static Command getFollowTrajectoryCommand(ChoreoTrajectory traj) {
    return Choreo.choreoSwerveCommand(traj, swerve::getPose,
        new PIDController(PIDValues.kP_PathPlannerDrive, 0,
            PIDValues.kD_PathPlannerDrive),
        new PIDController(PIDValues.kP_PathPlannerDrive, 0,
            PIDValues.kD_PathPlannerDrive),
        new PIDController(PIDValues.kP_PathPlannerTurn, 0,
            PIDValues.kD_PathPlannerTurn),
        swerve::setChassisSpeeds,
        () -> {
          Alliance robotAlliance = allianceChooser.getSelected();
          robotAlliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : robotAlliance;

          if (robotAlliance == Alliance.Red) {
            return true;
          } else {
            return false;
          }
        }, swerve);
  }

  public static Command pathFindThenFollowPathCommand(String pathName) {
    ChoreoTrajectory traj = Choreo.getTrajectory(pathName);

    return new SequentialCommandGroup(
        getPathFindToPoseCommand(traj.getInitialPose(), Constants.PathPlannerConstants.defaultPathConstraints),
        getFollowTrajectoryCommand(traj));
  }

  public static Command getAutoPath() {
    return autoChooser.getSelected();
  }

  public static Command getAutoPath(String autoPathName) {

    return AutoBuilder.buildAuto(autoPathName);
  }

  public static Command getPathFindToPoseCommand(Pose2d pose, PathConstraints constraints) {
    return AutoBuilder.pathfindToPose(pose, constraints);
  }

  public static Command getPathFindToPoseCommand(Pose2d targetPose, double endingVelocity) {
    return AutoBuilder.pathfindToPose(targetPose, Constants.PathPlannerConstants.defaultPathConstraints,
        endingVelocity);
  }

  public static Command getPathFindToPoseCommand(Pose2d targetPose, PathConstraints constraints,
      double endingVelocity) {

    return AutoBuilder.pathfindToPose(targetPose, constraints, endingVelocity);
  }
}