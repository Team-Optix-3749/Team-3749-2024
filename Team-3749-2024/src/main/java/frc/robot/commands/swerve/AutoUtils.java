package frc.robot.commands.swerve;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.utils.AutoConstants;
import frc.robot.utils.MiscConstants;
import frc.robot.utils.SuperStructureStates;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

public class AutoUtils {

  private static Swerve swerve = Robot.swerve;
  public static Consumer<Pose2d> pathTargetPose = pose -> swerve.logDesiredOdometry(pose);

  static SendableChooser<Command> autoChooser;
  static SendableChooser<Alliance> allianceChooser;

  public static void initPPUtils() {
    PathPlannerLogging.setLogTargetPoseCallback(pathTargetPose);

    AutoBuilder.configureHolonomic(
        swerve::getPose,
        (Pose2d pose) -> {
        },
        swerve::getChassisSpeeds,
        swerve::setChassisSpeeds,
        AutoConstants.cfgHolonomicFollower,
        MiscConstants::isRedAlliance,
        swerve);
    autoChooser = AutoBuilder.buildAutoChooser("Test");

  }

  public static void initAuto() {
    initPPUtils();
    // NamedCommands.registerCommands(commandList);
  }

  public static Command getAutoPath() {
    return autoChooser.getSelected().andThen(() -> swerve.stopModules());
  }

  public static Command getAutoPath(String autoPathName) {
    Command path = AutoBuilder.buildAuto(autoPathName);
    return path.andThen(() -> swerve.stopModules());
  }

  public static Command getAutoPath(String autoPathName, Pose2d startingPose) {
    Robot.swerve.resetOdometry(startingPose);
    Command path = AutoBuilder.buildAuto(autoPathName);
    return path.andThen(() -> swerve.stopModules());
  }

  public static Command getChoreoAutoPath(
      String autoPathName,
      Pose2d startingPose) {
    Pose2d fieldStartingPose = startingPose;

    if (MiscConstants.isRedAlliance()) {
      SmartDashboard.putNumber("pose x before", fieldStartingPose.getX());

      fieldStartingPose = GeometryUtil.flipFieldPose(fieldStartingPose);

      SmartDashboard.putNumber("pose x after", fieldStartingPose.getX());
      SmartDashboard.putNumber("pose theta after", fieldStartingPose.getRotation().getDegrees());

    }

    Robot.swerve.resetOdometry(fieldStartingPose);
    PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(autoPathName);
    Command cmd = AutoBuilder.followPath(path);
    return cmd.andThen(() -> swerve.stopModules());
  }

  public static Command followPathCommand(PathPlannerPath path) {
    return new FollowPathHolonomic(
        path,
        swerve::getPose,
        swerve::getChassisSpeeds,
        swerve::setChassisSpeeds,
        AutoConstants.cfgHolonomicFollower,
        MiscConstants::isRedAlliance,
        swerve);
  }

  public static Command getPathFindToPoseCommand(
      Pose2d targetPose,
      PathConstraints constraints,
      double endingVelocity) {
    return AutoBuilder.pathfindToPose(targetPose, constraints, endingVelocity);
  }

  public static Command pathFindToThenFollowTraj(
      String trajName,
      PathConstraints constraints) {
    ChoreoTrajectory traj = AutoUtils.getTraj(trajName);
    PathPlannerPath ppPath = PathPlannerPath.fromChoreoTrajectory(trajName);

    // Note from Neel --
    // Should I try to find the direction the robot is heading in and
    // calculate the deltas so that the ending velocity passed into pathFind will
    // be exactly what the inital state will sxet the speeds to.

    Command returnCommand = getPathFindToPoseCommand(
        traj.getInitialPose(),
        constraints,
        0);
    Command pathCommand = followPathCommand(ppPath);

    return returnCommand.andThen(pathCommand);
  }

  public static ChoreoTrajectory getTraj(String trajName) {
    return Choreo.getTrajectory(trajName);
  }

  public static Command timeCommand(Command cmd) {
    Timer timer = new Timer();

    return cmd
        .beforeStarting(() -> timer.start())
        .andThen(() -> {
          timer.stop();
          System.out.println(timer.get());
        });
  }

  public static Command getTroll() {
    return new SequentialCommandGroup(
        Commands.runOnce(() -> Robot.shooter.setState(ShooterStates.TROLL)));
  }

  public static Command getCycle(double wait) {
    return new SequentialCommandGroup(
        Commands.print("cycle"),
        new WaitCommand(wait),
        new SequentialCommandGroup(
            Commands.runOnce(() -> Robot.state = SuperStructureStates.SUBWOOFER),
            new WaitCommand(2.25),
            Commands.runOnce(() -> Robot.intake.setState(IntakeStates.FEED)),
            new WaitCommand(0.25),
            Commands.runOnce(() -> Robot.intake.setState(IntakeStates.INTAKE)),
            Commands.runOnce(() -> Robot.shooter.setState(ShooterStates.INTAKE)),
            Commands.runOnce(() -> Robot.state = SuperStructureStates.GROUND_INTAKE)));
  }

  public static Command getShoot(double wait) {
    System.out.println("subwoofer");
    return new SequentialCommandGroup(
        new WaitCommand(wait),
        Commands.runOnce(() -> Robot.state = SuperStructureStates.AIMBOT),
        getFeed(0.85));
  }
    public static Command getFirstShot(double wait) {
    System.out.println("subwoofer");
    return new SequentialCommandGroup(
        new WaitCommand(wait),
        Commands.runOnce(() -> Robot.state = SuperStructureStates.SUBWOOFER),
        getFeed(1.1));
  }

  public static Command getPodiumShot(double wait) {
    System.out.println("podium");
    return new SequentialCommandGroup(
        new WaitCommand(wait),

        Commands.runOnce(() -> Robot.state = SuperStructureStates.PODIUM),
        getFeed(2));
  }

  public static Command getFeed(double wait) {
    System.out.println("feed");
    return new SequentialCommandGroup(
        new WaitCommand(wait),
        Commands.runOnce(() -> Robot.intake.setState(IntakeStates.FEED)));
  }

  public static Command getintake(double wait) {
    System.out.println("intake");
    return new SequentialCommandGroup(
        new WaitCommand(wait),
        Commands.runOnce(() -> Robot.state = SuperStructureStates.GROUND_INTAKE),
        Commands.runOnce(() -> Robot.intake.setState(IntakeStates.INTAKE)),
        Commands.runOnce(() -> Robot.shooter.setState(ShooterStates.INTAKE)));
  }

  public static Command getStow(double wait) {
    System.out.println("stow");
    return new SequentialCommandGroup(
        new WaitCommand(wait),
        Commands.runOnce(() -> Robot.state = SuperStructureStates.STOW),
        Commands.runOnce(() -> Robot.intake.setState(IntakeStates.STOP), Robot.intake),
        Commands.runOnce(() -> Robot.shooter.setState(ShooterStates.STOP), Robot.shooter));
  }
  public static Command getStopVision(double wait){
    System.out.println("stop vision");
    return Commands.run(() -> Robot.swerve.setUtilizeVision(false));

  }
    public static Command getStartVision(double wait){
    System.out.println("start vision");
    return Commands.run(() -> Robot.swerve.setUtilizeVision(true));

  }
}
