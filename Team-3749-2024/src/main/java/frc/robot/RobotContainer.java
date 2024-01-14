// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;
import java.util.HashMap;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryStateBuilder;
import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.path.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.swerve.PPUtils;
import frc.robot.utils.JoystickIO;
import frc.robot.utils.Xbox;
import frc.robot.utils.Constants.DriveConstants;

public class RobotContainer {
  private Xbox pilot = new Xbox(0);
  private Xbox operator = new Xbox(1);
  private final JoystickIO joystickIO = new JoystickIO(pilot, operator);

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    DriverStation.removeRefreshedDataEventHandle(44000);

    initAuto();
    configureBindings();

    RobotController.setBrownoutVoltage(7.0);
    Robot.swerve.resetOdometry(DriveConstants.fieldStartingPose);
  }

  private void configureBindings() {
    joystickIO.getButtonBindings();

  }

  public void initAuto() {
    HashMap<String, Command> commandList = new HashMap<String, Command>();

    commandList.put("PrintCMD-hello", new PrintCommand("hello"));

    PPUtils.initPathCommands(commandList);
    PPUtils.initPPUtils();
  }

  public Command getAutonomousCommand() {
    Command command;
    // command = PPUtils.getPathFindToPoseCommand(new Pose2d(10, 3,
    // Rotation2d.fromDegrees(180)), 0);

    // command = PPUtils.getPathFindThenFollowPathCommand("CirclePath",
    // Constants.PathPlannerConstants.defaultPathConstraints);

    // command = PPUtils.pathFindThenFollowPathCommand("fullaround");
    ChoreoTrajectory traj = Choreo.getTrajectory("fullaround");

    Choreo

    command = PPUtils.getFollowTrajectoryCommand(traj);

    return command;
  }

}
