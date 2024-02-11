// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.photo.Photo;

import edu.wpi.first.networktables.NetworkTableInstance;
import java.io.FileNotFoundException;
import java.io.IOException;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.vision.PhotonSim;
// import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Limelight;
import java.nio.file.Path;
import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.swerve.MoveToPose;
import frc.robot.commands.swerve.Teleop;
import frc.robot.commands.swerve.TurnToAngle;
import frc.robot.utils.Constants;
import frc.robot.utils.JoystickIO;
import frc.robot.utils.Xbox;
import frc.robot.utils.Constants.DriveConstants;


public class RobotContainer {
  private Xbox pilot = new Xbox(0);
  private Xbox operator = new Xbox(1);
  private final JoystickIO joystickIO = new JoystickIO(pilot, operator);
  private PhotonSim photonSim = new PhotonSim();



  public RobotContainer() {
    if(Robot.isSimulation()) {
      NetworkTableInstance inst = NetworkTableInstance.getDefault();
      inst.stopServer();
      // Change the IP address in the below function to the IP address you use to connect to the PhotonVision UI.
      inst.setServer("127.0.0.1");
      inst.startClient4("Robot Simulation");
    }
    DriverStation.silenceJoystickConnectionWarning(true);
    DriverStation.removeRefreshedDataEventHandle(44000);

    configureBindings();

    RobotController.setBrownoutVoltage(7.0);

    SmartDashboard.putData("Toggle PhotonVision", photonSim);
    CommandScheduler.getInstance().schedule(photonSim);
    Robot.swerve.resetOdometry(DriveConstants.fieldStartingPose);
    Robot.swerve.setDefaultCommand(new Teleop(pilot::getLeftX, pilot::getLeftY, pilot::getRightX, pilot::getRightY));
  }

  private void configureBindings() {
    joystickIO.getButtonBindings();

  }

  public Command getAutonomousCommand() {
    // return Commands.run(() -> Robot.arm.setVoltage(8-0.973));
    return Commands.run(() -> Robot.arm.setGoal(Units.degreesToRadians(90)));
  }


}
