// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.swerve.AutoUtils;
import frc.robot.commands.swerve.Autos;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.utils.JoystickIO;
import frc.robot.utils.SuperStructureStates;
import java.util.HashMap;
import java.util.Map;

public class RobotContainer {

  private final JoystickIO joystickIO = new JoystickIO();

  public RobotContainer() {
    // if (Robot.isSimulation()) {
    // NetworkTableInstance inst = NetworkTableInstance.getDefault();
    // inst.stopServer();
    // // Change the IP address in the below function to the IP address you use to
    // // connect to the PhotonVision UI.
    // inst.setServer("127.0.0.1");
    // inst.startClient4("Robot Simulation");
    // }
    DriverStation.silenceJoystickConnectionWarning(true);
    DriverStation.removeRefreshedDataEventHandle(44000);
        initAuto();

    configureBindings();

    // DataLogManager.start(".wpilog");
    // DataLogManager.logNetworkTables(true);
    // DriverStation.startDataLog(DataLogManager.getLog(), true);


    RobotController.setBrownoutVoltage(7.0);
    // Robot.swerve.setDefaultCommand(new Teleop(pilot::getLeftX, pilot::getLeftY,
    // pilot::getRightX, pilot::getRightY));
  }

  private void configureBindings() {
    // joystickIO.pilotBindings();
    joystickIO.pilotAndOperatorBindings();
    joystickIO.setDefaultCommands();
  }

  public void initAuto() {
    System.out.println("Called");

    AutoUtils.initAuto();
  }

  public Command getAutonomousCommand() {
    return Autos.get5Piece();
    // return AutoUtils.getFirstShot(6);

  }
}
