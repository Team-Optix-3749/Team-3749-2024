// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.photo.Photo;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.vision.PhotonSim;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.utils.JoystickIO;
import frc.robot.utils.Xbox;

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
  
    DataLogManager.start("logs");
    DataLogManager.logNetworkTables(true);
    DriverStation.startDataLog(DataLogManager.getLog(), true);


    RobotController.setBrownoutVoltage(7.0);

    SmartDashboard.putData("Toggle PhotonVision", photonSim);
    CommandScheduler.getInstance().schedule(photonSim);
  }

  private void configureBindings() {
    joystickIO.getButtonBindings();

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
