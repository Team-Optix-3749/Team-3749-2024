// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.JoystickIO;
import frc.robot.utils.Xbox;

public class RobotContainer {

  private Xbox pilot = new Xbox(0);
  private Xbox operator = new Xbox(1);
  private final JoystickIO joystickIO = new JoystickIO(pilot, operator);

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    DriverStation.removeRefreshedDataEventHandle(44000);

    configureBindings();

    // DataLogManager.start("logs");
    // DataLogManager.logNetworkTables(true);
    // DriverStation.startDataLog(DataLogManager.getLog(), true);

    RobotController.setBrownoutVoltage(7.0);
    // Robot.swerve.resetOdometry(new Pose2d(1, 1, new Rotation2d(Math.PI * 11/6)));
  }

  private void configureBindings() {
    joystickIO.getButtonBindings();

  }

  public Command getAutonomousCommand() {

    // Command command = new MoveToPose(new Pose2d(5, 7, new Rotation2d(Math.PI /
    // 2)));
    // Command command = new TurnToAngle(new Rotation2d(Math.PI / 2));
    String[] strList = { "CirclePath", "SquigglyPath" };

    // Command command = FollowPath.followPath("CirclePath").andThen(FollowPath.followPath("SquigglyPath"));
    return null;
  }
}
