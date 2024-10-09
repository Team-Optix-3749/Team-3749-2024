// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.commands.superstructure.SuperStructureCommands;
import frc.robot.commands.swerve.AutoUtils;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.swerve.Swerve;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.utils.SuperStructureStates;
import frc.robot.utils.Xbox;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.led.Led;

public class Robot extends TimedRobot {
  public static final Xbox pilot = new Xbox(0);
  public static final Xbox operator = new Xbox(1);

  public static final Swerve swerve = new Swerve();
  public static final Arm arm = new Arm();
  public static final Wrist wrist = new Wrist();
  public static final Intake intake = new Intake();
  public static final Shooter shooter = new Shooter();
  public static Limelight limelight;

  public static SuperStructureStates state = SuperStructureStates.STOW;
  public static SuperStructureCommands centralCommand = new SuperStructureCommands();

  public static final Led led = new Led(0.05);

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    if (Robot.isReal()) {
      limelight = new Limelight();

    } else {
      limelight = null;
    }

    DataLogManager.logNetworkTables(true);
    DataLogManager.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (DriverStation.isEnabled()) {

      centralCommand.execute();
    }

  }

  @Override
  public void disabledInit() {
    wrist.setCoastMode();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    wrist.setBrakeMode();
  AutoUtils.getStartVision(0);
    state = SuperStructureStates.STOW;


  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}