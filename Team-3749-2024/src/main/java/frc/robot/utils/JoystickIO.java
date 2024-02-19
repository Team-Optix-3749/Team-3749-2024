package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.WristCommand;
import frc.robot.subsystems.shooter.Shooter;
import java.util.Map;

/**
 * Util class for button bindings
 *
 * @author Rohin Sood
 */
public class JoystickIO {

  private static String[] lastJoystickNames = new String[] {
    "",
    "",
    "",
    "",
    "",
    ""
  };

  private Xbox pilot;
  private Xbox operator;

  public JoystickIO(Xbox pilot, Xbox operator) {
    this.pilot = pilot;
    this.operator = operator;
  }

  public static boolean didJoysticksChange() {
    boolean joysticksChanged = false;
    for (int port = 0; port < DriverStation.kJoystickPorts; port++) {
      String name = DriverStation.getJoystickName(port);
      if (!name.equals(lastJoystickNames[port])) {
        joysticksChanged = true;
        lastJoystickNames[port] = name;
      }
    }
    return joysticksChanged;
  }

  /**
   * Calls binding methods according to the joysticks connected
   */
  public void getButtonBindings() {
    if (DriverStation.isJoystickConnected(1)) {
      // if both xbox controllers are connected
      pilotAndOperatorBindings();
    } else if (DriverStation.isJoystickConnected(0)) {
      // if only one xbox controller is connected
      pilotBindings();
    } else {
      // if no joysticks are connected (ShuffleBoard buttons)
      noJoystickBindings();
    }

    setDefaultCommands();
  }

  /**
   * If both controllers are plugged in (pi and op)
   */
  public void pilotAndOperatorBindings() {
    pilotBindings();
  }

  /**
   * If only one controller is plugged in (pi)
   */
  public void pilotBindings() {
    pilot.aWhileHeld(Robot.shooter.getShooterSysIDDynamicForwardTest());
    pilot.bWhileHeld(Robot.shooter.getShooterSysIDDynamicReverseTest());
    pilot.yWhileHeld(Robot.shooter.getShooterSysIDQuasistaticForwardTest());
    pilot.xWhileHeld(Robot.shooter.getShooterSysIDQuasistaticReverseTest());
  }

  /**
   * If NO joysticks are plugged in (Buttons for commands are runnable in the
   * "Controls" tab in ShuffleBoard)
   */
  public void noJoystickBindings() {
    ShuffleboardTab controlsTab = Shuffleboard.getTab("Controls");

    // Example
    ShuffleboardLayout armCommands = controlsTab
      .getLayout("Arm", BuiltInLayouts.kList)
      .withSize(2, 2)
      .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands
  }

  /**
   * Sets the default commands
   */
  public void setDefaultCommands() {
    //    Robot.wrist.setDefaultCommand(new WristCommand());
    //    Robot.intake.setDefaultCommand(new IntakeCommand());
    //    Robot.shooter.setDefaultCommand(new ShooterCommand());

    //    pilot.leftTrigger().onTrue(Commands.runOnce(() -> Robot.intake.setIntakeVelocity(Constants.ShintakeConstants.intakeVelocity)));
    //    pilot.leftTrigger().onFalse(Commands.runOnce(() -> Robot.intake.setIntakeVelocity(0)));

    //    pilot.rightTrigger().onTrue(Commands.runOnce(() -> Robot.intake.setIntakeVelocity(Constants.ShintakeConstants.outtakeVelocity)));
    //    pilot.rightTrigger().onFalse(Commands.runOnce(() -> Robot.intake.setIntakeVelocity(0)));

    //    pilot.rightBumper().onTrue(Commands.runOnce(() -> Robot.shooter.setShooterVelocity(Constants.ShintakeConstants.shooterVelocity)));
    //    pilot.rightBumper().onFalse(Commands.runOnce(() -> Robot.shooter.setShooterVelocity(0)));

    //    pilot.a().onTrue(Commands.runOnce(() -> Robot.wrist.toggleWristGoal()));

    // Basic Voltage Testing
    pilot.aWhileHeld(
      Commands.run(() -> Robot.wrist.setVoltage(7)),
      Commands.run(() -> Robot.wrist.setVoltage(0))
    );
    pilot.bWhileHeld(
      Commands.run(() -> Robot.wrist.setVoltage(-7)),
      Commands.run(() -> Robot.wrist.setVoltage(0))
    );

    pilot.xWhileHeld(
      Commands.run(() -> Robot.shooter.setVoltage(12)),
      Commands.run(() -> Robot.shooter.setVoltage(0))
    );
    pilot.rightTriggerWhileHeld(
      Commands.run(() -> Robot.intake.setVoltage(-7)),
      Commands.run(() -> Robot.intake.setVoltage(0))
    );
    pilot.leftTriggerWhileHeld(
      Commands.run(() -> Robot.intake.setVoltage(7)),
      Commands.run(() -> Robot.intake.setVoltage(0))
    );
  }
}
