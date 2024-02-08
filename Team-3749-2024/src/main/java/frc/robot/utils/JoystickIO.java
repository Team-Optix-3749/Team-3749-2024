package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.commands.arm.ArmMoveToGoal;
import frc.robot.commands.swerve.MoveToPose;
import frc.robot.commands.swerve.Teleop;
/**
 * Util class for button bindings
 * 
 * @author Rohin Sood
 */
public class JoystickIO {

    private Xbox pilot;
    private Xbox operator; // leave here

    public JoystickIO(Xbox pilot, Xbox operator) {
        this.pilot = pilot;
        this.operator = operator;
    }

    /**
     * Calls binding methods according to the joysticks connected
     */
    public void getButtonBindings() {
        System.out.println(DriverStation.isJoystickConnected(0));

        if (DriverStation.isJoystickConnected(1)) {
            // if both xbox controllers are connected
            pilotAndOperatorBindings();

        } else if (DriverStation.isJoystickConnected(0)) {
            // if only one xbox controller is connected
            pilotBindings();

        } else if (Robot.isSimulation()) {
            // will show not connected if on sim
            simBindings();

        } else {
            // if no joysticks are connected (ShuffleBoard buttons)

        }
        setDefaultCommands();
    }

    /**
     * If both controllers are plugged in (pi and op)
     */
    public void pilotAndOperatorBindings() {

    }

    /**
     * If only one controller is plugged in (pi)
     */

    public void pilotBindings() {
        pilot.aWhileHeld(Commands.run(() -> Robot.example.setVoltage(12)),
                Commands.run(() -> Robot.example.setVoltage(0)));


    }

    public void simBindings() {
        pilot.aWhileHeld(new MoveToPose(new Pose2d(5, 5, new Rotation2d())));
    }

    /**
     * Sets the default commands
     */
    public void setDefaultCommands() {
        Robot.arm.setDefaultCommand(new ArmMoveToGoal());
        Robot.swerve.setDefaultCommand(new Teleop(()->-pilot.getLeftX(), ()->-pilot.getLeftY(), ()->-pilot.getRightX()));

    }

}