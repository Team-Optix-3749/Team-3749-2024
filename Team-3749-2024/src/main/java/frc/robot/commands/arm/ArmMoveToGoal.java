package frc.robot.commands.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.utils.Constants;

public class ArmMoveToGoal extends Command {

    private double prevSetpointVelocity = 0;

    public ArmMoveToGoal() {
        addRequirements(Robot.arm); 

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        State setpoint = Robot.arm.getSetpoint();
        double accelerationSetpoint = (setpoint.velocity - prevSetpointVelocity) / 0.02;
        prevSetpointVelocity = setpoint.velocity;

        Robot.arm.setState(setpoint.position, setpoint.velocity, accelerationSetpoint);

    }

    @Override
    public void end(boolean interupted) {

    }

    @Override
    public boolean isFinished() {
        return false;

    }
}
