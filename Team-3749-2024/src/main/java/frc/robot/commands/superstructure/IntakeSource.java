package frc.robot.commands.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristConstants.WristStates;
import frc.robot.utils.SuperStructureStates;
import frc.robot.utils.UtilityFunctions;

public class IntakeSource implements SuperStructureCommandInterface {


    public IntakeSource() {
    }

    @Override
    public void execute() {
        Robot.arm.moveToGoal();
        Robot.wrist.moveWristToGoal();

    }

    @Override
    public void reset() {
        Robot.intake.stop();
        Robot.shooter.stop();
        Robot.wrist.setVoltage(0);

    }

    @Override
    public void start() {
        Robot.wrist.setGoal(WristStates.STOW);
        Robot.arm.setGoal(ArmStates.SOURCE);
        Robot.intake.setState(IntakeStates.INTAKE);
        Robot.shooter.setState(ShooterStates.INTAKE);

    }

}
