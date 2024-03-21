package frc.robot.commands.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristConstants.WristStates;
import frc.robot.utils.SuperStructureStates;
import frc.robot.utils.UtilityFunctions;

public class ScoreAmp implements SuperStructureCommandInterface {
    private boolean fullDeployedWrist = false;
    private boolean almostDeployedWrist = false;
    private boolean ampedArm = false;
    private boolean stowedArm = false;
    private boolean staticWrist = false;

    public ScoreAmp() {
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

        fullDeployedWrist = false;
        staticWrist = false;
        almostDeployedWrist = false;
        ampedArm = false;
        stowedArm = false;
    }

    @Override
    public void start() {
        Robot.wrist.setGoal(WristStates.FULL_DEPLOYED);
        Robot.arm.setGoal(ArmStates.AMP);

    }

}
