package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.wrist.WristConstants.WristStates;
import frc.robot.utils.SuperStructureStates;

public class GroundIntake implements SuperStructureCommandInterface {

    private boolean stowedWrist = false;
    private boolean stowedArm = false;
    private boolean almostDeployedWrist = false;
    private boolean deployedWrist = false;
    private boolean groundIntakeArm = false;

    public GroundIntake() {
    }

    @Override
    public void execute() {
        if (Robot.wrist.getState() == WristStates.STOW) {
            stowedWrist = true;
        }
        if (Robot.wrist.getState() == WristStates.ALMOST_DEPLOYED) {
            almostDeployedWrist = true;
        }
        if (Robot.wrist.getState() == WristStates.FULL_DEPLOYED) {
            deployedWrist = true;
        }
        if (Robot.arm.getState() == ArmStates.STOW) {
            stowedArm = true;
        }        
        if (Robot.arm.getState() == ArmStates.GROUND_INTAKE) {
            groundIntakeArm = true;
        }

        
        if (!stowedWrist && !almostDeployedWrist && !deployedWrist) {
            Robot.wrist.setGoal(WristStates.STOW);
        }
        if (!stowedArm) {
            Robot.arm.setGoal(ArmStates.STOW);
        }
        if (stowedWrist && stowedArm) {
            Robot.wrist.setGoal(WristStates.ALMOST_DEPLOYED);
            Robot.intake.setIntakeVelocity(IntakeConstants.intakeVelocityRadPerSec);
        }
        if (almostDeployedWrist){
            Robot.arm.setGoal(ArmStates.GROUND_INTAKE);
        }
        if (groundIntakeArm){
            Robot.wrist.setGoal(WristStates.FULL_DEPLOYED);
        }

        Robot.arm.moveToGoal();
        Robot.wrist.moveWristToGoal();

    }

    @Override
    public void reset() {
        stowedArm = false;
        stowedWrist = false;
        Robot.intake.stop();
    }

}
