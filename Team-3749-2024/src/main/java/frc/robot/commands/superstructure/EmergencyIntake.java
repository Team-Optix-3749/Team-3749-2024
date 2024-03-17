package frc.robot.commands.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.led.LEDConstants.LEDPattern;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristConstants.WristStates;
import frc.robot.utils.SuperStructureStates;

public class EmergencyIntake implements SuperStructureCommandInterface {

    private boolean stowedWrist = false;
    private boolean stowedArm = false;
    private boolean almostDeployedWrist = false;
    private boolean deployedWrist = false;
    private boolean startedRollers = false;
    private boolean uppedArm = false;

    public EmergencyIntake() {
    }

    @Override
    public void execute() {
        if (Robot.wrist.getState() == WristStates.STOW) {
            stowedWrist = true;
        }

        if (Robot.wrist.getState() == WristStates.FULL_DEPLOYED) {
            deployedWrist = true;
        }
        if (Robot.arm.getState() == ArmStates.STOW) {
            stowedArm = true;
        }
        if (stowedArm && Robot.arm.getPositionRad() > Units.degreesToRadians(15)) {
            uppedArm = true;
        }

        if (!stowedWrist && !deployedWrist) {
            Robot.wrist.setGoal(WristStates.STOW);
        }
        if (!stowedArm) {
            Robot.arm.setGoal(ArmStates.STOW);
        }
        if (stowedArm && !uppedArm) {
            Robot.arm.setGoal(Units.degreesToRadians(18.5));
            Robot.wrist.setGoal(WristStates.FULL_DEPLOYED);
        }

        if (deployedWrist) {
            Robot.arm.setGoal(ArmStates.GROUND_INTAKE);

        }

        Robot.arm.moveToGoal();
        Robot.wrist.moveWristToGoal();

        if (Robot.intake.getState() == IntakeStates.INDEX) {
            Robot.state = SuperStructureStates.STOW;
        }

    }

    @Override
    public void reset() {
        stowedWrist = false;
        stowedArm = false;
        almostDeployedWrist = false;
        deployedWrist = false;
        startedRollers = false;
        uppedArm = false;

    }

    private boolean atShoot = false;

    @Override
    public void autoExecute() {
        // execute();
        Robot.arm.setGoal(ArmConstants.groundIntakepositionRad);
        Robot.arm.moveToGoal();
        Robot.wrist.moveWristToGoal();

    }

    @Override
    public void start() {
        Robot.intake.setState(IntakeStates.INTAKE);
        Robot.shooter.setState(ShooterStates.INTAKE);

    }

    @Override
    public void autoReset() {
        reset();
        atShoot = false;
    }

}
