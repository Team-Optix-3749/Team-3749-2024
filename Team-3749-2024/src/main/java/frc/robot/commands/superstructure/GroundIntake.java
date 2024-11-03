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

public class GroundIntake implements SuperStructureCommandInterface {

    private boolean stowedWrist = false;
    private boolean stowedArm = false;
    private boolean almostDeployedWrist = false;
    private boolean deployedWrist = false;
    private boolean startedRollers = false;

    public GroundIntake() {
    }

    @Override
    public void execute() {
        if (Robot.wrist.getState() == WristStates.STOW) {
            stowedWrist = true;
        }
        if ((Robot.wrist.getState() == WristStates.ALMOST_DEPLOYED) ||
                ((Math.abs(Robot.wrist.getVelocityRadPerSec()) < 0.2)
                        && Robot.wrist.getPositionRad() > WristConstants.almostDeployedRad - 0.225)) {
            almostDeployedWrist = true;
        }
        if (Robot.wrist.getState() == WristStates.FULL_DEPLOYED) {
            deployedWrist = true;
        }
        if (Robot.arm.getState() == ArmStates.STOW) {
            stowedArm = true;
        }

        if (!stowedWrist && !almostDeployedWrist && !deployedWrist) {
            Robot.wrist.setGoal(WristStates.STOW);
        }
        if (!stowedArm) {
            Robot.arm.setGoal(ArmStates.STOW);
        }
        if (stowedWrist && stowedArm) {
            Robot.wrist.setGoal(WristStates.ALMOST_DEPLOYED);
            if (!startedRollers) {
                startedRollers = true;
                Robot.intake.setState(IntakeStates.INTAKE);
                Robot.shooter.setState(ShooterStates.INTAKE);
            }
        }
        if (almostDeployedWrist) {
            Robot.arm.setGoal(ArmStates.GROUND_INTAKE);

            Robot.wrist.setGoal(WristStates.FULL_DEPLOYED);
        }

        Robot.arm.moveToGoal();
        Robot.wrist.moveWristToGoal();

        if (Robot.intake.getState() == IntakeStates.INDEX) {
            Robot.wrist.setGoal(WristStates.STOW);
        }

    }

    @Override
    public void reset() {
        stowedWrist = false;
        stowedArm = false;
        almostDeployedWrist = false;
        deployedWrist = false;
        startedRollers = false;
        Robot.intake.stop();
        Robot.shooter.stop();
    }

    private boolean atShoot = false;

    @Override
    public void autoExecute() {
        execute();
        // Robot.arm.setGoal(ArmConstants.groundIntakepositionRad);
        // Robot.intake.setState(IntakeStates.INTAKE);
        // Robot.shooter.setState(ShooterStates.INTAKE);
        // // System.out.println(atShoot);

        // Robot.arm.moveToGoal();
        // Robot.wrist.moveWristToGoal();

    }
    @Override
    public void autoStart(){
        start();
        Robot.intake.setState(IntakeStates.INTAKE);
        Robot.shooter.setState(ShooterStates.INTAKE);
    }
    @Override
    public void autoReset() {
        reset();
        atShoot = false;
    }
    
}
