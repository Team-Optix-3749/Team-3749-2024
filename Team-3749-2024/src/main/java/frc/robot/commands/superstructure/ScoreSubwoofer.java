package frc.robot.commands.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSim;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.led.LEDConstants.LEDPattern;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristConstants.WristStates;
import frc.robot.utils.UtilityFunctions;

public class ScoreSubwoofer implements SuperStructureCommandInterface {
    private boolean fullDeployedWrist = false;
    private boolean almostDeployedWrist = false;
    private boolean subwoofedArm = false;
    private boolean staticWrist = false;

    public ScoreSubwoofer() {
    }

    @Override
    public void execute() {

        if (Robot.wrist.getState() == WristStates.FULL_DEPLOYED) {
            fullDeployedWrist = true;
        }
      
        if (Robot.arm.getState() == ArmStates.SUBWOOFER) {
            subwoofedArm = true;
        }

        Robot.wrist.moveWristToGoal();

        Robot.arm.moveToGoal();

        if (subwoofedArm && fullDeployedWrist&& Robot.shooter.getVelocityRadPerSec()>540){
            Robot.led.setLEDPattern(LEDPattern.BLUE);
        }
    }

    @Override
    public void start(){
        Robot.shooter.setState(ShooterStates.SPOOL);
        Robot.intake.setState(IntakeStates.STOP);
        Robot.arm.setGoal(ArmStates.SUBWOOFER);
        Robot.wrist.setGoal(WristStates.FULL_DEPLOYED);

    }

    @Override
    public void reset() {
        Robot.intake.setState(IntakeStates.STOP);
        Robot.shooter.setState(ShooterStates.STOP);
        Robot.wrist.setVoltage(0);
        Robot.led.setLEDPattern(LEDPattern.WHITE);

        fullDeployedWrist = false;
        staticWrist = false;
        almostDeployedWrist = false;
        subwoofedArm = false;
    }

    private boolean atIntake = false;

    @Override
    public void autoExecute() {
    
        if (Robot.wrist.getState() == WristStates.FULL_DEPLOYED) {
            fullDeployedWrist = true;
        }
      
        if (Robot.arm.getState() == ArmStates.SUBWOOFER) {
            subwoofedArm = true;
        }

        Robot.wrist.moveWristToGoal();

        Robot.arm.moveToGoal();

        if (subwoofedArm && fullDeployedWrist&& Robot.shooter.getVelocityRadPerSec()>540){
            Robot.led.setLEDPattern(LEDPattern.BLUE);
        }
    }

    @Override
    public void autoStart(){
        Robot.shooter.setState(ShooterStates.SPOOL);
        Robot.intake.setState(IntakeStates.STOP);
        Robot.arm.setGoal(Units.degreesToRadians(11.5));
        Robot.wrist.setGoal(WristStates.FULL_DEPLOYED);    }

    @Override
    public void autoReset() {
        reset();
        // Robot.intake.setState(IntakeStates.INTAKE);

        atIntake = false;
    }
}
