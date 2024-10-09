package frc.robot.commands.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.led.LEDConstants.LEDPattern;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristConstants.WristStates;
import frc.robot.utils.UtilityFunctions;

public class Podium implements SuperStructureCommandInterface {
    private boolean fullDeployedWrist = false;
    private boolean almostDeployedWrist = false;
    private boolean podiumedArm = false;
    private boolean staticWrist = false;


    public Podium() {
    }

    @Override
    public void execute() {
        Robot.shooter.setState(ShooterStates.SPOOL);
        Robot.arm.setGoal(ArmStates.PODIUM);
        Robot.wrist.setGoal(WristStates.FULL_DEPLOYED);
        
        if (Robot.wrist.getState() == WristStates.FULL_DEPLOYED) {
            fullDeployedWrist = true;
        }
      
        if (Robot.arm.getState() == ArmStates.PODIUM) {
            podiumedArm = true;
        }

        Robot.wrist.moveWristToGoal();

        Robot.arm.moveToGoal();

        if (podiumedArm && fullDeployedWrist&& Robot.shooter.getVelocityRadPerSec()>540){
            Robot.led.setLEDPattern(LEDPattern.BLUE);
        }

    }

    @Override
    public void reset() {
        Robot.intake.setState(IntakeStates.STOP);
        Robot.shooter.setState(ShooterStates.STOP);
        Robot.led.setLEDPattern(LEDPattern.WHITE);

        Robot.wrist.setVoltage(0);

        fullDeployedWrist = false;
        staticWrist = false;
        almostDeployedWrist = false;
        podiumedArm = false;
        atIntake = false;
    }

    private boolean atIntake = false;

    @Override
    public void autoExecute() {
        Robot.shooter.setState(ShooterStates.SPOOL);
        Robot.arm.setGoal(ArmStates.PODIUM);
        Robot.wrist.setGoal(WristStates.FULL_DEPLOYED);

        if (Robot.wrist.getState() == WristStates.FULL_DEPLOYED) {
            fullDeployedWrist = true;
        }
      
        if (Robot.arm.getState() == ArmStates.PODIUM) {
            podiumedArm = true;
        }


        if (podiumedArm && fullDeployedWrist&& Robot.shooter.getVelocityRadPerSec()>540){
            Robot.led.setLEDPattern(LEDPattern.BLUE);
            Robot.intake.setState(IntakeStates.FEED);
        }

        Robot.wrist.moveWristToGoal();
        Robot.arm.moveToGoal();

    }
    @Override
    public void autoReset() {
        reset();
        atIntake = false;
    }
    @Override
    public void autoStart(){
        start();
    }
}
