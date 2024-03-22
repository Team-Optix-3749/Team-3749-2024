package frc.robot.commands.superstructure;

import java.sql.Driver;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.wrist.WristConstants.WristStates;
import frc.robot.utils.SuperStructureStates;

public class SuperStructureCommands {

    private EmergencyIntake groundIntake = new EmergencyIntake();
    private Stow stow = new Stow();
    private ScoreAmp scoreAmp = new ScoreAmp();
    private ScoreSubwoofer scoreSubwoofer = new ScoreSubwoofer();
    private Podium podium = new Podium();
    private Reset reset = new Reset();
    private Climb climb = new Climb();
    private ClimbDown climbDown = new ClimbDown();
    private Aimbot aimbot = new Aimbot();
    private IntakeSource source = new IntakeSource();
    private SuperStructureCommandInterface currentCommand = stow;


    public SuperStructureCommands() {
    }

    private void switchCommands(SuperStructureCommandInterface command) {
        // System.out.println("ground intake");
        if (currentCommand != command) {
            if (DriverStation.isAutonomous()) {
                currentCommand.autoReset();
                currentCommand = command;
                currentCommand.autoStart();
                return;
            }
            currentCommand.reset();
            currentCommand = command;
            currentCommand.start();
        }

    }

    public void execute() {
        SmartDashboard.putString("state", Robot.state.name());
        SmartDashboard.putString("current command", currentCommand.getClass().getName());

        switch (Robot.state) {
            case STOW:
                switchCommands(stow);
                break;
            case GROUND_INTAKE:
                switchCommands(groundIntake);
                break;
            case AMP:
                switchCommands(scoreAmp);
                break;
            case SUBWOOFER:
                switchCommands(scoreSubwoofer);
                break;
            case RESET:
                switchCommands(reset);
                break;
            case CLIMB:
                switchCommands(climb);
                break;
            case CLIMBDOWN:
                switchCommands(climbDown);
                break;
            case PODIUM:
                switchCommands(podium);
                break;
            case AIMBOT:
                switchCommands(aimbot);
                break;
            case SOURCE:
                switchCommands(source);
                break;
        }
        if (DriverStation.isAutonomous()) {
            currentCommand.autoExecute();
            return;
        }
        currentCommand.execute();
    }

}
