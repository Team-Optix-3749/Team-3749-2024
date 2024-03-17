package frc.robot.commands.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.led.LEDConstants.LEDPattern;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristConstants.WristStates;
import frc.robot.utils.SuperStructureStates;
import frc.robot.utils.UtilityFunctions;

public class Stow implements SuperStructureCommandInterface {
    private boolean loweringArm = false;
    private boolean loweredArm = false;
    private boolean armWasRaised = false;
    private boolean stowedWrist = false;

    private Timer timer = new Timer();

    public Stow() {
    }

    private double maxArmRadForStow = Units.degreesToRadians(8);

    @Override
    public void execute() {

        Robot.wrist.setGoal(WristStates.STOW);

        Robot.arm.setGoal(ArmStates.STOW);

        Robot.arm.moveToGoal();
        Robot.wrist.moveWristToGoal();

    }

    @Override
    public void reset() {
        timer.stop();
        timer.reset();
    }

    @Override
    public void start() {
        if (!Robot.intake.getHasPiece()){

            Robot.intake.setState(IntakeStates.STOP);
            Robot.shooter.setState(ShooterStates.STOP);
        }
    }

}
