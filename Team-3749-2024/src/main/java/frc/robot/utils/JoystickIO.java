package frc.robot.utils;

import org.photonvision.estimation.RotTrlTransform3d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.swerve.TeleopJoystickRelative;
import frc.robot.commands.arm.Climb;
import frc.robot.commands.arm.GetConstraints;
import frc.robot.commands.arm.MoveArmToGoal;
import frc.robot.commands.superstructure.GroundIntake;
import frc.robot.commands.swerve.AlignToAmp;
// import frc.robot.commands.arm.ArmMoveToGoal;
import frc.robot.commands.swerve.SwerveTeleop;
import frc.robot.commands.swerve.SwerveTeleopShoot;
import frc.robot.commands.wrist.MoveWristToGoal;
import frc.robot.commands.wrist.getRegressionData;
import frc.robot.subsystems.arm.ArmSim;
import frc.robot.subsystems.arm.ShootKinematics;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
// import frc.robot.commands.swerve.MoveToPose;
// import frc.robot.commands.swerve.Teleop;
// import frc.robot.commands.swerve.TeleopJoystickRelative;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.wrist.WristConstants.WristStates;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Util class for button bindings
 *
 * @author Rohin Sood
 */
public class JoystickIO {

    public JoystickIO() {

    }

    /**
     * Calls binding methods according to the joysticks connected
     */
    public void getButtonBindings() {
        if (DriverStation.isJoystickConnected(1)) {
            // if both xbox controllers are connected
            pilotAndOperatorBindings();
        } else if (DriverStation.isJoystickConnected(0)) {
            // if only one xbox controller is connected
            pilotBindings();
        } else if (Robot.isSimulation()) {
            // will show not connected if on sim
            simBindings();
        } else {
            // if no joysticks are connected (ShuffleBoard buttons)

        }
        setDefaultCommands();
    }

    /**
     * If both controllers are plugged in (pi and op)
     */
    public void pilotAndOperatorBindings() {
        /**
         * Pilot:
         * LT: Ground Intake
         * LB: Outtake
         * RT: Source Intake
         * RB: N/A
         * X: Outtake
         * Y: Pass
         * A: N/A
         * B: N/A
         * Start: Reset Gyro
         * 
         * Operator:
         * LT: Subwoofer
         * LB: Amp
         * RT: Aimbot
         * RB: N/A
         * X: N/A
         * Y: N/A
         * A: N/A
         * B: Shoot
         * Start:
         * Back: Climb
         * DPadDown: Reset
         */


        // Robot.pilot.povLeft().onTrue(Commands.runOnce(() -> Robot.swerve.resetGyro()));

        // gyro
        Robot.pilot.start().onTrue(Commands.runOnce(() -> Robot.swerve.resetGyro()));
        // intake
        Robot.pilot.leftTrigger().onTrue(Commands.runOnce(() -> Robot.state = SuperStructureStates.GROUND_INTAKE))
                .onFalse(Commands.runOnce(() -> {
                    Robot.state = SuperStructureStates.STOW;
                }, Robot.wrist, Robot.intake));

        Robot.pilot.rightTrigger().onTrue(Commands.runOnce(() -> Robot.state = SuperStructureStates.SOURCE))
                .onFalse(Commands.runOnce(() -> {
                    Robot.state = SuperStructureStates.STOW;
                }, Robot.wrist, Robot.intake));
        // outtake
        Robot.pilot.leftBumper()
                .onTrue(Commands.runOnce(() -> Robot.intake.setState(IntakeStates.OUTTAKE), Robot.intake))
                .onFalse(Commands.runOnce(() -> Robot.intake.setState(IntakeStates.STOP), Robot.intake));

        // outtake
        Robot.pilot.x().onTrue(Commands.runOnce(() -> Robot.intake.setState(IntakeStates.OUTTAKE), Robot.intake))
                .onFalse(Commands.runOnce(() -> Robot.intake.setState(IntakeStates.STOP), Robot.intake));
        // pass
        Robot.pilot.y().onTrue(Commands.runOnce(() -> Robot.state = SuperStructureStates.PASS))
                .onFalse(Commands.runOnce(() -> {
                    Robot.state = SuperStructureStates.STOW;
                    Robot.shooter.setState(ShooterStates.STOP);
                }, Robot.wrist, Robot.intake));
        // feed
        Robot.pilot.b().onTrue(Commands.runOnce(() -> Robot.intake.setState(IntakeStates.FEED)))
                .onFalse(Commands.runOnce(() -> {
                    Robot.intake.setState(IntakeStates.STOP);
                    Robot.shooter.setState(ShooterStates.STOP);
                }, Robot.intake));
        
        // OPERATOR
        Robot.operator.rightTrigger().onTrue(Commands.runOnce(() -> Robot.state = SuperStructureStates.AIMBOT))
                .onFalse(Commands.runOnce(() -> {
                    Robot.state = SuperStructureStates.STOW;
                }, Robot.wrist)).whileTrue(new SwerveTeleopShoot(() -> -Robot.pilot.getLeftX(),
                        () -> -Robot.pilot.getLeftY(),
                        () -> -Robot.pilot.getRightX()));

        Robot.operator.leftTrigger().onTrue(Commands.runOnce(() -> Robot.state = SuperStructureStates.SUBWOOFER))
                .onFalse(Commands.runOnce(() -> {
                    Robot.state = SuperStructureStates.STOW;
                }, Robot.wrist));

                // feed
        Robot.operator.b().onTrue(Commands.runOnce(() -> Robot.intake.setState(IntakeStates.FEED)))
                .onFalse(Commands.runOnce(() -> {
                    Robot.intake.setState(IntakeStates.STOP);
                    Robot.shooter.setState(ShooterStates.STOP);
                }, Robot.intake));

        // amp
        // Robot.operator.leftBumper().onTrue(Commands.runOnce(() -> Robot.state = SuperStructureStates.AMP))
        //         .onFalse(Commands.runOnce(() -> {
        //             Robot.state = SuperStructureStates.STOW;
        //         }, Robot.arm, Robot.wrist, Robot.intake, Robot.shooter)).whileTrue(new AlignToAmp());

        Robot.operator.leftBumper().onTrue(Commands.runOnce(() -> Robot.state = SuperStructureStates.AMP))
                .onFalse(Commands.runOnce(() -> {
                    Robot.state = SuperStructureStates.STOW;
                }, Robot.arm, Robot.wrist, Robot.intake, Robot.shooter));


        // feed
        Robot.operator.b().onTrue(Commands.runOnce(() -> Robot.intake.setState(IntakeStates.FEED)))
                .onFalse(Commands.runOnce(() -> {
                    Robot.intake.setState(IntakeStates.STOP);
                    Robot.shooter.setState(ShooterStates.STOP);
                }, Robot.intake));

        Robot.operator.povDown().onTrue(Commands.runOnce(() -> Robot.state = SuperStructureStates.RESET));

        // Robot.operator.rightBumper().onTrue(Commands.runOnce(() ->
        // Robot.shooter.setState(ShooterStates.SPOOL)))
        // .onFalse(Commands.runOnce(() -> Robot.shooter.setState(ShooterStates.STOP)));
        Robot.operator.back().onTrue(Commands.runOnce(() -> Robot.state = SuperStructureStates.CLIMB))
                .onFalse(Commands.runOnce(() -> Robot.state = SuperStructureStates.CLIMBDOWN));
    }

    public void pilotBindings() {
        /// ground intake
        Robot.pilot.leftTrigger().onTrue(Commands.runOnce(() -> Robot.state = SuperStructureStates.GROUND_INTAKE))
                .onFalse(Commands.runOnce(() -> {
                    Robot.state = SuperStructureStates.STOW;
                }, Robot.wrist, Robot.intake));

        // subwoofer
        Robot.pilot.rightTrigger().onTrue(Commands.runOnce(() -> Robot.state = SuperStructureStates.SUBWOOFER))
                .onFalse(Commands.runOnce(() -> {
                    Robot.state = SuperStructureStates.STOW;
                }, Robot.wrist));

        // feed
        Robot.pilot.b().onTrue(Commands.runOnce(() -> Robot.intake.setState(IntakeStates.FEED)))
                .onFalse(Commands.runOnce(() -> {
                    Robot.intake.setState(IntakeStates.STOP);
                    Robot.shooter.setState(ShooterStates.STOP);
                }, Robot.intake));

        // gyro
        Robot.pilot.start().onTrue(Commands.runOnce(() -> Robot.swerve.resetGyro()));

        // re-run intake/indexing scheme
        Robot.pilot.rightBumper().onTrue(Commands.runOnce(() -> {
            Robot.intake.setState(IntakeStates.INTAKE);
            Robot.shooter.setState(ShooterStates.INTAKE);
        }));

        // outtake
        Robot.pilot.x().onTrue(Commands.runOnce(() -> Robot.intake.setState(IntakeStates.OUTTAKE), Robot.intake))
                .onFalse(Commands.runOnce(() -> Robot.intake.setState(IntakeStates.STOP), Robot.intake));

        // amp
        Robot.pilot.a().onTrue(Commands.runOnce(() -> Robot.state = SuperStructureStates.AMP))
                .onFalse(Commands.runOnce(() -> {
                    Robot.state = SuperStructureStates.STOW;
                }, Robot.arm, Robot.wrist, Robot.intake, Robot.shooter));

        Robot.pilot.povDown().onTrue(Commands.runOnce(() -> Robot.state = SuperStructureStates.RESET));

    }

    public void simBindings() {
        // Robot.pilot.aWhileHeld(new MoveToPose(new Pose2d(5, 5, new Rotation2d())));
    }

    /**
     * Sets the default commands
     */
    public void setDefaultCommands() {

        Robot.swerve.setDefaultCommand(
                new SwerveTeleop(
                        () -> -Robot.pilot.getLeftX(),
                        () -> -Robot.pilot.getLeftY(),
                        () -> -Robot.pilot.getRightX()));
    }

}
