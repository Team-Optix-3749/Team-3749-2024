package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveModuleIO.ModuleData;
import frc.robot.subsystems.swerve.sim.SwerveModuleSim;
import frc.robot.subsystems.swerve.GyroIO.GyroData;
import frc.robot.utils.Constants;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.Constants.ModuleConstants;

public class SwerveModule {

    private double index;
    private String name;
    private SwerveModuleState desiredState = new SwerveModuleState();
    private final PIDController turningPidController;
    private final PIDController drivingPidController;
    private final SimpleMotorFeedforward drivingFeedFordward;

    private ModuleData moduleData = new ModuleData();
    private SwerveModuleIO moduleIO;

    private ShuffleData<Double> driveSpeed;
    private ShuffleData<Double> drivePosition;
    private ShuffleData<Double> driveTemp;
    private ShuffleData<Double> driveVolts;
    private ShuffleData<Double> driveCurrent;

    private ShuffleData<Double> turningSpeed;
    private ShuffleData<Double> turningPosition;
    private ShuffleData<Double> turningTemp;
    private ShuffleData<Double> turningVolts;
    private ShuffleData<Double> turningCurrent;

    public SwerveModule(int i, SwerveModuleIO SwerveModule) {
        index = i;
        if (index == 0) {
            name = "FL Module";
        } else if (index == 1) {
            name = "FR Module";
        } else if (index == 2) {
            name = "BL Module";
        } else if (index == 3) {
            name = "BR Module";
        }

        if (Robot.isSimulation()) {

            moduleIO = SwerveModule;

            drivingPidController = new PIDController(ModuleConstants.kPDrivingSim, 0, 0);
            drivingFeedFordward = new SimpleMotorFeedforward(ModuleConstants.kSDrivingSim,
                    ModuleConstants.kVDrivingSim);
            turningPidController = new PIDController(ModuleConstants.kPTurningSim, 0, 0);
            turningPidController.enableContinuousInput(0, 2 * Math.PI);

        } else {
            moduleIO = SwerveModule;

            drivingPidController = new PIDController(ModuleConstants.kPDrivingReal, 0, 0);
            drivingFeedFordward = new SimpleMotorFeedforward(ModuleConstants.kSDrivingReal,
                    ModuleConstants.kVDrivingReal);
            turningPidController = new PIDController(ModuleConstants.kPTurningReal, 0, 0);
            turningPidController.enableContinuousInput(0, 2 * Math.PI);

        }
        // Tab, name, data
        driveSpeed = new ShuffleData<>("swerve/" + name, name + " driver speed", moduleData.driveVelocityMPerSec);
        drivePosition = new ShuffleData<>("swerve/" + name, name + " drive position", moduleData.driveVelocityMPerSec);
        driveTemp = new ShuffleData<>("swerve/" + name, name + " drive temp", moduleData.driveVelocityMPerSec);
        driveVolts = new ShuffleData<>("swerve/" + name, name + " drive volts", moduleData.driveVelocityMPerSec);
        driveCurrent = new ShuffleData<>("swerve/" + name, name + " drive current", moduleData.driveVelocityMPerSec);

        turningSpeed = new ShuffleData<>("swerve/" + name, name + " turning speed", moduleData.driveVelocityMPerSec);
        turningPosition = new ShuffleData<>("swerve/" + name, name + " turning position",
                moduleData.driveVelocityMPerSec);
        turningTemp = new ShuffleData<>("swerve/" + name, name + " turning temp", moduleData.driveVelocityMPerSec);
        turningVolts = new ShuffleData<>("swerve/" + name, name + " turning volts", moduleData.driveVelocityMPerSec);
        turningCurrent = new ShuffleData<>("swerve/" + name, name + " turning current", moduleData.turnCurrentAmps);
    }

    public String getName() {
        return name;
    }

    public SwerveModuleState getState() {

        return new SwerveModuleState(
                moduleData.driveVelocityMPerSec,
                new Rotation2d(moduleData.turnAbsolutePositionRad));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(moduleData.drivePositionM, new Rotation2d(moduleData.turnAbsolutePositionRad));
    }

    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    public void setDesiredState(SwerveModuleState state) {

        state = SwerveModuleState.optimize(state, getState().angle);
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            state.speedMetersPerSecond = 0;
        }
        this.desiredState = state;

        double drive_volts = drivingFeedFordward.calculate(state.speedMetersPerSecond)
                + drivingPidController.calculate(moduleData.driveVelocityMPerSec, state.speedMetersPerSecond);

        double turning_volts = turningPidController.calculate(moduleData.turnAbsolutePositionRad,
                state.angle.getRadians());
        // Make a drive PID Controller
        moduleIO.setDriveVoltage(drive_volts);
        moduleIO.setTurnVoltage(turning_volts);

    }

    public void stop() {
        moduleIO.setDriveVoltage(0);
        moduleIO.setTurnVoltage(0);
    }

    // called within the swerve subsystem's periodic
    public void periodic() {
        moduleIO.updateData(moduleData);

        driveSpeed.set(moduleData.driveVelocityMPerSec);
        drivePosition.set(moduleData.drivePositionM);
        driveTemp.set(moduleData.driveTempCelcius);
        driveVolts.set(moduleData.driveAppliedVolts);
        driveCurrent.set(moduleData.driveCurrentAmps);

        turningSpeed.set(moduleData.turnVelocityRadPerSec);
        turningPosition.set(moduleData.turnAbsolutePositionRad);
        turningTemp.set(moduleData.turnTempCelcius);
        turningVolts.set(moduleData.turnAppliedVolts);
        turningCurrent.set(moduleData.turnCurrentAmps);

    }
}
