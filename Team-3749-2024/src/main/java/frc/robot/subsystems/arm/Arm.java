package frc.robot.subsystems.arm;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;
import frc.robot.subsystems.arm.ArmIO.ArmData;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristConstants.WristStates;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.SuperStructureStates;
import frc.robot.utils.UtilityFunctions;

public class Arm extends SubsystemBase {

    private ArmData data = new ArmData();
    private ArmIO armIO;

    private ProfiledPIDController feedback = new ProfiledPIDController(ArmConstants.stowedPID.kP,
            ArmConstants.stowedPID.kI,
            ArmConstants.stowedPID.kD,
            ArmConstants.deployedConstraints);

    private ArmFeedforward stowedFeedforward = new ArmFeedforward(ArmConstants.stowedkS,
            ArmConstants.stowedkG,
            ArmConstants.stowedkV);
    private ArmFeedforward deployedFeedforward = new ArmFeedforward(ArmConstants.deployedkS,
            ArmConstants.deployedkG,
            ArmConstants.deployedkV);

    private Mechanism2d mechanism = new Mechanism2d(2.5, 2);
    private MechanismRoot2d mechanismArmPivot = mechanism.getRoot("mechanism arm pivot", 1, 0.5);
    private MechanismLigament2d mechanismArm = mechanismArmPivot
            .append(new MechanismLigament2d("mechanism arm", .93, 0));
    private ShuffleData<Double> positionLog = new ShuffleData<Double>(this.getName(), "position",
            0.0);
    private ShuffleData<Double> velocityLog = new ShuffleData<Double>(this.getName(), "velocity",
            0.0);
    private ShuffleData<Double> accelerationLog = new ShuffleData<Double>(this.getName(), "acceleration",
            0.0);
    private ShuffleData<Double> voltageLog = new ShuffleData<Double>(this.getName(), "voltage",
            0.0);
    private ShuffleData<Double> leftCurrentLog = new ShuffleData<Double>(this.getName(), "left current",
            0.0);
    private ShuffleData<Double> rightCurrentLog = new ShuffleData<Double>(this.getName(), "right current",
            0.0);
    private ShuffleData<Double> goalLog = new ShuffleData<Double>(this.getName(), "goal",
            0.0);
    private ShuffleData<Double> setpointPositionLog = new ShuffleData<Double>(this.getName(), "setpoint position",
            0.0);
    private ShuffleData<Double> setpointVelocityLog = new ShuffleData<Double>(this.getName(), "setpoint velocity",
            0.0);
    private ShuffleData<Double> setpointAccelerationLog = new ShuffleData<Double>(this.getName(),
            "setpoint acceleration", 0.0);

    private ShuffleData<String> stateLog = new ShuffleData<String>(this.getName(), "state",
            ArmStates.STOW.name());

    private double accelerationSetpoint = 0;
    private double prevSetpointVelocity = 0;

    private boolean isKilled = false;
    private boolean isEnabled = false;

    private boolean deployedMode = false;
    private ArmStates state = ArmStates.STOW;

    public Arm() {
        if (Robot.isSimulation()) {
            armIO = new ArmSim();
        } else {
            armIO = new ArmSparkMax();
        }
        try {
            ShootKinematics.loadDistCSV();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        feedback.setGoal(ArmConstants.stowPositionRad);

    }

    public double getPositionRad() {
        return data.positionRad;
    }

    public double getVelocityRadPerSec() {
        return data.velocityRadPerSec;
    }

    public void setGoal(ArmStates state) {
        if (state == ArmStates.AMP) {
            feedback.setGoal(ArmConstants.ampPositionRad);
        }
        if (state == ArmStates.STOW) {
            feedback.setGoal(ArmConstants.stowPositionRad);
        }
        if (state == ArmStates.CLIMB) {
            feedback.setGoal(ArmConstants.climbPositionRad);
        }
        if (state == ArmStates.GROUND_INTAKE) {
            feedback.setGoal(ArmConstants.groundIntakepositionRad);
        }
        if (state == ArmStates.SUBWOOFER) {
            feedback.setGoal(ArmConstants.subwooferPositionRad);
        }
        if (state == ArmStates.PODIUM) {
            feedback.setGoal(ArmConstants.podiumPositionRad);
        }
        if (state == ArmStates.AIMBOT) {
            double calcedArmAngle = ShootKinematics.getArmAngleRadGivenPose(Robot.swerve.getPose());
            if (calcedArmAngle < 0) {
                calcedArmAngle = Math.PI / 4;
            }
            Robot.arm.setGoal(calcedArmAngle);
        }
    }

    public void setGoal(double goalRad) {
        // state = ArmStates.SHOOT;
        feedback.setGoal(goalRad);
    }

    public double getGoal() {
        return feedback.getGoal().position;
    }

    public State getSetpoint() {

        return feedback.getSetpoint();
    }

    public ArmStates getState() {
        return state;
    }

    public void setDeployedMode(boolean isDeployed) {
        deployedMode = isDeployed;
        if (isDeployed) {
            feedback.setConstraints(ArmConstants.deployedConstraints);

        } else {
            feedback.setConstraints(ArmConstants.stowedConstraints);

        }
    }

    public void setVoltage(double volts) {
        // if (isKilled) {
        // armIO.setVoltage(0);
        // } else {
        armIO.setVoltage(volts);
        // }
    }

    // private ShuffleData<Double> kDData = new ShuffleData(this.getName(),
    // "kDdata", 0.0);

    public void moveToGoal() {

        double feedback = calculatePID(getPositionRad());
        State setpoint = getSetpoint();
        double accelerationSetpoint = (setpoint.velocity - prevSetpointVelocity) / 0.02;
        prevSetpointVelocity = setpoint.velocity;

        if (setpoint.position == ArmConstants.stowPositionRad
                && UtilityFunctions.withinMargin(0.05, getVelocityRadPerSec(), 0)) {
            setVoltage(0);
            return;
        }

        double feedforward;
        feedforward = calculateFF(getPositionRad(), setpoint.velocity,
                accelerationSetpoint);
        if (setpoint.velocity == 0) {
            // have the kS help the PID when stationary
            feedforward += Math.signum(feedback) * ArmConstants.stowedkS * 0.85;
        }
        setVoltage(feedforward + feedback);

    }

    public void toggleKill() {
        isKilled = !isKilled;
    }

    double lengthCFSToAxleX = Units.inchesToMeters(14);
    double lengthCFSToAxleY = Units.inchesToMeters(7);
    double lengthAxleToCFSAttatched = Units.inchesToMeters(15);

    double lengthCFSToAxle = Math.hypot(lengthCFSToAxleX, lengthCFSToAxleY);
    double axleToCFSTheta = Math.atan(lengthCFSToAxleY / lengthCFSToAxleX);

    public double calculateFF(double currentPositionRad, double setpointVelocityRadPerSec,
            double setpointAccelerationRadPerSecSquared) {
        // mathing the constant force spring angle for a seperate kG since its a
        // changing vector angle of constant magnitude
        double lengthCFS = Math.sqrt(lengthAxleToCFSAttatched * lengthAxleToCFSAttatched
                + lengthCFSToAxle * lengthCFSToAxle
                - 2 * lengthAxleToCFSAttatched * lengthCFSToAxle * Math.cos(currentPositionRad + axleToCFSTheta));

        double cosAngleCFS = (lengthCFS * lengthCFS + lengthAxleToCFSAttatched * lengthAxleToCFSAttatched
                - lengthCFSToAxle * lengthCFSToAxle) / (2 * lengthAxleToCFSAttatched * lengthCFSToAxle);

        double CFSFF = cosAngleCFS * ArmConstants.stowedkG * 0.3;
        double armFF;
        if (deployedMode) {
            armFF = deployedFeedforward.calculate(currentPositionRad, setpointVelocityRadPerSec,
                    accelerationSetpoint);
        } else {

            armFF = stowedFeedforward.calculate(currentPositionRad, setpointVelocityRadPerSec,
                    accelerationSetpoint);
        }

        return armFF + CFSFF;
    }

    public double calculatePID(double currentPositionRad) {
        return feedback.calculate(currentPositionRad);
    }

    public boolean atGoal() {
        return (Math.abs(data.positionRad - getGoal()) < 0.1);
    }

    public void updateState() {
        if (!atGoal() || Math.abs(getVelocityRadPerSec()) > 0.1) {
            state = ArmStates.IN_TRANIST;
            return;
        }
        if (getGoal() == ArmConstants.stowPositionRad) {
            state = ArmStates.STOW;
            return;
        }
        if (getGoal() == ArmConstants.climbPositionRad) {
            state = ArmStates.CLIMB;
            return;

        }
        if (getGoal() == ArmConstants.ampPositionRad) {
            state = ArmStates.AMP;
            return;
        }
        if (getGoal() == ArmConstants.subwooferPositionRad) {
            state = ArmStates.SUBWOOFER;
            return;
        }
        if (getGoal() == ArmConstants.podiumPositionRad) {
            state = ArmStates.PODIUM;
            return;
        }
        // if (Robot.state == SuperStructureStates.SHOOT) {
        // state = ArmStates.SHOOT;
        // return;
        // }

        if (getGoal() == ArmConstants.groundIntakepositionRad) {
            state = ArmStates.GROUND_INTAKE;
        }
    }

    // runs every 0.02 sec
    @Override
    public void periodic() {
        // System.out.println("0,0,0");
        armIO.updateData(data);
        updateState();
        stateLog.set(state.name());
        // moveToGoal();
        positionLog.set(Units.radiansToDegrees(getPositionRad()));
        velocityLog.set(Units.radiansToDegrees(data.velocityRadPerSec));
        // accelerationLog.set(Units.radiansToDegrees(data.accelerationRadPerSecSquared));
        voltageLog.set(data.appliedVolts);
        leftCurrentLog.set(data.leftCurrentAmps);
        rightCurrentLog.set(data.rightCurrentAmps);

        goalLog.set(Units.radiansToDegrees(feedback.getGoal().position));
        setpointPositionLog.set(Units.radiansToDegrees(feedback.getSetpoint().position));
        setpointVelocityLog.set(Units.radiansToDegrees(feedback.getSetpoint().velocity));
        setpointAccelerationLog.set(Units.radiansToDegrees(accelerationSetpoint));

        boolean driverStationStatus = DriverStation.isEnabled();
        if (driverStationStatus && !isEnabled) {
            isEnabled = driverStationStatus;
            armIO.setBreakMode();
        }
        if (!driverStationStatus && isEnabled) {
            armIO.setBreakMode();
            isEnabled = driverStationStatus;
        }

        SmartDashboard.putNumber("calced arm rad", ShootKinematics.getArmAngleRadGivenPose(Robot.swerve.getPose()));
        ShootKinematics.getRobotRotation(Robot.swerve.getPose());   
    }

}