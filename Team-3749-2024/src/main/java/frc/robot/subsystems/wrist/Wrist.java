package frc.robot.subsystems.wrist;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

import org.opencv.core.RotatedRect;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.wrist.WristIO.WristData;
import frc.robot.utils.Constants;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.SmartData;
import frc.robot.utils.Constants.WristConstants;

public class Wrist extends SubsystemBase {
    // hello test
    private WristIO wristIO;
    private WristData data = new WristData();

    private ProfiledPIDController wristController = new ProfiledPIDController(Constants.WristConstants.PID.kP,
            Constants.WristConstants.PID.kI, Constants.WristConstants.PID.kD,
            Constants.WristConstants.trapezoidConstraint);

    private ArmFeedforward wristFF = new ArmFeedforward(Constants.WristConstants.simkS, Constants.WristConstants.simkG,
            Constants.WristConstants.simkV);

    private HashMap<Boolean, Double> setpointToggle = new HashMap<Boolean, Double>();

    private boolean isGroundIntake = false;

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
    private ShuffleData<Double> currentLog = new ShuffleData<Double>(this.getName(), "current",
            0.0);

    private ShuffleData<Double> goalLog = new ShuffleData<Double>(this.getName(), "goal",
            0.0);
    private ShuffleData<Double> setpointPositionLog = new ShuffleData<Double>(this.getName(), "setpoint position",
            0.0);
    private ShuffleData<Double> setpointVelocityLog = new ShuffleData<Double>(this.getName(), "setpoint velocity",
            0.0);
    private ShuffleData<Double> setpointAccelerationLog = new ShuffleData<Double>(this.getName(),
            "setpoint acceleration", 0.0);
    private ShuffleData<Double> errorPositionLog = new ShuffleData<Double>(this.getName(), "error position",
            0.0);
    private ShuffleData<Double> errorVelocityLog = new ShuffleData<Double>(this.getName(), "error velocity",
            0.0);

    public Wrist() {
        setpointToggle.put(true, Constants.WristConstants.groundGoal);
        setpointToggle.put(false, Constants.WristConstants.stowGoal);
        wristIO = new WristSparkMax();
        if (Robot.isSimulation()) {
            wristIO = new WristSim();
        }
    }

    // runs twice???
    public void toggleWristGoal() {
        this.isGroundIntake = !this.isGroundIntake;
        wristController.setGoal(setpointToggle.get(this.isGroundIntake));
        System.out.println("togggle");
        System.out.println(isGroundIntake);
    }

    public void setGoalGround() {
        System.out.println("ground");
        wristController.setGoal(setpointToggle.get(true));

    }

    public void setGoalStow() {
        System.out.println("stow");

        wristController.setGoal(setpointToggle.get(false));

    }

    public State getWristGoal() {
        return wristController.getGoal();
    }

    public State getWristSetpoint() {
        return wristController.getSetpoint();
    }

    public boolean getIsGroundIntake() {
        return isGroundIntake;
    }

    public double getPositionRad() {
        return (data.positionRad);
    }

    public double getVelocityRadPerSec() {
        return (data.velocityRadPerSec);
    }

    private ShuffleData<Double> kPData = new ShuffleData(this.getName(), "kpdata", 0.0);
    private ShuffleData<Double> kVData = new ShuffleData(this.getName(), "kVdata", 0.0);

    public void moveWristToAngle(double positionRad, double velocityRadPerSec, double accelerationRadPerSecSquared) {



        State state = getWristSetpoint();
        double voltage = wristController.calculate(data.positionRad);
        if (positionRad == 0 && data.positionRad <7){
            setVoltage(0);
            return;
        }

        if (Robot.isSimulation()) {

            voltage += wristFF.calculate(data.positionRad, state.velocity); // is getting the goal redundant?
        } else {
            voltage += getWristGoal().position == Units.degreesToRadians(140)
                    ? velocityRadPerSec * WristConstants.realkVForward
                    : velocityRadPerSec *WristConstants.realkVBackward;
            voltage += calculateGravityFeedForward(data.positionRad, Robot.arm.getRotation2d().getRadians());
        }

        setVoltage(voltage);
        // System.out.println(wristController.getGoal().position);
    }

    public void setVoltage(double volts) {
        wristIO.setVoltage(volts);
    }

    private ShuffleData<Double> kGData = new ShuffleData<Double>("wrist", "kGData", 0.0);

    public void runFF(double add) {

        wristIO.setVoltage(calculateGravityFeedForward(data.positionRad, Robot.arm.getRotation2d().getRadians()) + add);
    }

    public double calculateGravityFeedForward(double wristPositionRad, double armPositionRad) {

        return WristConstants.kYIntercept
                + WristConstants.kBar * wristPositionRad
                + WristConstants.kBarSquared * Math.pow(wristPositionRad, 2)
                + WristConstants.kBarCubed * Math.pow(wristPositionRad, 3)
                + WristConstants.kArm * armPositionRad
                + WristConstants.kArmSquared * Math.pow(armPositionRad, 2)
                + WristConstants.kBarArm * wristPositionRad * armPositionRad
                + WristConstants.kBarSquaredArm * Math.pow(wristPositionRad, 2) * armPositionRad
                + WristConstants.kBarCubedArm * Math.pow(wristPositionRad, 3) * armPositionRad
                + WristConstants.kBarArmSquared * wristPositionRad * Math.pow(armPositionRad, 2)
                + WristConstants.kBarSquaredArmSquared * Math.pow(wristPositionRad, 2) * Math.pow(armPositionRad, 2)
                + WristConstants.kBarCubedArmSquared * Math.pow(wristPositionRad, 3) * Math.pow(armPositionRad, 2);
    }

    @Override
    public void periodic() {
        wristIO.updateData(data);

        // mechanismArm.setAngle(data.positionRad);
        // SmartDashboard.putData("Mech2d", mechanism);
        // mechanismArm.setAngle(Math.toDegrees(data.positionRad));

        positionLog.set(Units.radiansToDegrees(data.positionRad));
        velocityLog.set(Units.radiansToDegrees(data.velocityRadPerSec));
        accelerationLog.set(Units.radiansToDegrees(data.accelerationRadPerSecSquared));
        goalLog.set(Units.radiansToDegrees(getWristGoal().position));
        setpointPositionLog.set(Units.radiansToDegrees(getWristSetpoint().position));
        setpointVelocityLog.set(Units.radiansToDegrees(getWristSetpoint().velocity));
        voltageLog.set(data.appliedVolts);
        currentLog.set(data.currentAmps);
        errorPositionLog.set(Units.radiansToDegrees(getWristSetpoint().position - data.positionRad));
        errorVelocityLog.set(Units.radiansToDegrees(getWristSetpoint().velocity - data.velocityRadPerSec));

        SmartDashboard.putNumber("FF", calculateGravityFeedForward(data.positionRad, 0));

        // test
    }

}