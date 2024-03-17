package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.intake.IntakeIO.IntakeData;
import frc.robot.subsystems.intake.PhotoelectricIO.PhotoelectricData;
import frc.robot.subsystems.led.LEDConstants.LEDPattern;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.wrist.WristConstants.WristStates;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.SuperStructureStates;

public class Intake extends SubsystemBase {

    private IntakeIO intakeIO;
    private IntakeData data = new IntakeData();
    private PhotoelectricData sensorData = new PhotoelectricData();
    private PhotoelectricIO photoeletricIO;

    private PIDController feedback = new PIDController(
            IntakeConstants.intakePID.kP,
            IntakeConstants.intakePID.kI,
            IntakeConstants.intakePID.kD);

    private SimpleMotorFeedforward intakeFF = new SimpleMotorFeedforward(
            0,
            IntakeConstants.kV,
            0);

    private IntakeStates state = IntakeStates.STOP;

    private ShuffleData<Double> IntakeVelocityLog = new ShuffleData<Double>(this.getName(), "intake velocity", 0.0);
    private ShuffleData<Double> IntakevoltageLog = new ShuffleData<Double>(this.getName(), "intake voltage", 0.0);
    private ShuffleData<Double> IntakecurrentLog = new ShuffleData<Double>(this.getName(), "intake current", 0.0);
    private ShuffleData<Boolean> photoelectricLog = new ShuffleData<Boolean>(this.getName(),
            "photoelectric sensor tripped", false);
    private ShuffleData<String> stateLog = new ShuffleData<String>(this.getName(), "state",
            IntakeStates.STOP.name());

    private boolean hasPiece = false;
    private boolean indexedPiece = false;

    public Intake() {
        if (Robot.isSimulation()) {
            intakeIO = new IntakeSim();
            photoeletricIO = new PhotoelectricIO() {
            };

        } else {
            intakeIO = new IntakeSparkMax();
            photoeletricIO = new JTVisiSight();
        }
    }

    public double getVelocityRadPerSec() {
        return data.intakeVelocityRadPerSec;
    }

    public void setHasPiece(boolean has) {
        hasPiece = has;
        if (has) {
            Robot.led.setLEDPattern(LEDPattern.GREEN);

        } else {
            Robot.led.setLEDPattern(LEDPattern.WHITE);

        }
    }

    public boolean getHasPiece() {
        return hasPiece;
    }

    public void setIndexedPiece(boolean indexed) {
        indexedPiece = indexed;
    }

    public boolean getIndexedPiece() {
        return indexedPiece;
    }

    public IntakeStates getState() {
        return state;
    }

    public void setIntakeVelocity(double velocityRadPerSec) {

        double voltage = feedback.calculate(
                data.intakeVelocityRadPerSec,
                velocityRadPerSec) +
                intakeFF.calculate(velocityRadPerSec);

        // voltage = velData.get() * kVData.get() + (velData.get() -
        // data.intakeVelocityRadPerSec) * kPData.get();
        // System.out.println(voltage);
        // System.out.println(velData.get());

        setVoltage(voltage);
    }

    public void setVoltage(double volts) {
        intakeIO.setVoltage(volts);
    }

    public void stop() {
        intakeIO.setVoltage(0);

    }

    public void setState(IntakeStates state) {
        this.state = state;
    }

    public void runIntakeState() {
        switch (state) {
            case STOP:
                stop();
                break;
            case INTAKE:
                intake();
                break;
            case INDEX:
                index();
                break;
            case FEED:
                feed();
                break;
            case OUTTAKE:
                outtake();
                break;
            case AMP:
                amp();

        }
    }

    private void amp() {
        setVoltage(-8);
        setHasPiece(false);
        setIndexedPiece(false);
        Robot.led.setLEDPattern(LEDPattern.WHITE);
    }

    private void feed() {
        if (Robot.state == SuperStructureStates.AMP) {
            Robot.shooter.setState(ShooterStates.AMP);
            setState(IntakeStates.AMP);
            amp();
            return;
        }
        setVoltage(12);
        setHasPiece(false);
        setIndexedPiece(false);
        Robot.led.setLEDPattern(LEDPattern.WHITE);

    }

    private void outtake() {
        setVoltage(-12);
        setHasPiece(false);
        setIndexedPiece(false);
        Robot.led.setLEDPattern(LEDPattern.WHITE);

    }

    private void intake() {
        if (!hasPiece) {
            setIntakeVelocity(IntakeConstants.intakeVelocityRadPerSec);
        } else {
            state = IntakeStates.INDEX;
        }
    }

    private void index() {
        if (!indexedPiece) {

            setVoltage(-3.5);
        } else {
            state = IntakeStates.STOP;
        }

    }

    @Override
    public void periodic() {
        runIntakeState();
        intakeIO.updateData(data);
        photoeletricIO.updateData(sensorData);

        IntakeVelocityLog.set(data.intakeVelocityRadPerSec);

        IntakevoltageLog.set(data.intakeVolts);

        IntakecurrentLog.set(data.currentAmps);

        photoelectricLog.set(sensorData.sensing);

        stateLog.set(state.name());
        SmartDashboard.putBoolean("has piece", hasPiece);
        SmartDashboard.putBoolean("indexed piece", indexedPiece);

    }

}
