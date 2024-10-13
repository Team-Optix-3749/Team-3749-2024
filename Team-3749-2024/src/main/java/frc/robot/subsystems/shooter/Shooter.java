package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.shooter.ShooterIO.ShooterData;
import frc.robot.subsystems.wrist.WristConstants.WristStates;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.UtilityFunctions;

public class Shooter extends SubsystemBase {

  private ShooterIO shooterIO;
  private ShooterData data = new ShooterData();
  private ShooterStates state = ShooterStates.STOP;
  private boolean intakeSpedUp = false;

  private PIDController bottomFeedback = new PIDController(
      ShooterConstants.shooterBottomPID.kP,
      ShooterConstants.shooterBottomPID.kI,
      ShooterConstants.shooterBottomPID.kD);

  private PIDController topFeedback = new PIDController(
      ShooterConstants.shooterTopPID.kP,
      ShooterConstants.shooterTopPID.kI,
      ShooterConstants.shooterTopPID.kD);

  private SimpleMotorFeedforward topShooterFF = new SimpleMotorFeedforward(
      0,
      ShooterConstants.topkV,
      0);

  private SimpleMotorFeedforward bottomShooterFF = new SimpleMotorFeedforward(
      0,
      ShooterConstants.bottomkV,
      0);

  private ShuffleData<Double> topShooterVelocityLog = new ShuffleData<Double>(this.getName(), "top shooter velocity",
      0.0);
  private ShuffleData<Double> bottomShooterVelocityLog = new ShuffleData<Double>(this.getName(),
      "bottom shooter velocity", 0.0);
  private ShuffleData<Double> topShootervoltageLog = new ShuffleData<Double>(this.getName(), "top shooter voltage",
      0.0);
  private ShuffleData<Double> bottomShootervoltageLog = new ShuffleData<Double>(this.getName(),
      "bottom shooter voltage", 0.0);
  private ShuffleData<Double> topShootercurrentLog = new ShuffleData<Double>(this.getName(), "top shooter current",
      0.0);
  private ShuffleData<Double> bottomShootercurrentLog = new ShuffleData<Double>(this.getName(),
      "bottom shooter current", 0.0);
  private ShuffleData<String> stateLog = new ShuffleData<String>(this.getName(), "state",
      ShooterStates.STOP.name());

  private Timer timer = new Timer();

  public Shooter() {
    shooterIO = new ShooterSparkMax();
    if (Robot.isSimulation()) {
      shooterIO = new ShooterSim();
    }
  }

  public double getVelocityRadPerSec() {
    return (data.topShooterVelocityRadPerSec + data.bottomShooterVelocityRadPerSec) / 2;
  }

  public double getTopVelocityRadPerSec(){
    return data.topShooterVelocityRadPerSec;
  }
    public double getBottomVelocityRadPerSec(){
    return data.bottomShooterVelocityRadPerSec;
  }

  public ShooterStates getState() {
    return state;
  }

  public void setShooterVelocity(double velocityRadPerSec) {

    double topVoltage = topFeedback.calculate(
        data.topShooterVelocityRadPerSec,
        velocityRadPerSec) +
        topShooterFF.calculate(velocityRadPerSec);

    double bottomVoltage = bottomFeedback.calculate(
        data.bottomShooterVelocityRadPerSec,
        velocityRadPerSec) +
        bottomShooterFF.calculate(velocityRadPerSec);

    // double topVoltage = kVData.get() *velocityRadPerSec;
    // double bottomVoltage = kVData.get()
    setVoltage(topVoltage, bottomVoltage);
  }

  public void setVoltage(double topVolts, double bottomVolts) {
    shooterIO.setVoltage(topVolts, bottomVolts);
  }

  public void stop() {
    intakeSpedUp = false;

    shooterIO.setVoltage(0, 0);

  }

  public void runShooterState() {
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
      case SPOOL:
        setShooterVelocity(ShooterConstants.shooterVelocityRadPerSec);
        break;
      case AMP:
        stop();;
      case TROLL:
        setShooterVelocity(150);
    }
  }

  public void setState(ShooterStates state) {
    intakeSpedUp = false;   
    this.state = state;
  }

  private void intake() {
    
    // this is just for the setpoint checker below
    setVoltage(-0.75, -0.75);
    if (!intakeSpedUp && UtilityFunctions.withinMargin(15, Robot.intake.getVelocityRadPerSec(), IntakeConstants.intakeVelocityRadPerSec)) {
      intakeSpedUp = true;     
       timer.stop();
      timer.reset();
      timer.start();
    }
    SmartDashboard.putNumber("Index Timer", timer.get());
    if ((getBottomVelocityRadPerSec() > -8 || getTopVelocityRadPerSec() > -8) && intakeSpedUp && timer.get()>0.48) {
      Robot.intake.setHasPiece(true);
      state = ShooterStates.INDEX;
      timer.stop();
      timer.reset();
      // stateP
    }
    SmartDashboard.putBoolean("intake sped up", intakeSpedUp);

  }

  private void index() {
    setVoltage(-1.2, -1.2);

    
    if ((getBottomVelocityRadPerSec() < -30 && getTopVelocityRadPerSec() < -30)) {
      state = ShooterStates.STOP;
      Robot.intake.setIndexedPiece(true);
    }

  }

  @Override
  public void periodic() {
    shooterIO.updateData(data);
    runShooterState();

    topShooterVelocityLog.set(data.topShooterVelocityRadPerSec);
    bottomShooterVelocityLog.set(data.bottomShooterVelocityRadPerSec);

    topShootervoltageLog.set(data.topShooterVolts);
    bottomShootervoltageLog.set(data.bottomShooterVolts);

    topShootercurrentLog.set(data.topShooterCurrentAmps);
    bottomShootercurrentLog.set(data.bottomShooterCurrentAmps);

    stateLog.set(state.name());
  }

}
