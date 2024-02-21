package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.ShooterIO.ShooterData;
import frc.robot.utils.Constants;
import frc.robot.utils.ShuffleData;

public class Shooter extends SubsystemBase {

  private ShooterIO shooterIO;
  private ShooterData data = new ShooterData();

  private PIDController bottomFeedback = new PIDController(
      Constants.ShooterConstants.shooterBottomPID.kP,
      Constants.ShooterConstants.shooterBottomPID.kI,
      Constants.ShooterConstants.shooterBottomPID.kD);
  
      
  private PIDController topFeedback = new PIDController(
      Constants.ShooterConstants.shooterTopPID.kP,
      Constants.ShooterConstants.shooterTopPID.kI,
      Constants.ShooterConstants.shooterTopPID.kD);

  private SimpleMotorFeedforward topShooterFF = new SimpleMotorFeedforward(
      0,
      Constants.ShooterConstants.topkV,
      0);

  private SimpleMotorFeedforward bottomShooterFF = new SimpleMotorFeedforward(
      0,
      Constants.ShooterConstants.bottomkV,
      0);

  public Shooter() {
    shooterIO = new ShooterSparkMax();
    if (Robot.isSimulation()) {
      shooterIO = new ShooterSim();
    }
  }

  ShuffleData<Double> kVData = new ShuffleData<Double>(this.getName(), "kVData", 0.0);
    ShuffleData<Double> kPData = new ShuffleData<Double>(this.getName(), "kPData", 0.0);
    ShuffleData<Double> velData = new ShuffleData<Double>(this.getName(), "velData", 0.0);

  public void setShooterVelocity(double velocityRadPerSec) {

    double topVoltage = topFeedback.calculate(
        data.topShooterVelocityRadPerSec,
        velData.get()) +
        topShooterFF.calculate(velData.get());

    double bottomVoltage = bottomFeedback.calculate(
        data.bottomShooterVelocityRadPerSec,
        velData.get()) +
        bottomShooterFF.calculate(velData.get());

    bottomVoltage = velData.get() * kVData.get() + (velData.get() - data.topShooterVelocityRadPerSec)*kPData.get();

    setVoltage(topVoltage, bottomVoltage);
  }

  public void setVoltage(double topVolts, double bottomVolts) {
    shooterIO.setVoltage(topVolts, bottomVolts);
  }

  @Override
  public void periodic() {
    
    shooterIO.updateData(data);

    SmartDashboard.putNumber("bottomShooterVolts", data.bottomShooterVolts);
    SmartDashboard.putNumber("bottomShooterVelocityRadPerSec", data.bottomShooterVelocityRadPerSec);
    SmartDashboard.putNumber("bottomShooterPositionRad", data.bottomShooterPositionRad);
    SmartDashboard.putNumber("bottomShooterTempCelsius", data.bottomShooterTempCelcius);

    SmartDashboard.putNumber("topShooterVolts", data.topShooterVolts);
    SmartDashboard.putNumber("topShooterVelocityRadPerSec", data.topShooterVelocityRadPerSec);
    SmartDashboard.putNumber("topShooterTempCelsius", data.topShooterTempCelcius);
    SmartDashboard.putNumber("topShooterPositionRad", data.topShooterPositionRad);
  }

}
