package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeSparkMax implements IntakeIO {

  private CANSparkMax intakeMotor = new CANSparkMax(
      IntakeConstants.intakeId,
      MotorType.kBrushless);
  private RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
  private double intakeGoalVolts = 0;

  public IntakeSparkMax() {

    intakeMotor.setSmartCurrentLimit(40);
    intakeMotor.setInverted(true);
    intakeEncoder.setPosition(2 * Math.PI / IntakeConstants.gearRatio);
    intakeEncoder.setVelocityConversionFactor(2 * Math.PI / IntakeConstants.gearRatio * (1.0 / 60.0));
    intakeMotor.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void updateData(IntakeData data) {
    data.intakeVolts = intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput();
    data.intakeVelocityRadPerSec = intakeEncoder.getVelocity();
    data.intakeTempCelcius = intakeMotor.getMotorTemperature();
    

     
  }

  @Override
  public void setVoltage(double intakeVolts) {
    intakeGoalVolts = MathUtil.clamp(intakeVolts, -12, 12);
    intakeMotor.setVoltage(intakeGoalVolts);
  }
  
}
