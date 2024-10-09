package frc.robot.subsystems.shooter;

import org.opencv.photo.TonemapMantiuk;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;

public class ShooterSparkMax implements ShooterIO {

    private CANSparkMax topShooter = new CANSparkMax(ShooterConstants.shooterTopId, MotorType.kBrushless);
    private CANSparkMax bottomShooter = new CANSparkMax(ShooterConstants.shooterBottomId, MotorType.kBrushless);

    private RelativeEncoder topEncoder = topShooter.getEncoder();
    private RelativeEncoder bottomEncoder = bottomShooter.getEncoder();

    private double bottomShooterGoalVolts = 0;
    private double topShooterGoalVolts = 0;

    public ShooterSparkMax() {
        bottomShooter.setInverted(true);


        topShooter.setSmartCurrentLimit(40);
        bottomShooter.setSmartCurrentLimit(40);

        bottomEncoder.setVelocityConversionFactor((2 * Math.PI) / 60);
        topEncoder.setVelocityConversionFactor((2 * Math.PI) / 60);

        bottomEncoder.setPositionConversionFactor(2 * Math.PI);
        topEncoder.setPositionConversionFactor(2 * Math.PI);

        topShooter.setIdleMode(IdleMode.kCoast);
        bottomShooter.setIdleMode(IdleMode.kCoast); 
        bottomShooter.setIdleMode(IdleMode.kBrake);
        topShooter.setIdleMode(IdleMode.kBrake);
    }


    @Override
    public void updateData(ShooterData data) {
        data.topShooterVolts = topShooter.getBusVoltage() * topShooter.getAppliedOutput();
        data.topShooterVelocityRadPerSec = topEncoder.getVelocity();
        
        data.topShooterTempCelcius = topShooter.getMotorTemperature();
        data.topShooterPositionRad = topEncoder.getPosition();
        data.topShooterCurrentAmps = topShooter.getOutputCurrent();

        data.bottomShooterVolts = bottomShooter.getBusVoltage() * bottomShooter.getAppliedOutput();
        data.bottomShooterVelocityRadPerSec = bottomEncoder.getVelocity();
        data.bottomShooterTempCelcius = bottomShooter.getMotorTemperature();
        data.bottomShooterPositionRad = bottomEncoder.getPosition();
        data.bottomShooterCurrentAmps = bottomShooter.getOutputCurrent();
    }

    @Override
    public void setVoltage(double topShooterVolts, double bottomShooterVolts) {
        bottomShooterGoalVolts = MathUtil.clamp(bottomShooterVolts, -12, 12);
        topShooterGoalVolts = MathUtil.clamp(topShooterVolts, -12, 12);
        topShooter.setVoltage(topShooterGoalVolts);
        bottomShooter.setVoltage(bottomShooterGoalVolts);
    }

}

