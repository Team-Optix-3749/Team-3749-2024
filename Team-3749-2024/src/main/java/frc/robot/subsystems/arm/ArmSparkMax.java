package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.MiscConstants.Sim;

public class ArmSparkMax implements ArmIO {

    private CANSparkMax leftMotor = new CANSparkMax(ArmConstants.leftID, CANSparkMax.MotorType.kBrushless);
    private CANSparkMax rightMotor = new CANSparkMax(ArmConstants.rightID, CANSparkMax.MotorType.kBrushless);

    private RelativeEncoder leftEncoder = leftMotor.getEncoder();
    private RelativeEncoder rightEncoder = rightMotor.getEncoder();

    private AbsoluteEncoder absoluteEncoder;

    private double appliedVolts = 0;
    private double previousVelocity = 0;

    public ArmSparkMax() {
        System.out.println("[Init] Creating Arm Spark Max");
        absoluteEncoder = leftMotor.getAbsoluteEncoder();
        absoluteEncoder.setPositionConversionFactor(Math.PI * 2 / ArmConstants.sprocketRatio);
        absoluteEncoder.setVelocityConversionFactor(Math.PI * 2 / ArmConstants.sprocketRatio);
        absoluteEncoder.setZeroOffset(ArmConstants.encoderOffsetRad);

        leftEncoder.setVelocityConversionFactor(Math.PI * 2 / ArmConstants.gearRatio /60.0);
        rightEncoder.setVelocityConversionFactor(Math.PI * 2 / ArmConstants.gearRatio /60.0);

        rightMotor.setInverted(true);
        rightMotor.setSmartCurrentLimit(40);
        leftMotor.setSmartCurrentLimit(40);
        rightMotor.setIdleMode(IdleMode.kCoast);
        leftMotor.setIdleMode(IdleMode.kCoast);

    }
    // the arm will break if it ever goes past 120 degrees... should fix that lmao ;-;
    // technically would also break if we go past -60 but im much less concerned about that. 
    private double getAbsolutePositionRad() {
        double pos = absoluteEncoder.getPosition();

        if (pos > Units.degreesToRadians(128) ){
            pos -= absoluteEncoder.getPositionConversionFactor();
        }
        else if (pos < -1/3 * Math.PI){
            pos +=  absoluteEncoder.getPositionConversionFactor();
        }

        return pos ;
    }

    private double getVelocityRadPerSec() {
        return (leftEncoder.getVelocity() + rightEncoder.getVelocity())/2;
    }

    @Override
    public void updateData(ArmData data) {
        SmartDashboard.putString("break mode", leftMotor.getIdleMode().name());
        SmartDashboard.putNumber("abs encoder", absoluteEncoder.getPosition());

        previousVelocity = data.velocityRadPerSec;
        // distance traveled + Rad/Time * Time * diameter
        data.positionRad = getAbsolutePositionRad();

        data.velocityRadPerSec = getVelocityRadPerSec();

        data.accelerationRadPerSecSquared = (data.velocityRadPerSec - previousVelocity) / 0.02;

        data.appliedVolts = appliedVolts;

        data.leftCurrentAmps = Math.abs(leftMotor.getOutputCurrent());
        data.rightCurrentAmps = Math.abs(rightMotor.getOutputCurrent());

        data.leftTempCelcius = leftMotor.getMotorTemperature();
        data.rightTempCelcius = rightMotor.getMotorTemperature();

    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        leftMotor.setVoltage(appliedVolts);
        rightMotor.setVoltage(appliedVolts);

    }

    @Override
    public void setBreakMode() {
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void setCoastMode() {
        leftMotor.setIdleMode(IdleMode.kCoast);
        rightMotor.setIdleMode(IdleMode.kCoast);

    }

}
