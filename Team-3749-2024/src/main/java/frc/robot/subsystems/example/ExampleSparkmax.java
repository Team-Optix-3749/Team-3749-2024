package frc.robot.subsystems.example;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.ModuleConstants;
import frc.robot.utils.Constants.Sim;

public class ExampleSparkmax implements ExampleIO {

    private CANSparkMax motor = new CANSparkMax(0, MotorType.kBrushless);
    private RelativeEncoder encoder = motor.getEncoder();
    private double appliedVolts = 0.0;
    private int currentLimit = 0;

    public ExampleSparkmax() {
        System.out.println("[Init] Creating ExampleIOSim");

        motor.setSmartCurrentLimit(ModuleConstants.driveCurrentLimmit);
        encoder.setPositionConversionFactor(2 * Math.PI);
        encoder.setVelocityConversionFactor(1 / 60 * 2 * Math.PI);
    }

    @Override
    public void updateData(ExampleData data) {

        // update sim values every 0.02 sec

        // distance traveled + Rad/Time * Time * diameter
        data.positionRad = encoder.getPosition();

        data.velocityRadPerSec = encoder.getVelocity();

        data.appliedVolts = appliedVolts;

        data.currentAmps = Math.abs(motor.getOutputCurrent());

        data.tempCelcius = motor.getMotorTemperature();

        data.currentLimit = currentLimit;

    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        motor.setVoltage(appliedVolts);
    }

    @Override
    public void setCurrentLimitReduction(int currentReduction){
        currentLimit=ModuleConstants.driveCurrentLimmit-currentReduction;
        motor.setSmartCurrentLimit(currentLimit);
        
    }
}
