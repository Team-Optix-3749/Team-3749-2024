package frc.robot.subsystems.example5;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants.ElectricalConstants;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.ModuleConstants;
import frc.robot.utils.Constants.Sim;

public class Example5Sparkmax implements Example5IO {

    private CANSparkMax motor = new CANSparkMax(3, MotorType.kBrushless);
    private RelativeEncoder encoder = motor.getEncoder();
    private double appliedVolts = 0.0;
    private int currentLimit = 0;

    public Example5Sparkmax() {
        System.out.println("[Init] Creating ExampleIOSim");

        motor.setSmartCurrentLimit(ElectricalConstants.example5CurrentLimit);
    }

    @Override
    public void updateData(Example5Data data) {

        // update sim values every 0.02 sec

        // distance traveled + Rad/Time * Time * diameter
        data.positionRad = encoder.getPosition();

        data.velocityRadPerSec = encoder.getVelocity();

        data.appliedVolts = motor.getAppliedOutput()*motor.getBusVoltage();
        
        data.currentAmps = motor.getOutputCurrent();

        data.tempCelcius = motor.getMotorTemperature();


    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        motor.setVoltage(appliedVolts);
    }


}
