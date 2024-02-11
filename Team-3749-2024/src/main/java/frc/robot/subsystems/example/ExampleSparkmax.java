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

    private CANSparkMax motor = new CANSparkMax(12, MotorType.kBrushless);
    private CANSparkMax motor2 = new CANSparkMax(13, MotorType.kBrushless);

    private double appliedVolts = 0.0;
    private int currentLimit = 0;

    public ExampleSparkmax() {
        System.out.println("[Init] Creating ExampleIOSim");

        motor.setSmartCurrentLimit(40);
        motor.setSmartCurrentLimit(40);
    }

    @Override
    public void updateData(ExampleData data) {

        // update sim values every 0.02 sec

        // distance traveled + Rad/Time * Time * diameter


    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        motor.setVoltage(appliedVolts);
        motor2.setVoltage(-appliedVolts);
    }

}
