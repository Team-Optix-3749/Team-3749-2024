package frc.robot.subsystems.example;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.example.ExampleIO.ExampleData;
import frc.robot.utils.Constants.ElectricalConstants;
import frc.robot.utils.CurrentBudgettedSubsystem;
import frc.robot.utils.ShuffleData;

public class Example extends SubsystemBase implements CurrentBudgettedSubsystem {

    private ExampleData data = new ExampleData();
    private ExampleIO exampleIO;
    private double estimatedCurrentDraw = 0;
    private double maxOutput = 0;
    private double volts = 0;

    private ShuffleData<Double> voltageLog = new ShuffleData<Double>("ex 1", "voltage", 0.0);
    private ShuffleData<Double> currentLog = new ShuffleData<Double>("ex 1", "current", 0.0);
    private ShuffleData<Double> tempLog = new ShuffleData<Double>("ex 1", "temp", 0.0);

    // private ShuffleData<Double> voltagelog = new ShuffleData<Double>("ex 2",
    // "voltage", 0.0);
    // private ShuffleData<Double> voltagelog = new ShuffleData<Double>("ex 2",
    // "voltage", 0.0);

    // private Exampl

    // Constructor
    public Example() {
        if (Robot.isSimulation()) {
            exampleIO = new ExampleSim();
        } else {            
            exampleIO = new ExampleSparkmax();

        }
    }

    public void increaseVoltage(double volts) {
        this.volts += volts;
    }

    // runs every 0.02 sec
    @Override
    public void periodic() {
        exampleIO.updateData(data);
        exampleIO.setMaxOutput(maxOutput);
        exampleIO.setVoltage(volts);
        
        SmartDashboard.putNumber("volt input", volts);
        currentLog.set(data.currentAmps);
        voltageLog.set(data.appliedVolts);
        tempLog.set(data.tempCelcius);
    }

    @Override
    public void reduceCurrentSum(double currentReduction) {
        maxOutput = currentReduction/ElectricalConstants.example2CurrentLimit;
    }

    @Override
    public double getEstimatedCurrentDraw() {
        return estimatedCurrentDraw;
    }
}
