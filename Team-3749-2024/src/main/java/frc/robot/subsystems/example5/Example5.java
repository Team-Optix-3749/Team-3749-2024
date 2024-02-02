package frc.robot.subsystems.example5;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.example.ExampleIO.ExampleData;
import frc.robot.subsystems.example5.Example5IO.Example5Data;
import frc.robot.utils.CurrentBudgettedSubsystem;
import frc.robot.utils.ShuffleData;

public class Example5 extends SubsystemBase implements CurrentBudgettedSubsystem {

    private Example5Data data = new Example5Data();
    private Example5IO exampleIO;
    private double currentSum = 0;
    private double volts = 0;

    private ShuffleData<Double> voltageLog = new ShuffleData<Double>("ex 5", "voltage", 0.0);
    private ShuffleData<Double> currentLog = new ShuffleData<Double>("ex 5", "current", 0.0);
       private ShuffleData<Double> tempLog = new ShuffleData<Double>("ex 5", "temp", 0.0);

    // private ShuffleData<Double> voltagelog = new ShuffleData<Double>("ex 5", "voltage", 0.0);
    // private ShuffleData<Double> voltagelog = new ShuffleData<Double>("ex 5", "voltage", 0.0);

    // private Exampl

    // Constructor
    public Example5() {
        if (Robot.isSimulation()) {
            exampleIO = new Example5Sim();
        } else {
            exampleIO = new Example5Sparkmax();
        }
    }

    public double getCurrentSum() {
        return currentSum;
    }

    public void increaseVoltage(double volts) {
        this.volts += volts;
    }

    // runs every 0.02 sec
    @Override
    public void periodic() {
        exampleIO.updateData(data);
        exampleIO.setVoltage(volts);
        SmartDashboard.putNumber("volt input", volts);
        currentLog.set(data.currentAmps);
        voltageLog.set(data.appliedVolts);
        tempLog.set(data.tempCelcius);
    }

    @Override
    public void reduceCurrentSum(IntSupplier currentReductionSupplier) {
        int currentReduction = currentReductionSupplier.getAsInt();
        exampleIO.setCurrentLimitReduction(currentReduction);
    }
}
