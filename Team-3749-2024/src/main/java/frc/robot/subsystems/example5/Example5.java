package frc.robot.subsystems.example5;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.example.ExampleIO.ExampleData;
import frc.robot.subsystems.example5.Example5IO.Example5Data;
import frc.robot.utils.CurrentBudgettedSubsystem;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.Constants.ElectricalConstants;

public class Example5 extends SubsystemBase implements CurrentBudgettedSubsystem {

    private Example5Data data = new Example5Data();
    private Example5IO exampleIO;
    private double estimatedCurrentDraw = 0;
    private double maxOutput = 0;
    private double volts = 0;

    private ShuffleData<Double> voltageLog = new ShuffleData<Double>("ex 5", "voltage", 0.0);
    private ShuffleData<Double> currentLog = new ShuffleData<Double>("ex 5", "current", 0.0);
    private ShuffleData<Double> tempLog = new ShuffleData<Double>("ex 5", "temp", 0.0);
    private ShuffleData<Double> velocityLog = new ShuffleData<Double>("ex 5", "velocity", 0.0);

    private double currentSeconds = 0;
    private Timer timer = new Timer();

    // private ShuffleData<Double> voltagelog = new ShuffleData<Double>("ex 5",
    // "voltage", 0.0);
    // private ShuffleData<Double> voltagelog = new ShuffleData<Double>("ex 5",
    // "voltage", 0.0);

    // private Exampl

    // Constructor
    public Example5() {
        if (Robot.isSimulation()) {
            exampleIO = new Example5Sim();
        } else {
            exampleIO = new Example5Sparkmax();
        }
    }

    public void increaseVoltage(double volts) {
        timer.reset();
        timer.start();
        this.volts += volts;
    }

    // runs every 0.02 sec
    @Override
    public void periodic() {
        exampleIO.updateData(data);
        exampleIO.setMaxOutput(maxOutput);
        exampleIO.setVoltage(volts);

        currentLog.set(data.currentAmps);
        voltageLog.set(data.appliedVolts);
        tempLog.set(data.tempCelcius);
        velocityLog.set(data.velocityRadPerSec);
        if (timer.get() > 0) {
            currentSeconds += data.currentAmps * 0.02;
            SmartDashboard.putNumber("avg current", currentSeconds/timer.get());
        }
        if (DriverStation.isDisabled() && timer.get()>0) {
            System.out.println("total!");
            
            SmartDashboard.putNumber("avg current", currentSeconds/timer.get());
            timer.stop();
            timer.reset();
        }
    }

    @Override
    public void reduceCurrentSum(double currentReduction) {
        maxOutput = currentReduction / ElectricalConstants.example5CurrentLimit;
    }

    @Override
    public double getEstimatedCurrentDraw() {
        return estimatedCurrentDraw;
    }
}
