package frc.robot.subsystems.example5;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.example.ExampleIO.ExampleData;
import frc.robot.utils.CurrentBudgettedSubsystem;

public class Example5 extends SubsystemBase implements CurrentBudgettedSubsystem {

    private ExampleData data = new ExampleData();
    private Example5IO exampleIO;
    private double currentSum = 0;
    private double volts = 0;
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
        exampleIO.setVoltage(volts);
        SmartDashboard.putNumber("volt input", volts);

    }

    @Override
    public void reduceCurrentSum(IntSupplier currentReductionSupplier) {
        int currentReduction = currentReductionSupplier.getAsInt();
        exampleIO.setCurrentLimitReduction(currentReduction);
    }
}
