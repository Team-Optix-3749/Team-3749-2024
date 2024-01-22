package frc.robot.subsystems.example3;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.example.ExampleIO.ExampleData;
import frc.robot.utils.CurrentBudgettedSubsystem;

public class Example3 extends SubsystemBase implements CurrentBudgettedSubsystem {

    private ExampleData data = new ExampleData();
    private ExampleIO exampleIO;
    private double currentSum = 0;
    // private Exampl

    // Constructor
    public Example3() {
        if (Robot.isReal()) {
            exampleIO = new ExampleSim();
        }
    }

    public double getCurrentSum() {
        return currentSum;
    }

    // runs every 0.02 sec
    @Override
    public void periodic() {

    }

    @Override
    public void reduceCurrentSum(IntSupplier currentReductionSupplier) {
        double currentReduction = currentReductionSupplier.getAsInt();
        if (currentReduction > 0) {

        }
    }
}
