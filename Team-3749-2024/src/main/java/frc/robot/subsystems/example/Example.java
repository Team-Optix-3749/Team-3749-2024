package frc.robot.subsystems.example;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.example.ExampleIO.ExampleData;
import frc.robot.utils.CurrentBudgettedSubsystem;

public class Example extends SubsystemBase implements CurrentBudgettedSubsystem {

    private ExampleData data = new ExampleData();
    private ExampleIO exampleIO;
    private double currentSum = 240;
    // private Example

    // Constructor
    public Example() {
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
    public void reduceCurrentSum(DoubleSupplier currentReductionSupplier) {
        double currentReduction = currentReductionSupplier.getAsDouble();
        if (currentReduction>0){
            System.out.println("swerve");

            System.out.println(currentReduction);
        }    }

}
