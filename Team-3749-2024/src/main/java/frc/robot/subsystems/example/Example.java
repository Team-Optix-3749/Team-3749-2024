package frc.robot.subsystems.example;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.example.ExampleIO.ExampleData;
import frc.robot.utils.CurrentBudgettedSubsystem;

public class Example extends SubsystemBase implements CurrentBudgettedSubsystem {

    private ExampleData data = new ExampleData();
    private ExampleIO exampleIO;
    private double currentSum = 120;
    // private Example

    // Constructor
    public Example() {
        if (Robot.isSimulation()) {
            exampleIO = new ExampleSim();
        }
    }

    public double getCurrentSum() {
        return currentSum;
    }

    // runs every 0.02 sec
    @Override
    public void periodic() {
        exampleIO.updateData(data);
        currentSum = data.currentAmps;
    }

    @Override
    public void reduceCurrentSum(IntSupplier currentReductionSupplier) {
        int currentReduction = currentReductionSupplier.getAsInt();
        if (currentReduction > 0) {

            // divide by 4 since this is supposed to kinda be like swerve 
            exampleIO.setCurrentLimitReduction((int)Math.ceil(currentReduction/4.0));

        }

    }

}
