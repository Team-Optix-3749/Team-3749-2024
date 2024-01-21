package frc.robot.subsystems.example2;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.example.ExampleIO.ExampleData;
import frc.robot.utils.CurrentBudgettedSubsystem;

public class Example2 extends SubsystemBase implements CurrentBudgettedSubsystem {

    private ExampleData data = new ExampleData();
    private ExampleIO exampleIO;
    private double currentSum = 30;
    // private Exampl

    // Constructor
    public Example2() {
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
            System.out.println("arm");

            System.out.println(currentReduction);
        }    }

}
