package frc.robot.subsystems.example4;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.example.ExampleIO.ExampleData;
import frc.robot.utils.CurrentBudgettedSubsystem;

public class Example4 extends SubsystemBase implements CurrentBudgettedSubsystem {

    private ExampleData data = new ExampleData();
    private Example4IO exampleIO;
    private double currentSum = 0;
    // private Exampl

    // Constructor
    public Example4() {
        if (Robot.isReal()) {
            exampleIO = new Example4Sim();
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
            System.out.println("shintake");

            System.out.println(currentReduction);
        }    }
}
