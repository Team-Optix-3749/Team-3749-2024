package frc.robot.utils;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utils.Constants.CurrentConstants;

public class CurrentBudgeteer extends SubsystemBase {

    private double currentSum = 0;
    private CurrentData[] currentDatas = new CurrentData[5];

    public CurrentBudgeteer() {

        currentDatas[0] = new CurrentData(CurrentConstants.minShooterCurrent, 0);
        currentDatas[1] = new CurrentData(CurrentConstants.minIntakeCurrentAmps, 1);
        currentDatas[2] = new CurrentData(CurrentConstants.minArmCurrentAmps * 2, 2);
        currentDatas[3] = new CurrentData(CurrentConstants.minShintakeCurrentAmps, 3);
        currentDatas[4] = new CurrentData(
                (CurrentConstants.minDriveCurrentAmps + CurrentConstants.minTurningCurrentAmps) * 4, 4);

    }

    private void updateSubsystemCurrent(int indexByPrioirty, double amps) {
        currentDatas[indexByPrioirty].setCurrent(amps);
    }

    private void updateCurrentSum() {
        currentSum = 0;
        for (CurrentData data : currentDatas) {
            currentSum += data.getCurrent();
        }
    }

    @Override
    public void periodic() {
        // swerve
        updateSubsystemCurrent(4, Robot.example.getCurrentSum());
        // shintake
        updateSubsystemCurrent(3, 0);
        // arm
        updateSubsystemCurrent(2, Robot.example2.getCurrentSum());
        // intake
        updateSubsystemCurrent(1, 0);
        // shooter
        updateSubsystemCurrent(0, Robot.example3.getCurrentSum());

        updateCurrentSum();

        if (currentSum > CurrentConstants.maxCurrentDrawAmps) {
            double currentOverun = currentSum - CurrentConstants.maxCurrentDrawAmps;
            double[] availibleCurrent = { 0, 0, 0, 0, 0 };
            for (CurrentData data : currentDatas) {
                availibleCurrent[data.getPriority()] = data.getCurrent() - data.getMinimumCurrent();
            }

        }
    }

}

class CurrentData {
    private double minimumCurrent;
    private int priority;
    private double current = 0;

    public CurrentData(double minimumCurrent, int priority) {
        this.minimumCurrent = minimumCurrent;
        this.priority = priority;
    }

    public double getMinimumCurrent() {
        return minimumCurrent;
    }

    public int getPriority() {
        return priority;
    }

    public double getCurrent() {
        return current;
    }

    public void setCurrent(double amps) {
        current = amps;
    }

}