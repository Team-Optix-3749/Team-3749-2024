package frc.robot.utils;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utils.Constants.CurrentConstants;

public class CurrentBudgeteer extends SubsystemBase {

    private double currentSum = 0;
    private CurrentData[] currentDatas = new CurrentData[5];
    private double[] currentReductions = { 0, 0, 0, 0, 0 };

    public CurrentBudgeteer() {

        currentDatas[0] = new CurrentData(CurrentConstants.minShooterCurrent, 0,
                () -> Robot.example3.reduceCurrentSum(() -> getCurrentReduction(0)));

        currentDatas[1] = new CurrentData(CurrentConstants.minIntakeCurrentAmps, 1,
                () -> Robot.example3.reduceCurrentSum(() -> getCurrentReduction(1)));
        currentDatas[2] = new CurrentData(CurrentConstants.minArmCurrentAmps * 2, 2,
                () -> Robot.example3.reduceCurrentSum(() -> getCurrentReduction(2)));
        currentDatas[3] = new CurrentData(CurrentConstants.minShintakeCurrentAmps, 3,
                () -> Robot.example3.reduceCurrentSum(() -> getCurrentReduction(3)));
        currentDatas[4] = new CurrentData(
                (CurrentConstants.minDriveCurrentAmps + CurrentConstants.minTurningCurrentAmps) * 4, 4,
                () -> Robot.example3.reduceCurrentSum(() -> getCurrentReduction(4)));

    }

    private double getCurrentReduction(int indexByPrioirty) {
        return currentReductions[indexByPrioirty];
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

    private double[] calcExcessCurrent() {
        double currentOverun = currentSum - CurrentConstants.maxCurrentDrawAmps;
        double[] excessCurrent = { 0, 0, 0, 0, 0 };
        int priotiyIndex = 4;
        while (currentOverun > 0) {
            double availibleCurrent = currentDatas[priotiyIndex].getCurrent()
                    - currentDatas[priotiyIndex].getMinimumCurrent();
            if (currentOverun > availibleCurrent) {
                excessCurrent[priotiyIndex] = availibleCurrent;
                currentOverun -= availibleCurrent;
            } else if (currentOverun < availibleCurrent) {
                excessCurrent[priotiyIndex] = currentOverun;
                currentOverun = 0;
            }
            priotiyIndex -= 1;
        }
        return excessCurrent;
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

        double[] excessCurrent = calcExcessCurrent();

        double indexByPrioirty = 4;
        for (CurrentData data : currentDatas) {

        }

    }

}

class CurrentData {
    private Runnable reduceSubystemCurrentSumRunnable;
    private double minimumCurrent;
    private int priority;
    private double current = 0;

    public CurrentData(double minimumCurrent, int priority, Runnable reduceSubystemCurrentSumRunnable) {
        this.minimumCurrent = minimumCurrent;
        this.priority = priority;
        this.reduceSubystemCurrentSumRunnable = reduceSubystemCurrentSumRunnable;

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


    public void reduceCurrentSum() {
        reduceSubystemCurrentSumRunnable.run();
    }

}