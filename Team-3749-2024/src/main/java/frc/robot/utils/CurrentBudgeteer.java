package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utils.Constants.CurrentConstants;

public class CurrentBudgeteer extends SubsystemBase {

    private int currentSum = 0;
    private CurrentData[] currentDatas = new CurrentData[5];
    private int[] currentReductions = { 0, 0, 0, 0, 0 };
    private final ShuffleData<Integer> currentSumLog = new ShuffleData("Current Budgetteer", "Current Sum", currentSum);
    private final ShuffleData<int[]> currentReductionsLog = new ShuffleData("Current Budgetteer", "Current Reductions",
            currentReductions);

    public CurrentBudgeteer() {

        // currentDatas[0] = new CurrentData(CurrentConstants.minShooterCurrent, 0,
        // () -> Robot.example3.reduceCurrentSum(() -> getCurrentReduction(0)));

        currentDatas[1] = new CurrentData(0, 1, () -> {
        });
        currentDatas[2] = new CurrentData(0, 2, () -> {
        });
        currentDatas[3] = new CurrentData(0, 3, () -> {
        });
        currentDatas[0] = new CurrentData(0, 4, () -> {
        });

        currentDatas[4] = new CurrentData(CurrentConstants.example5CurrentLimit, 4,
                () -> Robot.example5.reduceCurrentSum(() -> getCurrentReduction(4)));
        // currentDatas[2] = new CurrentData(CurrentConstants.minArmCurrentAmps * 2, 2,
        // () -> Robot.example2.reduceCurrentSum(() -> getCurrentReduction(2)));
        // currentDatas[3] = new CurrentData(CurrentConstants.minShintakeCurrentAmps, 3,
        // () -> Robot.example4.reduceCurrentSum(() -> getCurrentReduction(3)));
        // currentDatas[4] = new CurrentData(
        // (CurrentConstants.minDriveCurrentAmps +
        // CurrentConstants.minTurningCurrentAmps) * 4, 4,
        // () -> Robot.example.reduceCurrentSum(() -> currentReductions[4]));

    }

    private int getCurrentReduction(int indexByPrioirty) {
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

    private int[] calcExcessCurrent() {
        int currentOverun = currentSum - CurrentConstants.maxCurrentDrawAmps;
        int[] tempCurrentReductions = { 0, 0, 0, 0, 0 };

        System.out.println(currentOverun == 0);

        if (currentOverun <= 0) {
            for (int i = 4; i >= 0; i--) {
                if (currentOverun + currentReductions[i] <= 0) { // if the currentoverrun+reduced current still leaves
                                                                 // some availible
                    currentOverun += currentReductions[i];
                    // the reductions are already set to 0
                } else {
                    tempCurrentReductions[i] = currentOverun + currentReductions[i];
                    currentOverun += currentReductions[i] - tempCurrentReductions[i];
                }
            }
        } else {
            int priotiyIndex = 4;
            while (currentOverun > 0 && priotiyIndex >= 0) {
                int availibleCurrent = (int) (currentDatas[priotiyIndex].getCurrent()
                        - currentDatas[priotiyIndex].getMinimumCurrent());

                if (currentOverun > availibleCurrent) {
                    tempCurrentReductions[priotiyIndex] = availibleCurrent;
                    currentOverun -= availibleCurrent;
                } else if (currentOverun <= availibleCurrent) {
                    tempCurrentReductions[priotiyIndex] = currentOverun;
                    currentOverun = 0;
                }
                priotiyIndex -= 1;

            }
        }
        return tempCurrentReductions;
    }

    @Override
    public void periodic() {

        // swerve
        // updateSubsystemCurrent(4, Robot.example.getCurrentSum());
        // shintake
        updateSubsystemCurrent(3, 0);
        // arm
        // updateSubsystemCurrent(2, Robot.example2.getCurrentSum());
        // // intake
        // updateSubsystemCurrent(1, 0);
        // // shooter
        // updateSubsystemCurrent(0, Robot.example3.getCurrentSum());

        updateCurrentSum();

        currentReductions = calcExcessCurrent();
        for (CurrentData data : currentDatas) {
            data.reduceCurrentSum();
        }
        for (int current : currentReductions) {
            System.out.println(current);
        }
        // System.out.println(currentSum);

        currentSumLog.set(currentSum);
        currentReductionsLog.set(currentReductions);

    }

}

class CurrentData {
    private Runnable reduceSubystemCurrentSumRunnable;
    private int minimumCurrent;
    private int priority;
    private double current = 0;

    public CurrentData(int minimumCurrent, int priority, Runnable reduceSubystemCurrentSumRunnable) {
        this.minimumCurrent = minimumCurrent;
        this.priority = priority;
        this.reduceSubystemCurrentSumRunnable = reduceSubystemCurrentSumRunnable;

    }

    public int getMinimumCurrent() {
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