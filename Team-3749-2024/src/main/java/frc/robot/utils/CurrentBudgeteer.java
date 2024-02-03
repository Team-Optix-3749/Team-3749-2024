package frc.robot.utils;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utils.Constants.ElectricalConstants;

public class CurrentBudgeteer extends SubsystemBase {
    private PowerDistribution powerDistribution = new PowerDistribution(1, ModuleType.kRev);
    private CurrentData[] currentDatas = new CurrentData[5];
    private Double[] currentReductions = { 0.0, 0.0, 0.0, 0.0, 0.0 };
    private final ShuffleData<Double> currentSumLog = new ShuffleData<Double>("Current Budgetteer", "Current Sum",
            0.0);
    private final ShuffleData<Double[]> currentReductionsLog = new ShuffleData<Double[]>("Current Budgetteer",
            "Current Reductions",
            currentReductions);

    public CurrentBudgeteer() {


        currentDatas[0] = new CurrentData(ElectricalConstants.example5CurrentLimit, 0,
                Robot.example1);
        currentDatas[1] = new CurrentData(ElectricalConstants.example5CurrentLimit, 1,
                Robot.example2);
        currentDatas[2] = new CurrentData(ElectricalConstants.example5CurrentLimit, 2,
                Robot.example3);
        currentDatas[3] = new CurrentData(ElectricalConstants.example5CurrentLimit, 3,
                Robot.example4);

        currentDatas[4] = new CurrentData(ElectricalConstants.example5CurrentLimit, 4,
                Robot.example5);
 
    }



    private Double[] calcExcessCurrent() {
        double currentOverun = powerDistribution.getTotalCurrent() - ElectricalConstants.maxCurrentDrawAmps;
        Double[] tempCurrentReductions = { 0.0, 0.0, 0.0, 0.0, 0.0 };


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
                double availibleCurrent = currentDatas[priotiyIndex].getCurrent()
                        - currentDatas[priotiyIndex].getMinimumCurrent();

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



        currentReductions = calcExcessCurrent();
        for (int i = 0; i<=4; i++) {
            currentDatas[i].reduceCurrentSum(currentReductions[i]);
        }

        currentSumLog.set(powerDistribution.getTotalCurrent() );
        currentReductionsLog.set(currentReductions);
        SmartDashboard.putNumber("max current robot", ElectricalConstants.maxCurrentDrawAmps);

    }

}

class CurrentData {
    private CurrentBudgettedSubsystem subsystem;
    private double minimumCurrent;
    private int priority;
    private double current = 0;

    public CurrentData(double minimumCurrent, int priority, CurrentBudgettedSubsystem subsystem) {
        this.minimumCurrent = minimumCurrent;
        this.priority = priority;
        this.subsystem = subsystem;

    }

    public double getMinimumCurrent() {
        return minimumCurrent;
    }

    public double getPriority() {
        return priority;
    }

    public double getCurrent() {
        return current;
    }

    public void reduceCurrentSum(double currentReduction) {
        subsystem.reduceCurrentSum(currentReduction);
    }

    public void updateCurrent(){
        setCurrent(subsystem.getEstimatedCurrentDraw());
    }
        private void setCurrent(double amps) {
        current = amps;
    }
}