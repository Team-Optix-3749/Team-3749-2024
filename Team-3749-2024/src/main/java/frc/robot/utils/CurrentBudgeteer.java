package frc.robot.utils;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CurrentBudgeteer extends SubsystemBase {

    private double totalCurrent = 0;
    private HashMap<String, CurrentData> currentData;

    public CurrentBudgeteer(){
        currentData.put("Swerve", new CurrentData(20, 5));
        currentData.put("Arm", new CurrentData(20, 5));

    }




    public void setSwerveCurrent(double amps) {
        swerveCurrent = amps;
    }

    public void setArmCurrent(double amps) {
        armCurrent = amps;
    }

    public void setShintakeCurrent(double amps) {
        shintakeCurrent = amps;
    }

    public void setShooterCurrent(double amps) {
        shooterCurrent = amps;
    }

    public void setIntakeCurrent(double amps) {
        intakeCurrent = amps;
    }

    @Override
    public void periodic() {
        totalCurrent = swerveCurrent + armCurrent + shintakeCurrent + intakeCurrent + shooterCurrent;

    }

    private class CurrentData {
        private double minimumCurrent;
        private double priority;
        private double current = 0;

        public CurrentData(double minimumCurrent, double priority) {
            this.minimumCurrent = minimumCurrent;
            this.priority = priority;
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

        public void setCurrent() {
            current = 0;
        }
    }
}