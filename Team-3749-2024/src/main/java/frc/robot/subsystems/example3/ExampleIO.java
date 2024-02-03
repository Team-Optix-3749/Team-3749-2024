package frc.robot.subsystems.example3;

// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.


/***
 * @author Noah Simon
 *         An example for how to set up an IO class. This is to handle your
 *         motors and their encoders for a subsytems and sets up for easy
 *         logging, easy simulation, and future advantage kit implementation
 */
public interface ExampleIO {
    public static class ExampleData {
        // each of these for each motor
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempCelcius = 0.0;
        
    }
    /** Updates the set of loggable inputs. */
    public default void updateData(ExampleData data) {

    }

    /** Run the drive motor at the specified voltage. */
    public default void setVoltage(double volts) {
        
    }
    public default void setMaxOutput(double maxOutputPercent){

    }

    /** Enable or disable brake mode on the drive motor. */
    public default void setBrakeMode(boolean enable) {
    }



    
}