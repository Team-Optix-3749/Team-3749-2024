// Importing necessary libraries
package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.limelight.api.ControlMode.*;
import org.limelight.api.Limelight;
import org.limelight.api.NetworkTableLookupException;

/**
 * Encapsulated Limelight object used for vision processing and alignment
 * 
 * @author Your Name
 */
public class NotPhoton extends SubsystemBase {

    // Limelight instance
    private final Limelight limelight;

    // NetworkTables entry for pipeline
    private final NetworkTable limelightTable;

    // Constructor
    public NotPhoton () {
        // Initializing Limelight instance
        limelight = new Limelight();
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    // Method to set the LED mode for the Limelight
    public void setLED(LEDMode ledMode) {
        try {
            limelight.getLED().set(ledMode);
        } catch (NetworkTableLookupException e) {
            System.out.println("Error setting LED mode: " + e.getMessage());
        }
    }

    // Method to get the pipeline index from Limelight
    public int getPipeline() {
        try {
            return limelight.getPipeline();
        } catch (NetworkTableLookupException e) {
            System.out.println("Error getting pipeline index: " + e.getMessage());
            return -1;
        }
    }

    // Method to set the pipeline index for Limelight
    public void setPipeline(int index) {
        try {
            limelight.setPipeline(index);
        } catch (NetworkTableLookupException e) {
            System.out.println("Error setting pipeline index: " + e.getMessage());
        }
    }

    // Method to get whether the Limelight has a valid target
    public boolean hasTarget() {
        try {
            return limelight.hasTarget();
        } catch (NetworkTableLookupException e) {
            System.out.println("Error checking target presence: " + e.getMessage());
            return false;
        }
    }

    // Method to get the horizontal offset to the target
    public double getHorizontalOffset() {
        try {
            return limelight.getdegRotationToTarget();
        } catch (NetworkTableLookupException e) {
            System.out.println("Error getting horizontal offset: " + e.getMessage());
            return 0.0;
        }
    }

    // Method to get the vertical offset to the target
    public double getVerticalOffset() {
        try {
            return limelight.getdegVerticalToTarget();
        } catch (NetworkTableLookupException e) {
            System.out.println("Error getting vertical offset: " + e.getMessage());
            return 0.0;
        }
    }

    // Method for logging information
    public void logging() {
        SmartDashboard.putNumber("Pipeline", getPipeline());
    }

    // Overridden periodic method for logging during each robot loop iteration
    @Override
    public void periodic() {
        logging();
    }
}
