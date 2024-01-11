package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
public class Elevator extends SubsystemBase {
    private final CANSparkMax motorOne = new CANSparkMax(Constants.Elevator.elevatorMotorOneID,
      MotorType.kBrushless);
    private final CANSparkMax motorTwo = new CANSparkMax(Constants.Elevator.elevatorMotorTwoID,
      MotorType.kBrushless);
    private final RelativeEncoder motorOneEncoder = motorOne.getEncoder();
    private final RelativeEncoder motorTwoEncoder = motorTwo.getEncoder();

    private final PIDController elevatorController = new PIDController(0.55, 0, 0);

    private final DoubleSupplier pivotAngDoubleSupplier;

    // change to setpoints
    private double desiredElevatorPosition;

    // move these to constants / tune these
    // feed forward constants
    private final double ks = 0.3;
    private final double kg = 0.06;
    // extension
    private final double ke = -0.2;
    // private final double cgOutLengthInches =
    // pivot rotation
    private final double kr = -.1;

    // Constructor
    public Elevator(DoubleSupplier pivotAngDoubleSupplier){
        this.pivotAngDoubleSupplier = pivotAngDoubleSupplier;
        
        motorTwo.setInverted(true);
        // This is for gear ratio stuff
        // gear ratio * 1 / circumfrance rotarty bar * 1 / total length elevator
        // motorOneEncoder.setPositionConversionFactor(1/225);
        motorTwoEncoder.setPositionConversionFactor(1/225);

        // So it doesn't go too fast
        motorOne.setSmartCurrentLimit(40);
        motorTwo.setSmartCurrentLimit(40);

        motorOne.setIdleMode(IdleMode.kCoast);
        motorTwo.setIdleMode(IdleMode.kCoast);
    }

    // Stops the elevator motors
    public void stop() {
        motorOne.set(0);
        motorTwo.set(0);
      }

    // Sets the voltage of the elevator motors
    public void setVoltage(double volts) {
        SmartDashboard.putNumber("Elevator Voltage", volts);

        motorOne.setVoltage(volts);
        motorTwo.setVoltage(volts);
    }

    // Runs the elevator
    public void runElevator() {
        double voltage = 0;
        if (Math.abs(desiredElevatorPosition - getElevatorPositionInches()) > 0.25) { // tune threshol (i think this is what this is) and add to constants
          voltage = elevatorController.calculate(getElevatorPositionInches(),
              desiredElevatorPosition);
    
            // tune voltage limit and add to constants
            double maxVoltage = 6;
            if (voltage > maxVoltage) {
            voltage = maxVoltage;
            }
            if (voltage < -maxVoltage) {
            voltage = -maxVoltage;
            }
        }
    
        // tune limits, and add to constants
        if (getElevatorPositionInches() < 0.15 && voltage < 0) {
          voltage = 0;
        } else if (getElevatorPositionInches() > 42 && voltage > 0) {
          voltage = 0;
        }
        voltage += ffcalculate(1);
    
        SmartDashboard.putNumber("Elevator Voltage", voltage);
        setVoltage(voltage);
    }

    // Returns the position of the elevator in inches
    // Average of the two encoders * 1/58 (the raw reading at max extension) * 42.5
    // (the max extenstion in inches)
    public double getElevatorPositionInches() {
        return ((motorOneEncoder.getPosition() + motorTwoEncoder.getPosition()) / 2 * 1 / 58 * 42.5); // TODO: fix conversion values
    }

    // Generic position setter for now
    // Possibly create setpoints?
    // Sets desired elevator position
    public void setElevatorPosition(double position) {
        if (position > Constants.Elevator.elevatorMaxHeight) {
            desiredElevatorPosition = Constants.Elevator.elevatorMaxHeight;
        } else if (position < Constants.Elevator.elevatorMinHeight) {
            desiredElevatorPosition = Constants.Elevator.elevatorMinHeight;
        }

        desiredElevatorPosition = position;
    }

    // TODO: Add Feedforward
    public double ffcalculate(double velocity){
        double total = 0;
        total += ks * Math.signum(velocity);
        total += kg;
        total += kr * Math.sin(pivotAngDoubleSupplier.getAsDouble() / 180 * Math.PI); // fix pivot kr in feedforward

        // BS center of mass constant forcespring torque crap
        if (getElevatorPositionInches() > 4 && getElevatorPositionInches() < 10) {
        total += ke;
        }
        if (getElevatorPositionInches() > 10) {
        total += ke / 2;
        }
        return total;
    }

    // runs every 0.02 sec
    // TODO: add code for periodic adjustment
    @Override
    public void periodic(){
        runElevator();

        // SmartDashboard stuff, can be removed  
        SmartDashboard.putNumber("Elevator Motor 1 Bus Voltage", motorOne.getBusVoltage());
        SmartDashboard.putNumber("Elevator Motor 2 Bus Voltage", motorTwo.getBusVoltage());
        
        SmartDashboard.putNumber("Elevator Motor 1 Current", motorOne.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Motor 2 Current", motorTwo.getOutputCurrent());
    
        SmartDashboard.putNumber("Elevator Position", getElevatorPositionInches());
    }

    // TODO: Add Simulation
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}