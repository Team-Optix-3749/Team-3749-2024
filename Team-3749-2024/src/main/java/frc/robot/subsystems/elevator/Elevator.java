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

    private double elevatorPosition;

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

      // Run calculations for necessary voltage with PID Controller

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
            elevatorPosition = Constants.Elevator.elevatorMaxHeight;
        } else if (position < Constants.Elevator.elevatorMinHeight) {
            elevatorPosition = Constants.Elevator.elevatorMinHeight;
        }

        elevatorPosition = position;
    }

    // TODO: Add Feedforward
    public double ffcalculate(double velocity){
        return 0.0;
    }

    // runs every 0.02 sec
    // TODO: add code for periodic adjustment
    @Override
    public void periodic(){
    }

    // TODO: Add Simulation
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}