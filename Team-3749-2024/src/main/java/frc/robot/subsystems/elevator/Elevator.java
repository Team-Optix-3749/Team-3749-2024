package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
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

    // Constructor
    public Elevator(){
        motorTwo.setInverted(true);

        // This is for gear ratio stuff
        // gear ratio * 1 / circumfrance rotarty bar * 1 / total length elevator
        // motorOneEncoder.setPositionConversionFactor(1/225);
        // motorTwoEncoder.setPositionConversionFactor(1 / 225);

        // So it doesn't go too fast
        motorOne.setSmartCurrentLimit(40);
        motorTwo.setSmartCurrentLimit(40);
    }

    public void stop() {
        motorOne.set(0);
        motorTwo.set(0);
      }

    public void setVoltage(double volts) {
        motorOne.setVoltage(volts);
        motorTwo.setVoltage(volts);
  }

    // runs every 0.02 sec
    @Override
    public void periodic(){

    }
    
}