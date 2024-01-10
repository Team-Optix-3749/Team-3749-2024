package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.example.ExampleIO.ExampleData;
import frc.robot.subsystems.example.ExampleSim;
import frc.robot.utils.Constants;

public class Elevator extends SubsystemBase {

    private final CANSparkMax motorOne = new CANSparkMax(Constants.Elevator.elevatorMotorOneID,
      MotorType.kBrushless);
  private final CANSparkMax motorTwo = new CANSparkMax(Constants.Elevator.elevatorMotorTwoID,
      MotorType.kBrushless);
    // private Exampl

    // Constructor
    public Example(){
        if (Robot.isReal()){
            exampleIO = new ExampleSim();
        }
    }

    // runs every 0.02 sec
    @Override
    public void periodic(){

    }
    
}