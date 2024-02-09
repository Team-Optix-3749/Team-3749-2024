package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.intake.IntakeIO.IntakeData;
import frc.robot.utils.Constants;

public class Intake extends SubsystemBase {

    private IntakeIO intakeModule;
    private IntakeData data = new IntakeData();
    private PIDController intakeController = new PIDController(Constants.ShintakeConstants.intakePID.kP,Constants.ShintakeConstants.intakePID.kI,Constants.ShintakeConstants.intakePID.kD);
        
    private SimpleMotorFeedforward intakeFF = new SimpleMotorFeedforward(0, 1);
    private double intakeVelocity = 0;

    public Intake() 
        {
        intakeModule = new IntakeSparkMax();
         if(Robot.isSimulation()) 
         {
            intakeModule = new IntakeSim();
         }
    }

    public void setIntakeVelocity(double velocity)
   {
    this.intakeVelocity = velocity;
   }

   public void moveIntake()
   {
    intakeModule.setVoltage(
        intakeController.calculate(intakeModule.getIntakeEncoder(),intakeVelocity) + intakeFF.calculate(intakeVelocity)
    );
   }

    @Override
    public void periodic() {
        intakeModule.updateData(data);
        SmartDashboard.putNumber("intakeVolts",data.intakeVolts);
        SmartDashboard.putNumber("intakeVelocity", data.intakeVelocityRadPerSec);
    }

}
