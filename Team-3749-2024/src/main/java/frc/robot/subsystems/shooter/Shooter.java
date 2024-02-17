package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.ShooterIO.ShooterData;
import frc.robot.utils.Constants;

public class Shooter extends SubsystemBase {

    private ShooterIO shooterIO;
    private ShooterData data = new ShooterData();
    private PIDController shooterController = new PIDController(Constants.ShintakeConstants.shooterPID.kP, Constants.ShintakeConstants.shooterPID.kI, Constants.ShintakeConstants.shooterPID.kD);
    private SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(0, 1);  
    private double shooterVelocity = 0;
    private double armAngle = 0;
    private double antiShooterRubbing = 0; //i dont actually know what angle the arm is at when it's resting 
    //and the wheels are touching the body of the robot but thats what this represents

    public Shooter() 
        {
        shooterIO = new ShooterSparkMax();
         if(Robot.isSimulation()) 
         {
            shooterIO = new ShooterSim();
         }
    }

   public void setShooterVelocity(double velocity)
   {
    this.shooterVelocity = velocity;
   }

   public void moveShooter()
   {
    if(armAngle == antiShooterRubbing)
    {
        return; //the wheels might still have inertia from spinning but it shouldn't be a problem?
    }
    shooterIO.setVoltage(
        shooterController.calculate(data.leftShooterVelocityRadPerSec,shooterVelocity) + shooterFF.calculate(shooterVelocity),
        shooterController.calculate(data.rightShooterVelocityRadPerSec,shooterVelocity) + shooterFF.calculate(shooterVelocity)
    );
   }

    @Override
    public void periodic() {
        shooterIO.updateData(data);
        SmartDashboard.putNumber("shooterVelocity", data.leftShooterVelocityRadPerSec);
        SmartDashboard.putNumber("shooterVolts", data.leftShooterVolts);
        SmartDashboard.putNumber("shooterTemp", data.leftShooterTempCelcius);
    }

}
