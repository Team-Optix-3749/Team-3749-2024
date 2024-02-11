package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.utils.Constants.Sim;

public class ArmSim implements ArmIO {

    // private SingleJointedArmSim armSimOut = new SingleJointedArmSim(DCMotor.getNEO(2),
    //     333.333,
    //     1.763, //6025.4
    //     Units.inchesToMeters(34), //34 in    
    //     Units.degreesToRadians(-170),
    //     Units.degreesToRadians(170),
    //     true,
    //     Units.degreesToRadians(0));

    private SingleJointedArmSim armSimIn = new SingleJointedArmSim(DCMotor.getNEO(2),
        333.333,
        0.755, //2578.65
        Units.inchesToMeters(21.1), //  21.1 in
        Units.degreesToRadians(-170),
        Units.degreesToRadians(170),
        true,
        Units.degreesToRadians(0));


    private SingleJointedArmSim armSim;


    private double appliedVolts = 0;
    private double previousVelocity = 0;

    public ArmSim() {
        System.out.println("[Init] Creating ExampleIOSim");

        armSim = armSimIn;
    }

    @Override
    public void updateData(ArmData data) {
        previousVelocity = data.velocityRadPerSec;

        // update sim values every 0.02 sec
        armSim.update(Sim.loopPeriodSec);

        // distance traveled + Rad/Time * Time * diameter
        data.positionRad = armSim.getAngleRads();

        data.velocityRadPerSec =armSim.getVelocityRadPerSec();

        data.accelerationRadPerSecSquared = (armSim.getVelocityRadPerSec() - previousVelocity)/Sim.loopPeriodSec;

        data.appliedVolts = appliedVolts;
        System.out.println(appliedVolts);

        data.leftCurrentAmps = Math.abs(armSim.getCurrentDrawAmps());
        data.rightCurrentAmps = Math.abs(armSim.getCurrentDrawAmps());

        data.leftTempCelcius = 0;
        data.rightTempCelcius = 0;
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        armSim.setInputVoltage(appliedVolts);
    }

}
