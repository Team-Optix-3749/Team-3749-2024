package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.utils.Constants.Sim;

public class ArmSim implements ArmIO {

    private SingleJointedArmSim armSim = new SingleJointedArmSim(DCMotor.getNEO(2),
        150,
        4,
        .93, 
        Units.degreesToRadians(0),
        Units.degreesToRadians(100),
        true,
        Units.degreesToRadians(0));

    private double appliedVolts = 0;
    private double previousVelocity = 0;

    public ArmSim() {
        System.out.println("[Init] Creating ExampleIOSim");
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
        // System.out.println(appliedVolts);

        data.leftCurrentAmps = Math.abs(armSim.getCurrentDrawAmps());
        data.rightCurrentAmps = Math.abs(armSim.getCurrentDrawAmps());

        data.leftTempCelcius = 0;
        data.rightTempCelcius = 0;
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -8.0, 8.0);
        armSim.setInputVoltage(appliedVolts);
    }

}
