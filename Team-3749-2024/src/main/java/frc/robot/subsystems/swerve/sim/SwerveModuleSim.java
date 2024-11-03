package frc.robot.subsystems.swerve.sim;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;
import frc.robot.utils.MiscConstants.Sim;

/* 
Very closely inspired by 6328's Swerve Sim code,
 https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/subsystems/drive/ModuleIOSim.java
*/
public class SwerveModuleSim implements SwerveModuleIO {
    private FlywheelSim driveSim = new FlywheelSim(DCMotor.getNEO(1), ModuleConstants.driveMotorGearRatio,
            0.02931);
    private FlywheelSim turnSim = new FlywheelSim(DCMotor.getNEO(1), 
        ModuleConstants.turnMotorGearRatio, 0.04);

    private double turnPositionRad = 0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public SwerveModuleSim() {
        System.out.println("[Init] Creating ModuleIOSim");
    }

    @Override
    public void updateData(ModuleData data) {
        // update sim values
        driveSim.update(Sim.loopPeriodSec);
        turnSim.update(Sim.loopPeriodSec);

        // how far have we turned in the previous loop?
        double angleDiffRad = turnSim.getAngularVelocityRadPerSec() * Sim.loopPeriodSec;
        // update our angle variables
        turnPositionRad += angleDiffRad;
        // keep our absolute position within 0-2 pi
        while (turnPositionRad < 0) {
            turnPositionRad += 2.0 * Math.PI;
        }
        while (turnPositionRad > 2.0 * Math.PI) {
            turnPositionRad -= 2.0 * Math.PI;
        }
        // distance traveled + Rad/Time * Time * diameter
        data.drivePositionM = data.drivePositionM
                + (driveSim.getAngularVelocityRadPerSec() * 0.02 * ModuleConstants.wheelDiameterMeters) / 2;
        data.driveVelocityMPerSec = driveSim.getAngularVelocityRadPerSec() * ModuleConstants.wheelDiameterMeters / 2;
        data.driveAppliedVolts = driveAppliedVolts;
        data.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());
        data.driveTempCelcius = 0;

        data.turnAbsolutePositionRad = turnPositionRad;
        data.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
        data.turnAppliedVolts = turnAppliedVolts;
        data.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());
        data.turnTempCelcius = 0;

    }

    @Override
    public void setDriveVoltage(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -DriveConstants.maxMotorVolts,
                DriveConstants.maxMotorVolts);
        driveSim.setInputVoltage(driveAppliedVolts);
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -DriveConstants.maxMotorVolts,
                DriveConstants.maxMotorVolts);
        turnSim.setInputVoltage(turnAppliedVolts);
    }
}