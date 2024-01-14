package frc.robot.subsystems.swerve.real;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveModuleIO.ModuleData;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.ModuleConstants;
import frc.robot.utils.Constants.Sim;

public class SwerveModuleSparkMax implements SwerveModuleIO {

    private CANSparkMax driveMotor;
    private CANSparkMax turnMotor; 

    private CANcoder absoluteEncoder;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder turnEncoder;

    private double drivePositionRad;
    private double turnPositionRad;

    private double driveAppliedVolts;
    private double turnAppliedVolts;

    public SwerveModuleSparkMax(int driveID, int turnID, int absoluteEncoderID) { // taken from drive constants
        
        driveMotor = new CANSparkMax(driveID, CANSparkMax.MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnID, CANSparkMax.MotorType.kBrushless);

        absoluteEncoder = new CANcoder(absoluteEncoderID);
        turnPositionRad = Units.rotationsToRadians(absoluteEncoder.getAbsolutePosition().getValueAsDouble());
        /* absoluteEncoder.configMagnetOffset(absoluteEncoderOffset); */ //deprecated in phoenixv6

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        drivePositionRad = Units.rotationsToRadians(driveEncoder.getPosition());
        turnPositionRad = Units.rotationsToRadians(turnEncoder.getPosition());

        driveMotor.setSmartCurrentLimit(30, 60);
        turnMotor.setSmartCurrentLimit(30, 60);

    };

    @Override
    public void updateData(ModuleData data) {

        double driveRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity());
        double turnRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(turnEncoder.getVelocity());

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        drivePositionRad = driveEncoder.getPosition();
        turnPositionRad = turnEncoder.getPosition();

        driveAppliedVolts = driveMotor.getBusVoltage();
        turnAppliedVolts = turnMotor.getBusVoltage();
  
        data.drivePositionM = data.drivePositionM + (turnRadPerSec * 0.02 * ModuleConstants.wheelDiameterMeters) / 2;
        data.driveVelocityMPerSec = driveRadPerSec * ModuleConstants.wheelDiameterMeters / 2;
        data.driveAppliedVolts = driveAppliedVolts;
        data.driveCurrentAmps = Math.abs(driveMotor.getOutputCurrent());
        data.driveTempCelcius = 0;
        data.turnAbsolutePositionRad = turnPositionRad;
        data.turnVelocityRadPerSec = turnRadPerSec;
        data.turnAppliedVolts = turnAppliedVolts;
        data.turnCurrentAmps = Math.abs(turnMotor.getOutputCurrent());
        data.turnTempCelcius = 0;

    };

    @Override
    public void setDriveVoltage(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -8.0, 8.0);
        driveMotor.setVoltage(driveAppliedVolts);
    };

    @Override
    public void setTurnVoltage(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -8.0, 8.0);
        turnMotor.setVoltage(turnAppliedVolts);
    };

    public void stopMotors() {
        driveMotor.setVoltage(0);
        turnMotor.setVoltage(0);
    };

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    };

    public double getTurningPosition() {
        return turnEncoder.getPosition();
    };

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    };

    public double getTurningVelocity() {
        return turnEncoder.getVelocity();
    };

    public double getAbsoluteEncoderRad() {
        return turnPositionRad;
    };
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(getAbsoluteEncoderRad());
    };
}