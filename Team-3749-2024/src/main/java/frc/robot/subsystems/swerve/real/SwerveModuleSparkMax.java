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
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.ModuleConstants;
import frc.robot.utils.Constants.Sim;

public class SwerveModuleSparkMax implements SwerveModuleIO {
    private CANSparkMax driveMotor;
    private CANSparkMax turnMotor;

    private RelativeEncoder driveEncoder;
    private CANcoder turnEncoder;

    private double driveAppliedVolts;
    private double turnAppliedVolts;

    public SwerveModuleSparkMax(int index) {
        driveMotor = new CANSparkMax(Constants.DriveConstants.driveMotorPorts[index], CANSparkMax.MotorType.kBrushless);
        turnMotor = new CANSparkMax(Constants.DriveConstants.turningMotorPorts[index],
                CANSparkMax.MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = new CANcoder(Constants.DriveConstants.absoluteEncoderPorts[index]);

        driveMotor.setInverted(DriveConstants.driveEncoderReversed[index]);
        driveEncoder.setInverted(DriveConstants.driveEncoderReversed[index]);
        driveEncoder.setPositionConversionFactor((1 / ModuleConstants.driveMotorGearRatio) * Math.PI
                * ModuleConstants.wheelDiameterMeters);
        driveEncoder.setVelocityConversionFactor((1 / ModuleConstants.driveMotorGearRatio) * (Math.PI
                * ModuleConstants.wheelDiameterMeters) * (1 / 60));

        turnMotor.setInverted(Constants.DriveConstants.turningEncoderReversed[index]);

        driveMotor.setSmartCurrentLimit(Constants.DriveConstants.driveMotorStallLimit,
                Constants.DriveConstants.driveMotorFreeLimit);
        turnMotor.setSmartCurrentLimit(Constants.DriveConstants.turnMotorStallLimit,
                Constants.DriveConstants.turnMotorFreeLimit);

    };

    @Override
    public void updateData(ModuleData data) {
        
        driveAppliedVolts = driveMotor.getBusVoltage();
        turnAppliedVolts = turnMotor.getBusVoltage();
        
        data.drivePositionM = driveEncoder.getPosition();
        data.driveVelocityMPerSec = driveEncoder.getVelocity();
        data.driveAppliedVolts = driveAppliedVolts;
        data.driveCurrentAmps = Math.abs(driveMotor.getOutputCurrent());
        data.driveTempCelcius = driveMotor.getMotorTemperature();
        
        data.turnAbsolutePositionRad = Units.rotationsToRadians(turnEncoder.getPosition().getValueAsDouble());
        data.turnVelocityRadPerSec = Units.rotationsToRadians(turnEncoder.getVelocity().getValueAsDouble());
        data.turnAppliedVolts = turnAppliedVolts;
        data.turnCurrentAmps = Math.abs(turnMotor.getOutputCurrent());
        data.turnTempCelcius = turnMotor.getMotorTemperature();
    };

    @Override
    public void setDriveVoltage(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -Constants.DriveConstants.maxMotorVolts,
                Constants.DriveConstants.maxMotorVolts);
        driveMotor.setVoltage(driveAppliedVolts);
    };

    @Override
    public void setTurnVoltage(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -Constants.DriveConstants.maxMotorVolts,
                Constants.DriveConstants.maxMotorVolts);
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
        return turnEncoder.getPosition().getValueAsDouble();
    };

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    };

    public double getTurningVelocity() {
        return turnEncoder.getVelocity().getValueAsDouble();
    };

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(0);
    };
}