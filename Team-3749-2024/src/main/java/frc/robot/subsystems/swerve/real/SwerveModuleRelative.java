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
import frc.robot.utils.Constants.ModuleConstants;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.Sim;

public class SwerveModuleRelative implements SwerveModuleIO {
    private CANSparkMax driveMotor;
    private CANSparkMax turnMotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder turnEncoder;

    private double driveAppliedVolts;
    private double turnAppliedVolts;

    public SwerveModuleRelative(int index) {
        driveMotor = new CANSparkMax(DriveConstants.driveMotorPorts[index], CANSparkMax.MotorType.kBrushless);
        turnMotor = new CANSparkMax(DriveConstants.turningMotorPorts[index],
                CANSparkMax.MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        driveMotor.setInverted(DriveConstants.driveEncoderReversed[index]);
        // driveEncoder.setInverted(DriveConstants.driveEncoderReversed[index]);
        driveEncoder.setPositionConversionFactor((1 / ModuleConstants.driveMotorGearRatio) * Math.PI
                * ModuleConstants.wheelDiameterMeters);
        driveEncoder.setVelocityConversionFactor((1 / ModuleConstants.driveMotorGearRatio) * (Math.PI
                * ModuleConstants.wheelDiameterMeters) * (1.0 / 60.0));

        turnMotor.setInverted(DriveConstants.turningEncoderReversed[index]);
        // turnEncoder.setInverted(DriveConstants.turningEncoderReversed[index]);
        turnEncoder.setPositionConversionFactor((1 / ModuleConstants.turnMotorGearRatio)*(2*Math.PI));
        turnEncoder.setVelocityConversionFactor((1 / ModuleConstants.turnMotorGearRatio)*(2*Math.PI) * (1.0 / 60.0));

        driveMotor.setSmartCurrentLimit(DriveConstants.driveMotorStallLimit,
                DriveConstants.driveMotorFreeLimit);
        turnMotor.setSmartCurrentLimit(DriveConstants.turnMotorStallLimit,
                DriveConstants.turnMotorFreeLimit);

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
        
        double turningPositionRads = turnEncoder.getPosition();
        while (turningPositionRads > Math.PI*2){
            turningPositionRads-= 2 * Math.PI;
        }
        while (turningPositionRads <0){
            turningPositionRads+= 2 * Math.PI;
        }
        data.turnAbsolutePositionRad = turningPositionRads;
        data.turnVelocityRadPerSec = turnEncoder.getVelocity();
        data.turnAppliedVolts = turnAppliedVolts;
        data.turnCurrentAmps = Math.abs(turnMotor.getOutputCurrent());
        data.turnTempCelcius = turnMotor.getMotorTemperature();
    };

    @Override
    public void setDriveVoltage(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -DriveConstants.maxMotorVolts,
                DriveConstants.maxMotorVolts);
        driveMotor.setVoltage(driveAppliedVolts);
    };

    @Override
    public void setTurnVoltage(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -DriveConstants.maxMotorVolts,
                DriveConstants.maxMotorVolts);
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

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(0);
    };
}