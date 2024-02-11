package frc.robot.subsystems.swerve.real;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.GyroIO;

public class NavX2Gyro implements GyroIO {

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private double yaw = 0;

    public NavX2Gyro() {
        new Thread(() -> {
            try {
                while (gyro.isCalibrating()) {

                }
                gyro.reset();

            } catch (Exception e) {
            }
        }).start();
    }

    @Override
    public void updateData(GyroData data) {

        data.isCalibrating = gyro.isCalibrating();
        data.connected = gyro.isConnected();

        if (data.connected && !data.isCalibrating) {
            data.yawDeg = gyro.getYaw();
            data.pitchDeg = gyro.getPitch();
            data.rollDeg = gyro.getRoll();
        } else {
            // double angleDiffRad = Robot.swerve.getChassisSpeeds().omegaRadiansPerSecond * 0.02;
            // Rotation2d currentRotationDiff = new Rotation2d(angleDiffRad);

            // yaw = (yaw + currentRotationDiff.getDegrees() + 360) % 360;
            // data.yawDeg = yaw;
        }
    }

    @Override
    public void resetGyro() {
        gyro.reset();

    }

}
