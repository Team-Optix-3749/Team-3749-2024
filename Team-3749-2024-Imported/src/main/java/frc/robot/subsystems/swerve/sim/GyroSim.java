package frc.robot.subsystems.swerve.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.GyroIO;

public class GyroSim implements GyroIO {
  public GyroData gyroData = new GyroData();

  public void updateData(GyroData data) {
    gyroData = data;
  }

  public void updateGyroYaw() {
    double angleDiffRad = Robot.swerve.getChassisSpeeds().omegaRadiansPerSecond * 0.02;
    Rotation2d currentRotationDiff = new Rotation2d(angleDiffRad);

    gyroData.yawDeg = (gyroData.yawDeg + currentRotationDiff.getDegrees() + 360) % 360;
  }

  @Override
  public void resetGyro() {
    GyroData newData = new GyroData();

    // set all gyroData values to newData values
    gyroData.pitch = newData.pitch;
    gyroData.roll = newData.roll;
    gyroData.yawDeg = newData.yawDeg;
  }
}
