// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;
import java.util.function.DoubleConsumer;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANDrivetrain extends SubsystemBase {

  DifferentialDrive m_drivetrain;

  public CANDrivetrain() {

    TalonSRX leftFront = new TalonSRX(kLeftFrontID);
    TalonSRX leftRear = new TalonSRX(kLeftRearID);
    TalonSRX rightFront = new TalonSRX(kRightFrontID);
    TalonSRX rightRear = new TalonSRX(kRightRearID);

    leftRear.follow(leftFront);
    rightRear.follow(rightFront);

    leftFront.configPeakCurrentLimit(kCurrentLimit);
    rightFront.configPeakCurrentLimit(kCurrentLimit);

    leftFront.setInverted(true);
    rightFront.setInverted(false);

    DoubleConsumer setLeftSpeed = (double speed) -> {
      speed = MathUtil.clamp(speed, -12, 12);
      leftFront.set(TalonSRXControlMode.Current, kCurrentLimit);
    };

    DoubleConsumer setRightSpeed = (double speed) -> {
      speed = MathUtil.clamp(speed, -12, 12);
      rightFront.set(TalonSRXControlMode.Current, kCurrentLimit);
    };
 
    m_drivetrain = new DifferentialDrive(setLeftSpeed, setRightSpeed);
  }

  public void arcadeDrive(double speed, double rotation) {
    m_drivetrain.arcadeDrive(speed, rotation);
  }

  @Override
  public void periodic() {

  }
}
