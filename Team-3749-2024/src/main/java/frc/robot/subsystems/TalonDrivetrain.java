// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;
import java.util.function.DoubleConsumer;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TalonDrivetrain extends SubsystemBase {

  DifferentialDrive m_drivetrain;

  public TalonDrivetrain() {

    /* define motors */
    TalonSRX leftFront = new TalonSRX(kLeftFrontID);
    TalonSRX leftRear = new TalonSRX(kLeftRearID);
    TalonSRX rightFront = new TalonSRX(kRightFrontID);
    TalonSRX rightRear = new TalonSRX(kRightRearID);

    /* rear motors follow the control mode and output value of front motors */
    leftRear.follow(leftFront);
    rightRear.follow(rightFront);

    /* set current limits */
    leftFront.configPeakCurrentLimit(kCurrentLimit);
    rightFront.configPeakCurrentLimit(kCurrentLimit);

    /* left motors are inverted, right motors are not */
    leftFront.setInverted(true);
    rightFront.setInverted(false);

    /* lambdas to set motor speeds which are plugged into differential drive */
    DoubleConsumer setLeftSpeed = (double speed) -> {
      leftFront.set(TalonSRXControlMode.PercentOutput, speed);
    };

    DoubleConsumer setRightSpeed = (double speed) -> {
      rightFront.set(TalonSRXControlMode.PercentOutput, speed);
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
