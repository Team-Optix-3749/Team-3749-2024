// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* This class declares the subsystem for the robot drivetrain if controllers are connected via CAN. Make sure to go to
 * RobotContainer and uncomment the line declaring this subsystem and comment the line for PWMDrivetrain.
 *
 * The subsystem contains the objects for the hardware contained in the mechanism and handles low level logic
 * for control. Subsystems are a mechanism that, when used in conjuction with command "Requirements", ensure
 * that hardware is only being used by 1 command at a time.
 */
public class CANDrivetrain extends SubsystemBase {
  /*Class member variables. These variables represent things the class needs to keep track of and use between
  different method calls. */
  DifferentialDrive m_drivetrain;
  /*Constructor. This method is called when an instance of the class is created. This should generally be used to set up
   * member variables and perform any configuration or set up necessary on hardware.
   */
  public CANDrivetrain() {
    // CANSparkMax leftFront = new CANSparkMax(kLeftFrontID, MotorType.kBrushed);
    // CANSparkMax leftRear = new CANSparkMax(kLeftRearID, MotorType.kBrushed);
    // CANSparkMax rightFront = new CANSparkMax(kRightFrontID, MotorType.kBrushed);
    // CANSparkMax rightRear = new CANSparkMax(kRightRearID, MotorType.kBrushed);

    TalonSRX leftFront = new TalonSRX(kLeftFrontID);
    TalonSRX leftRear = new TalonSRX(kLeftRearID);
    TalonSRX rightFront = new TalonSRX(kRightFrontID);
    TalonSRX rightRear = new TalonSRX(kRightRearID);


    /*Sets current limits for the drivetrain motors. This helps reduce the likelihood of wheel spin, reduces motor heating
     *at stall (Drivetrain pushing against something) and helps maintain battery voltage under heavy demand */

    // Set the rear motors to follow the front motors.
    leftRear.follow(leftFront);
    rightRear.follow(rightFront);

    /*Sets current limits for the drivetrain motors. This helps reduce the likelihood of wheel spin, reduces motor heating
     *at stall (Drivetrain pushing against something) and helps maintain battery voltage under heavy demand */

    // leftFront.setSmartCurrentLimit(kCurrentLimit);
    // leftRear.setSmartCurrentLimit(kCurrentLimit);
    // rightFront.setSmartCurrentLimit(kCurrentLimit);
    // rightRear.setSmartCurrentLimit(kCurrentLimit);


    leftFront.configPeakCurrentLimit(kCurrentLimit);
    rightFront.configPeakCurrentLimit(kCurrentLimit);



    // Invert the left side so both side drive forward with positive motor outputs
    leftFront.setInverted(true);
    rightFront.setInverted(false);

    // Put the front motors into the differential drive object. This will control all 4 motors with
    // the rears set to follow the fronts

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

  /*Method to control the drivetrain using arcade drive. Arcade drive takes a speed in the X (forward/back) direction
   * and a rotation about the Z (turning the robot about it's center) and uses these to control the drivetrain motors */
  public void arcadeDrive(double speed, double rotation) {
    m_drivetrain.arcadeDrive(speed, rotation);
  }

  @Override
  public void periodic() {
    /*This method will be called once per scheduler run. It can be used for running tasks we know we want to update each
     * loop such as processing sensor data. Our drivetrain is simple so we don't have anything to put here */
  }
}
