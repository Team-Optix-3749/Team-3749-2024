package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class Constants {

  public static final boolean ROBOT_IS_REAL = Robot.isReal();

  public static final class Elevator {
    public static final int elevatorMotorOneID = 1;
    public static final int elevatorMotorTwoID = 2;
  }

  public static final class Sim {
    public static final double loopPeriodSec = 0.02;

    public static final class PIDValues {
      // will eventally be easier to change values from here than poke around through
      // files
      public static double kP_teleopTurn = 1.3;
      public static double kD_teleopTurn = 0.0;

      public static double kP_MiscDrive = 0.42;
      public static double kD_MiscDrive = 0.02;
      public static double kP_MiscTurn = 0.15;
      public static double kD_MiscTurn = 0.003;

      public static double kP_TurnToAngle = 0.15;
      public static double kD_TurnToAngle = 0.008;

      public static double kP_PathPlannerDrive = 9;
      public static double kD_PathPlannerDrive = 0.01;

      public static double kP_PathPlannerTurn = 4.2;
      public static double kD_PathPlannerTurn = 0.0;
    }
  }

}