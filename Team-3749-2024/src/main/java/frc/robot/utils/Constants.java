package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class Constants {

    public static enum RobotType {
        REAL,
        SIM
      }
    
    public static final RobotType ROBOT_TYPE = Robot.isReal() ? RobotType.REAL : RobotType.SIM;


    public static final class Sim{
        public static final double loopPeriodSec = 0.02;
    }

    public static final class Elevator {
        public static final int elevatorMotorOneID = 1; // TODO: change to correct ID
        public static final int elevatorMotorTwoID = 2; // TODO: change to correct ID
    }

    public static final class ControllerConstants {
        public static final double deadband = 0.1;
    }


    public static final class AutoConstants {
        public static final Map<String, Command> eventMap = new HashMap<>();

        public static final SendableChooser<Command> autoChooser = new SendableChooser<>();

    }
}