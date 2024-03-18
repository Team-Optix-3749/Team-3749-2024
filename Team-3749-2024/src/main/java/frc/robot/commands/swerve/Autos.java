package frc.robot.commands.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.utils.MiscConstants;
import frc.robot.utils.SuperStructureStates;

public class Autos {

    public static Command getStraightLine() {
        return AutoUtils.getAutoPath(
                "straight line",
                new Pose2d(4, 5, new Rotation2d()));
    }

    public static Command getTaxi() {
        return new SequentialCommandGroup(
                NamedCommands.getCommand("shoot_subwoofer"),
                AutoUtils.getChoreoAutoPath(
                        "taxi",
                        new Pose2d(
                                0.7623372673988342,
                                4.4742279052734375,
                                Rotation2d.fromRadians(-1.0465789848978129))));
    }

    public static Command get4Piece() {
        Pose2d startingPos = new Pose2d(
                new Translation2d(1.318, 5.436),
                Rotation2d.fromRadians(0));

        return new SequentialCommandGroup(
                AutoUtils.getSubwooferShot(0),

                new ParallelCommandGroup(
                        AutoUtils.getChoreoAutoPath("middle-speaker-3xwing_speaker", startingPos),
                        AutoUtils.getintake(0.5),
                        AutoUtils.getSubwooferShot(2),
                        AutoUtils.getintake(3.3),
                        AutoUtils.getSubwooferShot(5.3),
                        AutoUtils.getintake(7.3),
                        AutoUtils.getSubwooferShot(9.7),
                        AutoUtils.getStow(12)
                )
                 );
    }

    public static Command get4PieceNoRotation() {
        Pose2d startingPos = new Pose2d(
                new Translation2d(1.318, 5.436),
                Rotation2d.fromRadians(0));

        return new SequentialCommandGroup(
                AutoUtils.getCycle(0),
                new ParallelCommandGroup(
                        AutoUtils.getChoreoAutoPath("no rotation", startingPos),
                        AutoUtils.getCycle(3.5),
                        AutoUtils.getCycle(10),
                        AutoUtils.getCycle(15)));
    }

    public static Command get4PieceNoCommands() {
        return AutoUtils.getChoreoAutoPath(
                "no rotation",
                new Pose2d(new Translation2d(1.318, 5.436), Rotation2d.fromRadians(0)));
    }

    public static Command getTroll() {
        return new SequentialCommandGroup(
                AutoUtils.getCycle(0),
                Commands.run(() -> Robot.state = SuperStructureStates.STOW),
                AutoUtils.getChoreoAutoPath(
                        "top-speaker-troll",
                        new Pose2d(
                                0.7535725831985474,
                                4.482047080993652,
                                new Rotation2d(-1.0074806999559232))));
    }

    public static Command getBottomRun() {
        return new SequentialCommandGroup(
                AutoUtils.getCycle(0),
                new WaitCommand(6),
                AutoUtils.getChoreoAutoPath(
                        "bottom-speaker",
                        new Pose2d(
                                0.7692505121231079,
                                4.479482650756836,
                                new Rotation2d(-1.0356489717435746))));
    }

    public static Command getTestPath() {
        return AutoUtils.getChoreoAutoPath("testPath", new Pose2d());
    }
}
// public static Command getSide() {
// // return AutoUtils.getCycle(0);
// return new SequentialCommandGroup(AutoUtils.getCycle(0), new WaitCommand(4)
// AutoUtils.getAutoPath("4 piece middle solo", new Pose2d(1.32, 5.44, new
// Rotation2d())),
// AutoUtils.getCycle(8));
// }
