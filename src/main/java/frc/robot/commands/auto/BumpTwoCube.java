package frc.robot.commands.auto;

import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathCommand;
import frc.robot.location.LocationHelper;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import java.util.List;

public class BumpTwoCube {

  public static Command get(
      DriveSubsystem driveSubsystem, ManipulatorSubsystem manipulatorSubsystem) {
    PathPlannerPath path = PathPlannerPath.fromPathFile("CubeBump");

    return new SequentialCommandGroup(
            manipulatorSubsystem.getShootCubeCommand(),
            new FollowPathWithEvents(
                new PathCommand(driveSubsystem, path, true, true), path, driveSubsystem::getPose),
            manipulatorSubsystem.getHomeAllCommand(),
            new ProxyCommand(() -> getDynamicPathToCubeNode(driveSubsystem)),
            manipulatorSubsystem.getShootCubeCommand(),
            new ProxyCommand(
                () -> getDynamicPathToSecondCube(driveSubsystem, manipulatorSubsystem)),
            manipulatorSubsystem.getCubeHuntCommand(driveSubsystem),
            driveSubsystem.getDriveStopCommand(),
            manipulatorSubsystem.getHomeAllCommand(),
            new ProxyCommand(() -> getDynamicPathToCubeNode(driveSubsystem)),
            manipulatorSubsystem.getShootCubeCommand(),
            new ProxyCommand(
                () -> getDynamicPathToSecondCube(driveSubsystem, manipulatorSubsystem)),
            driveSubsystem.getDriveStopCommand())
        .withName("CubeCubeLS");
  }

  public static Command getDynamicPathToCommunityZone1(DriveSubsystem driveSubsystem) {
    Pose2d startPose = driveSubsystem.getPose();
    Pose2d endPose = new Pose2d(5.07, .93, Rotation2d.fromDegrees(180.0));

    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPose, endPose);

    PathPlannerPath path =
        new PathPlannerPath(
            bezierPoints,
            new PathConstraints(2.0, 3.0, 2 * Math.PI, 4 * Math.PI),
            new GoalEndState(0.0, Rotation2d.fromDegrees(0)));

    return new PathCommand(driveSubsystem, path, false, false)
        .andThen(driveSubsystem.getDriveStopCommand());
  }

  public static Command getDynamicPathToCubeNode(DriveSubsystem driveSubsystem) {
    Pose2d startPose = driveSubsystem.getPose();

    Pose2d cubeNodePose =
        LocationHelper.getTransformedYAxisForAllianceColor(
            new Pose2d(2.1, .95, Rotation2d.fromDegrees(180.0)));
    Pose2d entryCubeNodePose =
        LocationHelper.getTransformedYAxisForAllianceColor(
            new Pose2d(3.7, .77, Rotation2d.fromDegrees(180.0)));

    List<Translation2d> bezierPoints =
        PathPlannerPath.bezierFromPoses(startPose, entryCubeNodePose, cubeNodePose);

    PathPlannerPath path =
        new PathPlannerPath(
            bezierPoints,
            new PathConstraints(3.0, 5.0, 2 * Math.PI, 4 * Math.PI),
            new GoalEndState(0.0, Rotation2d.fromDegrees(0)));

    return new PathCommand(driveSubsystem, path, false, false)
        .andThen(driveSubsystem.getDriveStopCommand());
  }

  public static Command getDynamicPathToSecondCube(
      DriveSubsystem driveSubsystem, ManipulatorSubsystem manipulatorSubsystem) {

    Pose2d startPose = driveSubsystem.getPose();

    Pose2d cubeNodePose = new Pose2d(6.0, 1.2, Rotation2d.fromDegrees(45.0));

    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPose, cubeNodePose);

    PathPlannerPath path =
        new PathPlannerPath(
            bezierPoints,
            new PathConstraints(2.5, 4.0, 2 * Math.PI, 4 * Math.PI),
            new GoalEndState(0.0, Rotation2d.fromDegrees(45)));

    return new PathCommand(driveSubsystem, path, false, false)
        .andThen(driveSubsystem.getDriveStopCommand());
  }
}
