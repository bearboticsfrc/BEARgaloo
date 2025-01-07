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

public class CubeCubeLS {

  public static Command get(
      DriveSubsystem driveSubsystem, ManipulatorSubsystem manipulatorSubsystem) {

    PathPlannerPath path = PathPlannerPath.fromPathFile("CubeCubeLS");

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
            manipulatorSubsystem.getHomeAllCommand(),
            new ProxyCommand(() -> getDynamicPathToCubeNode(driveSubsystem)),
            manipulatorSubsystem.getShootCubeCommand(),
            new ProxyCommand(() -> getDynamicPathToLeaveCZ(driveSubsystem)),
            driveSubsystem.getDriveStopCommand())
        .withName("CubeCubeLS");
  }

  public static Command getDynamicPathToCommunityZone1(DriveSubsystem driveSubsystem) {
    Pose2d startPose = driveSubsystem.getPose();

    Pose2d endPose =
        LocationHelper.getTransformedYAxisForAllianceColor(
            new Pose2d(5.07, .93, Rotation2d.fromDegrees(180.0)));

    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPose, endPose);

    PathPlannerPath path =
        new PathPlannerPath(
            bezierPoints,
            new PathConstraints(2.0, 4.0, 2 * Math.PI, 4 * Math.PI),
            new GoalEndState(0.0, Rotation2d.fromDegrees(0)));

    return new PathCommand(driveSubsystem, path, false, false)
        .andThen(driveSubsystem.getDriveStopCommand());
  }

  public static Command getDynamicPathToCubeNode(DriveSubsystem driveSubsystem) {
    Pose2d startPose = driveSubsystem.getPose();

    Pose2d cubeNodePose =
        LocationHelper.getTransformedYAxisForAllianceColor(
            new Pose2d(2.1, 4.52, Rotation2d.fromDegrees(180.0)));
    Pose2d entryPose =
        LocationHelper.getTransformedYAxisForAllianceColor(
            new Pose2d(4.5, 4.85, Rotation2d.fromDegrees(180.0)));

    List<Translation2d> bezierPoints =
        PathPlannerPath.bezierFromPoses(startPose, entryPose, cubeNodePose);

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

    Pose2d cubeNodePose =
        LocationHelper.getTransformedYAxisForAllianceColor(
            new Pose2d(5.8, 4.70, Rotation2d.fromDegrees(-45.0)));

    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPose, cubeNodePose);

    //    List<EventMarker> eventMarkers =
    //        Arrays.asList(
    //            new EventMarker(Arrays.asList("startRollers"), .6),
    //            new EventMarker(Arrays.asList("lowerWrist"), .8));

    PathPlannerPath path =
        new PathPlannerPath(
            bezierPoints,
            new PathConstraints(2.5, 4.0, 2 * Math.PI, 4 * Math.PI),
            new GoalEndState(0.0, Rotation2d.fromDegrees(0)));

    return new PathCommand(driveSubsystem, path, false, false)
        .andThen(driveSubsystem.getDriveStopCommand());
  }

  public static Command getDynamicPathToLeaveCZ(DriveSubsystem driveSubsystem) {

    Pose2d startPose = driveSubsystem.getPose();

    Pose2d cubeNodePose =
        LocationHelper.getTransformedYAxisForAllianceColor(
            new Pose2d(6.1, 4.8, Rotation2d.fromDegrees(180.0)));

    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPose, cubeNodePose);

    PathPlannerPath path =
        new PathPlannerPath(
            bezierPoints,
            new PathConstraints(3.0, 5.0, 2 * Math.PI, 4 * Math.PI),
            new GoalEndState(0.0, Rotation2d.fromDegrees(0)));

    return new PathCommand(driveSubsystem, path, false, false)
        .andThen(driveSubsystem.getDriveStopCommand());
  }
}
