package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathCommand;
import frc.robot.constants.AutoConstants;
import frc.robot.location.LocationHelper;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import java.util.Arrays;
import java.util.List;

public class BumpTwoCube {

  public static Command get(
      DriveSubsystem driveSubsystem, ManipulatorSubsystem manipulatorSubsystem) {
    PathPlannerTrajectory pathPlannerTrajectory =
        PathPlanner.loadPath(
            "CubeBump",
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(
            manipulatorSubsystem.getShootCubeCommand(),
            new FollowPathWithEvents(
                new PathCommand(driveSubsystem, pathPlannerTrajectory, true, true),
                pathPlannerTrajectory.getMarkers(),
                manipulatorSubsystem.getEventMap()),
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
    Pose2d endPose =
        LocationHelper.getTransformedYAxisForAllianceColor(
            new Pose2d(5.07, .93, Rotation2d.fromDegrees(180.0)));

    PathPlannerTrajectory trajectory =
        PathPlanner.generatePath(
            new PathConstraints(2.0, 4),
            new PathPoint(
                    startPose.getTranslation(),
                    Rotation2d.fromDegrees(0.0),
                    startPose.getRotation())
                .withNextControlLength(0.1),
            new PathPoint(
                    endPose.getTranslation(), endPose.getRotation(), Rotation2d.fromDegrees(0.0))
                .withPrevControlLength(1.0));

    return new PathCommand(driveSubsystem, trajectory, false, false)
        .andThen(driveSubsystem.getDriveStopCommand());
  }

  public static Command getDynamicPathToCubeNode(DriveSubsystem driveSubsystem) {
    Pose2d cubeNodePose =
        LocationHelper.transformYAxisForAllianceColor(
            new Pose2d(2.1, .95, Rotation2d.fromDegrees(180.0)));
    Pose2d entryCubeNodePose =
        LocationHelper.transformYAxisForAllianceColor(
            new Pose2d(3.7, .77, Rotation2d.fromDegrees(180.0)));
    PathPlannerTrajectory trajectory =
        PathPlanner.generatePath(
            new PathConstraints(3.0, 5),
            new PathPoint(
                    driveSubsystem.getPose().getTranslation(),
                    LocationHelper.getTransformedHeadingForAllianceColor(Rotation2d.fromDegrees(0)),
                    driveSubsystem.getPose().getRotation())
                .withNextControlLength(1),
            new PathPoint(
                    entryCubeNodePose.getTranslation(),
                    entryCubeNodePose.getRotation(),
                    Rotation2d.fromDegrees(0.0))
                .withPrevControlLength(1.0),
            new PathPoint(
                    cubeNodePose.getTranslation(),
                    cubeNodePose.getRotation(),
                    Rotation2d.fromDegrees(0))
                .withPrevControlLength(1));

    return new PathCommand(driveSubsystem, trajectory, false, false)
        .andThen(driveSubsystem.getDriveStopCommand());
  }

  public static Command getDynamicPathToSecondCube(
      DriveSubsystem driveSubsystem, ManipulatorSubsystem manipulatorSubsystem) {
    Pose2d cubeNodePose =
        LocationHelper.transformYAxisForAllianceColor(
            new Pose2d(6.0, 1.2, Rotation2d.fromDegrees(45.0)));

    List<PathPoint> points =
        Arrays.asList(
            new PathPoint(
                    driveSubsystem.getPose().getTranslation(),
                    LocationHelper.getTransformedHeadingForAllianceColor(Rotation2d.fromDegrees(0)),
                    driveSubsystem.getPose().getRotation())
                .withNextControlLength(.75),
            new PathPoint(
                    cubeNodePose.getTranslation(),
                    cubeNodePose.getRotation(),
                    cubeNodePose.getRotation())
                .withPrevControlLength(1));

    List<EventMarker> eventMarkers =
        Arrays.asList(
            new EventMarker(Arrays.asList("startRollers"), .6),
            new EventMarker(Arrays.asList("lowerWrist"), .8));

    PathPlannerTrajectory trajectory =
        PathPlanner.generatePath(new PathConstraints(2.5, 4), points, eventMarkers);

    return new PathCommand(driveSubsystem, trajectory, false, false)
        .andThen(driveSubsystem.getDriveStopCommand());
  }
}
