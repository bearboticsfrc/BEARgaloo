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

public class CubeCubeLS {

  public static Command get(
      DriveSubsystem driveSubsystem, ManipulatorSubsystem manipulatorSubsystem) {
    PathPlannerTrajectory pathPlannerTrajectory =
        PathPlanner.loadPath(
            "CubeCubeLS",
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
            manipulatorSubsystem.getHomeAllCommand(),
            new ProxyCommand(() -> getDynamicPathToCubeNode(driveSubsystem)),
            manipulatorSubsystem.getShootCubeCommand(),
            new ProxyCommand(() -> getDynamicPathToLeaveCZ(driveSubsystem)),
            driveSubsystem.getDriveStopCommand())
        .withName("CubeCubeLS");
  }

  public static Command getDynamicPathToCommunityZone1(DriveSubsystem driveSubsystem) {
    Pose2d endPose =
        LocationHelper.getTransformedYAxisForAllianceColor(
            new Pose2d(5.07, .93, Rotation2d.fromDegrees(180)));

    PathPlannerTrajectory trajectory =
        PathPlanner.generatePath(
            new PathConstraints(2.0, 4),
            new PathPoint(
                    driveSubsystem.getPose().getTranslation(),
                    Rotation2d.fromDegrees(0),
                    driveSubsystem.getPose().getRotation())
                .withNextControlLength(0.1),
            new PathPoint(
                    endPose.getTranslation(), endPose.getRotation(), Rotation2d.fromDegrees(0))
                .withPrevControlLength(1));

    return new PathCommand(driveSubsystem, trajectory, false, false)
        .andThen(driveSubsystem.getDriveStopCommand());
  }

  public static Command getDynamicPathToCubeNode(DriveSubsystem driveSubsystem) {
    Pose2d cubeNodePose =
        LocationHelper.getTransformedYAxisForAllianceColor(
            new Pose2d(2.1, 4.52, Rotation2d.fromDegrees(180)));
    Pose2d entryPose =
        LocationHelper.getTransformedYAxisForAllianceColor(
            new Pose2d(4.5, 4.85, Rotation2d.fromDegrees(180)));

    PathPlannerTrajectory trajectory =
        PathPlanner.generatePath(
            new PathConstraints(3.0, 5),
            new PathPoint(
                    driveSubsystem.getPose().getTranslation(),
                    LocationHelper.getTransformedHeadingForAllianceColor(
                        Rotation2d.fromDegrees(155)),
                    driveSubsystem.getPose().getRotation())
                .withNextControlLength(1),
            new PathPoint(
                    entryPose.getTranslation(), entryPose.getRotation(), Rotation2d.fromDegrees(0))
                .withPrevControlLength(1),
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
        LocationHelper.getTransformedYAxisForAllianceColor(
            new Pose2d(5.8, 4.70, Rotation2d.fromDegrees(-45)));

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

  public static Command getDynamicPathToLeaveCZ(DriveSubsystem driveSubsystem) {
    Pose2d cubeNodePose =
        LocationHelper.getTransformedYAxisForAllianceColor(
            new Pose2d(6.1, 4.8, Rotation2d.fromDegrees(180)));

    PathPlannerTrajectory trajectory =
        PathPlanner.generatePath(
            new PathConstraints(3.0, 5),
            new PathPoint(
                    driveSubsystem.getPose().getTranslation(),
                    LocationHelper.getTransformedHeadingForAllianceColor(
                        Rotation2d.fromDegrees(155)),
                    driveSubsystem.getPose().getRotation())
                .withNextControlLength(1),
            new PathPoint(
                    cubeNodePose.getTranslation(),
                    cubeNodePose.getRotation(),
                    Rotation2d.fromDegrees(0))
                .withPrevControlLength(1));

    return new PathCommand(driveSubsystem, trajectory, false, false)
        .andThen(driveSubsystem.getDriveStopCommand());
  }
}
