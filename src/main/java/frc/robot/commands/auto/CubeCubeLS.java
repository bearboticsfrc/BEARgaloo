package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.LogCommand;
import frc.robot.commands.PathCommand;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.manipulator.RollerConstants.RollerSpeed;
import frc.robot.constants.manipulator.WristConstants.WristPositions;
import frc.robot.location.LocationHelper;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class CubeCubeLS {

  public static Command get(
      DriveSubsystem driveSubsystem, ManipulatorSubsystem manipulatorSubsystem) {
    PathPlannerTrajectory pathPlannerTrajectory =
        PathPlanner.loadPath(
            "CubeCubeLS",
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    PathPlannerTrajectory pathPlannerTrajectory2 =
        PathPlanner.loadPath(
            "CubeCubeLS",
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    return new SequentialCommandGroup(
            manipulatorSubsystem.getShootCubeCommand(),
            new ProxyCommand(
                () ->
                    new FollowPathWithEvents(
                        new PathCommand(driveSubsystem, pathPlannerTrajectory, true, true),
                        pathPlannerTrajectory.getMarkers(),
                        manipulatorSubsystem.getEventMap())),
            manipulatorSubsystem.getRollerRunCommand(RollerSpeed.OFF),
            manipulatorSubsystem.getWristRunCommand(WristPositions.HOME),
            new ProxyCommand(() -> getDynamicPathToCubeNode(driveSubsystem)),
            manipulatorSubsystem.getShootCubeCommand(),
            new ProxyCommand(() -> getDynamicPathToSecondCube(driveSubsystem)),
            new InstantCommand(() -> driveSubsystem.stop()))
        .withName("CubeCubeLS");
  }

  public static Command getDynamicPathToCommunityZone1(DriveSubsystem driveSubsystem) {
    Pose2d startPose = driveSubsystem.getPose();
    Pose2d endPose =
        LocationHelper.transformYAxisForAllianceColor(
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

    Command command =
        new PathCommand(driveSubsystem, trajectory, false, false)
            .beforeStarting(new LogCommand("Starting Path Command"))
            .andThen(new InstantCommand(driveSubsystem::stop))
            .andThen(new LogCommand("Finished PathCommand"));
    System.out.println("Making dynamic path command");
    return command;
  }

  public static Command getDynamicPathToCubeNode(DriveSubsystem driveSubsystem) {
    // Pose2d cubeNodePose = new Pose2d(1.9, 1.05, new Rotation2d());
    // Pose2d cubeNodePose = new Pose2d(1.9, 4.42, Rotation2d.fromDegrees(180.0));
    Pose2d cubeNodePose =
        LocationHelper.transformYAxisForAllianceColor(
            new Pose2d(2.3, 4.52, Rotation2d.fromDegrees(180.0)));
    Pose2d entryPose =
        LocationHelper.transformYAxisForAllianceColor(
            new Pose2d(4.0, 4.85, Rotation2d.fromDegrees(180.0)));

    PathPlannerTrajectory trajectory =
        PathPlanner.generatePath(
            new PathConstraints(2.5, 4),
            new PathPoint(
                    driveSubsystem.getPose().getTranslation(),
                    LocationHelper.transformHeadingForAllianceColor(Rotation2d.fromDegrees(155.0)),
                    driveSubsystem.getPose().getRotation())
                .withNextControlLength(1.0),
            new PathPoint(
                    entryPose.getTranslation(),
                    entryPose.getRotation(),
                    Rotation2d.fromDegrees(0.0))
                .withPrevControlLength(1.0),
            new PathPoint(
                    cubeNodePose.getTranslation(),
                    cubeNodePose.getRotation(),
                    Rotation2d.fromDegrees(0.0))
                .withPrevControlLength(1.0));

    Command command =
        new PathCommand(driveSubsystem, trajectory, false, false)
            .beforeStarting(new LogCommand("Starting Path Command"))
            .andThen(new InstantCommand(driveSubsystem::stop))
            .andThen(new LogCommand("Finished PathCommand"));
    System.out.println("Making dynamic path command");
    return command;
  }

  public static Command getDynamicPathToSecondCube(DriveSubsystem driveSubsystem) {
    // Pose2d cubeNodePose = new Pose2d(1.9, 1.05, new Rotation2d());
    // Pose2d cubeNodePose = new Pose2d(1.9, 4.42, Rotation2d.fromDegrees(180.0));
    Pose2d cubeNodePose =
        LocationHelper.transformYAxisForAllianceColor(
            new Pose2d(6.0, 4.50, Rotation2d.fromDegrees(-45.0)));
    //            new Pose2d(6.27, 4.21, Rotation2d.fromDegrees(-45.0)));
    // Pose2d entryPose =
    //     LocationHelper.transformYAxisForAllianceColor(
    //         new Pose2d(4.0, 4.85, Rotation2d.fromDegrees(180.0)));

    PathPlannerTrajectory trajectory =
        PathPlanner.generatePath(
            new PathConstraints(2.5, 4),
            new PathPoint(
                    driveSubsystem.getPose().getTranslation(),
                    LocationHelper.transformHeadingForAllianceColor(Rotation2d.fromDegrees(20.0)),
                    driveSubsystem.getPose().getRotation())
                .withNextControlLength(.75),
            new PathPoint(
                    cubeNodePose.getTranslation(),
                    cubeNodePose.getRotation(),
                    cubeNodePose.getRotation())
                .withPrevControlLength(1.0));

    Command command =
        new PathCommand(driveSubsystem, trajectory, false, false)
            .beforeStarting(new LogCommand("Starting Path Command"))
            .andThen(new InstantCommand(driveSubsystem::stop))
            .andThen(new LogCommand("Finished PathCommand"));
    System.out.println("Making dynamic path command");
    return command;
  }
}
