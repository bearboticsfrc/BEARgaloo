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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CubeHuntCommand;
import frc.robot.commands.LogCommand;
import frc.robot.commands.PathCommand;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.manipulator.RollerConstants.RollerSpeed;
import frc.robot.constants.manipulator.WristConstants.WristPositions;
import frc.robot.location.LocationHelper;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import java.util.ArrayList;
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
            manipulatorSubsystem.getRollerRunCommand(RollerSpeed.OFF),
            manipulatorSubsystem.getWristRunCommand(WristPositions.HOME),
            new ProxyCommand(() -> getDynamicPathToCubeNode(driveSubsystem)),
            manipulatorSubsystem.getShootCubeCommand(),
            new ProxyCommand(
                () -> getDynamicPathToSecondCube(driveSubsystem, manipulatorSubsystem)),
            manipulatorSubsystem.getWristRunCommand(WristPositions.BOTTOM),
            manipulatorSubsystem.getRollerRunCommand(RollerSpeed.INTAKE),
            new CubeHuntCommand(driveSubsystem, manipulatorSubsystem::hasCube),
            new LogCommand("Done with CubeHunt"),
            new InstantCommand(() -> driveSubsystem.stop()),
            manipulatorSubsystem.getRollerRunCommand(RollerSpeed.OFF),
            manipulatorSubsystem.getWristRunCommand(WristPositions.HOME),
            new ProxyCommand(() -> getDynamicPathToCubeNode(driveSubsystem)),
            manipulatorSubsystem.getShootCubeCommand(),
            new ProxyCommand(
                () -> getDynamicPathToSecondCube(driveSubsystem, manipulatorSubsystem)),
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
            new Pose2d(2.1, .95, Rotation2d.fromDegrees(180.0)));

    PathPlannerTrajectory trajectory =
        PathPlanner.generatePath(
            new PathConstraints(3.0, 5),
            new PathPoint(
                    driveSubsystem.getPose().getTranslation(),
                    LocationHelper.transformHeadingForAllianceColor(Rotation2d.fromDegrees(0.0)),
                    driveSubsystem.getPose().getRotation())
                .withNextControlLength(1.0),
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

  public static Command getDynamicPathToSecondCube(
      DriveSubsystem driveSubsystem, ManipulatorSubsystem manipulatorSubsystem) {
    // Pose2d cubeNodePose = new Pose2d(1.9, 1.05, new Rotation2d());
    // Pose2d cubeNodePose = new Pose2d(1.9, 4.42, Rotation2d.fromDegrees(180.0));
    Pose2d cubeNodePose =
        LocationHelper.transformYAxisForAllianceColor(
            new Pose2d(6.15, 1.5, Rotation2d.fromDegrees(35.0)));

    List<PathPoint> points = new ArrayList<PathPoint>();
    points.add(
        new PathPoint(
                driveSubsystem.getPose().getTranslation(),
                LocationHelper.transformHeadingForAllianceColor(Rotation2d.fromDegrees(0.0)),
                driveSubsystem.getPose().getRotation())
            .withNextControlLength(.75));
    points.add(
        new PathPoint(
                cubeNodePose.getTranslation(),
                cubeNodePose.getRotation(),
                cubeNodePose.getRotation())
            .withPrevControlLength(1.0));

    List<EventMarker> eventMarkers = new ArrayList<EventMarker>();
    String[] startRollersStrings = {"startRollers"};
    String[] lowerWristStrings = {"lowerWrist"};
    List<String> startRollers = Arrays.asList(startRollersStrings);
    List<String> lowerWrist = Arrays.asList(lowerWristStrings);
    eventMarkers.add(new EventMarker(lowerWrist, .60));
    eventMarkers.add(new EventMarker(startRollers, .80));

    PathPlannerTrajectory trajectory =
        PathPlanner.generatePath(new PathConstraints(2.5, 4), points, eventMarkers);

    //   Command command =
    //      new FollowPathWithEvents(
    //              new PathCommand(driveSubsystem, trajectory, false, false),
    //              eventMarkers,
    //             manipulatorSubsystem.getEventMap())
    //         .beforeStarting(new LogCommand("Starting Path Command"))
    ///         .andThen(new InstantCommand(driveSubsystem::stop))
    //         .andThen(new LogCommand("Finished PathCommand"));

    Command command =
        new PathCommand(driveSubsystem, trajectory, false, false)
            .beforeStarting(new LogCommand("Starting Path Command"))
            .andThen(new InstantCommand(driveSubsystem::stop))
            .andThen(new LogCommand("Finished PathCommand"));
    System.out.println("Making dynamic path command");
    return command;
  }
}
