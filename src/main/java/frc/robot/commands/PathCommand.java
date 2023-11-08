package frc.robot.commands;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.DriveConstants.SpeedMode;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.DriveSubsystem;

/** A command to run a given trajectory with support for debug output about trajectory accuracy */
public class PathCommand extends SequentialCommandGroup {
  public static final boolean DEBUG_MODE = false;

  public PathCommand(DriveSubsystem driveSubsystem, PathPlannerPath path) {
    this(driveSubsystem, path, true, true);
  }

  /** Creates a new PathCommand. */
  public PathCommand(
      DriveSubsystem driveSubsystem,
      PathPlannerPath path,
      boolean withRequirements,
      boolean isFirstPath) {
    // Pose2d initialPose = new Pose2d(path.getPoint(0).position,
    // path.getPoint(0).holonomicRotation);

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);
    PathPlannerTrajectory pathPlannerTrajectory = new PathPlannerTrajectory(path, chassisSpeeds);
    if (DEBUG_MODE) {
      DataLogManager.log(
          "%%%%%%%%%% Trajectory total time = " + pathPlannerTrajectory.getTotalTimeSeconds());
      DataLogManager.log(
          "%%%%%%%%%% Trajectory states size = " + pathPlannerTrajectory.getStates().size());
    }

    HolonomicPathFollowerConfig holonomicPathFollowerConfig =
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            AutoConstants.MAX_SPEED_METERS_PER_SECOND, // Max module speed, in m/s
            RobotConstants
                .ROBOT_RADIUS, // Drive base radius in meters. Distance from robot center to
            // furthest
            // module.
            new ReplanningConfig() // Default path replanning config. See the API for the options
            // here
            );

    addCommands(
        new InstantCommand(
            () -> {
              if (isFirstPath) {
                driveSubsystem.setSpeedMode(SpeedMode.TURBO);

                //  PathPlannerTrajectory transformedTrajectory =
                //      PathPlannerTrajectory.transformTrajectoryForAlliance(
                //          pathPlannerTrajectory, DriverStation.getAlliance());
                //
                //                Pose2d pose = transformedTrajectory.getInitialHolonomicPose();
                //                driveSubsystem.resetOdometry(pose);
                Pose2d pose = pathPlannerTrajectory.getInitialTargetHolonomicPose();
                driveSubsystem.resetOdometry(pose);
              }
            }),
        new FollowPathHolonomic(
                path,
                driveSubsystem::getPose,
                driveSubsystem::getRobotRelativeSpeeds,
                driveSubsystem::driveRobotRelative,
                holonomicPathFollowerConfig,
                driveSubsystem)
            .alongWith(
                new ConditionalCommand(
                    new PathPlannerDebugCommand(pathPlannerTrajectory, driveSubsystem::getPose),
                    new InstantCommand(),
                    () -> DEBUG_MODE)));
    if (withRequirements) {
      addRequirements(driveSubsystem);
    }
  }
}
