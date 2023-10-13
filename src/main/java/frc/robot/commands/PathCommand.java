package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
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

  public PathCommand(DriveSubsystem driveSubsystem, PathPlannerTrajectory pathPlannerTrajectory) {
    this(driveSubsystem, pathPlannerTrajectory, true, true);
  }

  /** Creates a new PathCommand. */
  public PathCommand(
      DriveSubsystem driveSubsystem,
      PathPlannerTrajectory pathPlannerTrajectory,
      boolean withRequirements,
      boolean isFirstPath) {

    if (DEBUG_MODE) {
      DataLogManager.log(
          "%%%%%%%%%% Trajectory total time = " + pathPlannerTrajectory.getTotalTimeSeconds());
      DataLogManager.log(
          "%%%%%%%%%% Trajectory states size = " + pathPlannerTrajectory.getStates().size());
    }

    AutoConstants.THETA_SPEED_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);

    addCommands(
        new InstantCommand(
            () -> {
              if (isFirstPath) {
                driveSubsystem.setSpeedMode(SpeedMode.TURBO);

                PathPlannerTrajectory transformedTrajectory =
                    PathPlannerTrajectory.transformTrajectoryForAlliance(
                        pathPlannerTrajectory, DriverStation.getAlliance());

                Pose2d pose = transformedTrajectory.getInitialHolonomicPose();
                driveSubsystem.resetOdometry(pose);
              }
            }),
        new PPSwerveControllerCommand(
                pathPlannerTrajectory,
                driveSubsystem::getPose,
                RobotConstants.DRIVE_KINEMATICS,
                AutoConstants.X_SPEED_CONTROLLER,
                AutoConstants.Y_SPEED_CONTROLLER,
                AutoConstants.THETA_SPEED_CONTROLLER,
                driveSubsystem::setModuleStates,
                true)
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
