package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.subsystems.vision.StringFormatting;

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
      double trajectoryLength = pathPlannerTrajectory.getTotalTimeSeconds();
      System.out.println("%%%%%%%%%% Trajectory total time = " + trajectoryLength);
      System.out.println(
          "%%%%%%%%%% Trajectory states size = " + pathPlannerTrajectory.getStates().size());
      /* List<State> states = pathPlannerTrajectory.getStates();
      for (State state : states) {
        PathPlannerState pState = (PathPlannerState) state;
        System.out.println(
            "%%%%%%% holonomicRotation: "
                + pState.holonomicRotation
                + " state: "
                + pState.toString());
      } */
    }

    PIDController xController = new PIDController(AutoConstants.PX_CONTROLLER, 0, 0);
    PIDController yController = new PIDController(AutoConstants.PY_CONTROLLER, 0, 0);
    PIDController thetaController = new PIDController(AutoConstants.PTHETA_CONTROLLER, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addCommands(
        new LogCommand(
            "Starting path from "
                + StringFormatting.poseToString(pathPlannerTrajectory.getInitialHolonomicPose())
                + " to "
                + StringFormatting.poseToString(pathPlannerTrajectory.getEndState().poseMeters)),
        new InstantCommand(
            () -> {
              DataLogManager.log(
                  "Starting trajectory from "
                      + StringFormatting.poseToString(
                          pathPlannerTrajectory.getInitialHolonomicPose())
                      + " to "
                      + StringFormatting.poseToString(
                          pathPlannerTrajectory.getEndState().poseMeters));
              if (isFirstPath) {
                driveSubsystem.setSpeedMode(SpeedMode.TURBO);
                PathPlannerTrajectory transformedTrajectory =
                    PathPlannerTrajectory.transformTrajectoryForAlliance(
                        pathPlannerTrajectory, DriverStation.getAlliance());
                Pose2d pose = transformedTrajectory.getInitialHolonomicPose();
                // AllianceFlipUtil.apply(transformedTrajectory.getInitialHolonomicPose());
                DataLogManager.log(
                    "Setting initial pose to " + StringFormatting.poseToString(pose));
                driveSubsystem.resetOdometry(pose);
                DataLogManager.log(
                    "Initial pose to " + StringFormatting.poseToString(driveSubsystem.getPose()));
              }
            }),
        new PPSwerveControllerCommand(
                pathPlannerTrajectory,
                driveSubsystem::getPose,
                RobotConstants.DRIVE_KINEMATICS,
                xController,
                yController,
                thetaController,
                driveSubsystem::setModuleStates,
                true)
            .alongWith(
                new ConditionalCommand(
                    new PathPlannerDebugCommand(pathPlannerTrajectory, driveSubsystem::getPose),
                    new InstantCommand(),
                    () -> DEBUG_MODE)),
        new LogCommand("Finished Path."));
    if (withRequirements) {
      addRequirements(driveSubsystem);
    }
  }
}
