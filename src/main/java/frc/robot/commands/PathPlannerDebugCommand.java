package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

/**
 * Command to run in parallel with a trajectory based command. Prints the current trajectory point
 * and the odomentry point.
 */
public class PathPlannerDebugCommand extends Command {

  private PathPlannerTrajectory trajectory;
  private Supplier<Pose2d> poseSupplier;
  private final Timer timer = new Timer();

  public PathPlannerDebugCommand(PathPlannerTrajectory trajectory, Supplier<Pose2d> poseSupplier) {
    this.trajectory = trajectory;
    this.poseSupplier = poseSupplier;
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    double curTime = timer.get();

    State desiredState = trajectory.sample(curTime);
    Pose2d pose = poseSupplier.get();

    System.out.println(
        String.format(
            "trajectory[%05.2f, %05.2f, %05.2f] actual[%05.2f, %05.2f, %05.2f] error[%05.2f, %05.2f, %05.2f]",
            desiredState.getTargetHolonomicPose().getX(),
            desiredState.getTargetHolonomicPose().getY(),
            desiredState.getTargetHolonomicPose().getRotation().getDegrees(),
            pose.getX(),
            pose.getY(),
            pose.getRotation().getDegrees(),
            desiredState.getTargetHolonomicPose().getX() - pose.getX(),
            desiredState.getTargetHolonomicPose().getY() - pose.getY(),
            desiredState.getTargetHolonomicPose().getRotation().getDegrees()
                - pose.getRotation().getDegrees()));
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }
}
