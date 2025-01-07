package frc.robot.commands.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathCommand;
import frc.robot.subsystems.DriveSubsystem;

public class LeaveCommunityTop {
  public static Command get(DriveSubsystem driveSubsystem) {

    PathPlannerPath path = PathPlannerPath.fromPathFile("LeaveCommunityTop");

    return new SequentialCommandGroup(
            new PathCommand(driveSubsystem, path, true, true), driveSubsystem.getDriveStopCommand())
        .withName("LeaveCommunityTop");
  }
}
