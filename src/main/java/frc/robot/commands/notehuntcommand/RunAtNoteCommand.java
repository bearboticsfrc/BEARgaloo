package frc.robot.commands.notehuntcommand;

import frc.robot.commands.DriveToPoseCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RunAtNoteCommand extends DriveToPoseCommand {

  public RunAtNoteCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    super(visionSubsystem.getNotePose(), driveSubsystem);
    addRequirements(driveSubsystem);
    addRequirements(visionSubsystem);
  }
}
