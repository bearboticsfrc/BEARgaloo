package frc.robot.commands.notehuntcommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class NoteHuntCommand extends SequentialCommandGroup {

  private final PointAtNoteCommand pointAtNoteCommand;
  private final RunAtNoteCommand runAtNoteCommand;

  public NoteHuntCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    pointAtNoteCommand = new PointAtNoteCommand(driveSubsystem, visionSubsystem);
    runAtNoteCommand = new RunAtNoteCommand(driveSubsystem, visionSubsystem);
    this.addCommands(pointAtNoteCommand, runAtNoteCommand);

    addRequirements(driveSubsystem);
  }
}
