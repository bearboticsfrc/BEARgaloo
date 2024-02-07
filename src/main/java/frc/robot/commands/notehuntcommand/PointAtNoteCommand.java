package frc.robot.commands.notehuntcommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class PointAtNoteCommand extends InstantCommand {
  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final PIDController rotSpeedController = new PIDController(0.01, 0, 0.001);

  public PointAtNoteCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;

    addRequirements(driveSubsystem);
    rotSpeedController.setTolerance(2);
  }

  @Override
  public void execute() {
    if (visionSubsystem.getNote() == null) {
      driveSubsystem.drive(0, 0, 0);
      return;
    }

    double rot = visionSubsystem.getNote().getYaw();

    rot = rotSpeedController.calculate(rot, 0);

    driveSubsystem.drive(0, 0, rot, false);
  }

  @Override
  public boolean isFinished() {
    return (rotSpeedController.atSetpoint());
  }
}
