package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToPoseCommand extends InstantCommand {
  private final DriveSubsystem driveSubsystem;
  private final Pose2d targetPose;
  private PIDController xController = new PIDController(0, 0, 0);
  private PIDController yController = new PIDController(0, 0, 0);

  public DriveToPoseCommand(Pose2d targetPose, DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.targetPose = targetPose;
    xController.setTolerance(2);
    yController.setTolerance(2);
    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {

    Pose2d currentPose = driveSubsystem.getPose();

    double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());

    double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());

    driveSubsystem.drive(xSpeed, ySpeed, 0);
  }

  @Override
  public boolean isFinished() {
    return (xController.atSetpoint() && yController.atSetpoint());
  }
}
