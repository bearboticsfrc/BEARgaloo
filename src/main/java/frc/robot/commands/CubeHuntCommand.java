package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.LimelightHelpers;
import java.util.function.BooleanSupplier;

public class CubeHuntCommand extends Command {

  private final DriveSubsystem driveSubsystem;
  private final BooleanSupplier hasCube;
  private final String LIMELIGHT_NAME = "limelight";
  private final PIDController rotSpeedController = new PIDController(0.001, 0, 0);
  private final PIDController xSpeedController = new PIDController(0.1, 0, 0);

  public CubeHuntCommand(DriveSubsystem driveSubsystem, BooleanSupplier hasCube) {
    this.driveSubsystem = driveSubsystem;
    this.hasCube = hasCube;

    addRequirements(driveSubsystem);
    rotSpeedController.setTolerance(2);
    xSpeedController.setTolerance(2);
  }

  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex(LIMELIGHT_NAME, 0);
  }

  @Override
  public void execute() {
    if (!LimelightHelpers.getTV(LIMELIGHT_NAME)) {
      driveSubsystem.drive(0, 0, 0);
      return;
    }

    double targetY = LimelightHelpers.getTY(LIMELIGHT_NAME);
    double targetX = LimelightHelpers.getTX(LIMELIGHT_NAME);

    double xSpeed = -xSpeedController.calculate(targetY, 0);
    double rot = rotSpeedController.calculate(targetX, 0);

    if (xSpeedController.atSetpoint()) {
      xSpeed += 1;
    }

    driveSubsystem.drive(xSpeed, 0, rot, false);
  }

  @Override
  public boolean isFinished() {
    return hasCube.getAsBoolean();
  }
}
