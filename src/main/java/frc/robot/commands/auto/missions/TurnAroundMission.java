package frc.robot.commands.auto.missions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.auto.campaign.Mission;
import frc.robot.subsystems.DriveSubsystem;

public class TurnAroundMission extends Mission {
      private final double MAX_SPEED = 0.2;

  private final DriveSubsystem driveSubsystem;
  private final PIDController rotationSpeedController = new PIDController(0.003, 0.0, 0);
  private final Debouncer setpointDebouncer = new Debouncer(0.2);
  private double targetHeading = 0.0;

  public TurnAroundMission(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;

    rotationSpeedController.setTolerance(1);
    rotationSpeedController.enableContinuousInput(0.0, 360.0);
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    targetHeading =
        MathUtil.inputModulus(driveSubsystem.getHeading().getDegrees() + 180.0, 0.0, 360.0);
  }

  @Override
  public void execute() {
    double rotation =
        MathUtil.clamp(
            rotationSpeedController.calculate(
                driveSubsystem.getHeading().getDegrees(), targetHeading),
            -MAX_SPEED,
            MAX_SPEED);

    driveSubsystem.drive(0, 0, rotation);
  }

  @Override
  public boolean isFinished() {
    boolean atSetpoint = setpointDebouncer.calculate(rotationSpeedController.atSetpoint());

    if (atSetpoint) {
      driveSubsystem.drive(0, 0, 0);
    }

    return atSetpoint;
  }

  @Override
  public boolean isSuccess() {
    return true; // Could impliment state to have a true success, but this is just for gags.
  }
}
