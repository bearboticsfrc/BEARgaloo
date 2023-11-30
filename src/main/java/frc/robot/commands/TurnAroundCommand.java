package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TurnAroundCommand extends Command {
  private final double MAX_SPEED = 0.2;

  private final DriveSubsystem driveSubsystem;
  private final PIDController rotationSpeedController = new PIDController(0.003, 0.0, 0);
  private final Debouncer setpointDebouncer = new Debouncer(0.2);
  private double targetHeading = 0.0;
  private final BooleanLogEntry logEntry =
      new BooleanLogEntry(
          DataLogManager.getLog(), "/command/TurnAround", "Indicates the command is active");

  private boolean isActive = false;

  public TurnAroundCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;

    rotationSpeedController.setTolerance(1);
    rotationSpeedController.enableContinuousInput(0.0, 360.0);
    addRequirements(driveSubsystem);
    RobotConstants.MANIPULATOR_SYSTEM_TAB.addBoolean("TurnAround", this::getActive);

    logEntry.append(false);
  }

  public boolean getActive() {
    return isActive;
  }

  @Override
  public void initialize() {
    targetHeading =
        MathUtil.inputModulus(driveSubsystem.getHeading().getDegrees() + 180.0, 0.0, 360.0);
    logEntry.append(true);
    isActive = true;
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
  public void end(boolean interrupted) {
    isActive = false;
    logEntry.append(false);
    driveSubsystem.drive(0, 0, 0);
  }
}
