package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import java.util.HashMap;

public class AutoBalanceCommand extends Command {
  private final double MAX_SPEED = 0.3;

  private final DriveSubsystem driveSubsystem;
  private final PIDController pitchSpeedController = new PIDController(0.010, 0.0, 0);
  private final Debouncer setpointDebouncer = new Debouncer(0.1);

  private HashMap<String, DataLogEntry> dataLogs = new HashMap<String, DataLogEntry>();

  public AutoBalanceCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;

    pitchSpeedController.setTolerance(2);

    setupDataLogging(DataLogManager.getLog());
    addRequirements(driveSubsystem);
  }

  private void setupDataLogging(DataLog log) {
    dataLogs.put("AUTO_BALANCE/ACTIVE", new BooleanLogEntry(log, "AUTO_BALANCE/ACTIVE"));
    dataLogs.put(
        "AUTO_BALANCE/PITCH_SETPOINT", new BooleanLogEntry(log, "AUTO_BALANCE/PITCH_SETPOINT"));
    dataLogs.put("AUTO_BALANCE/SPEED", new DoubleLogEntry(log, "AUTO_BALANCE/SPEED"));
  }

  private void updateDataLogs(double xSpeed) {
    ((BooleanLogEntry) dataLogs.get("AUTO_BALANCE/ACTIVE")).append(!this.isFinished());
    ((BooleanLogEntry) dataLogs.get("AUTO_BALANCE/PITCH_SETPOINT"))
        .append(pitchSpeedController.atSetpoint());
    ((DoubleLogEntry) dataLogs.get("AUTO_BALANCE/SPEED")).append(xSpeed);
  }

  @Override
  public void execute() {
    double xSpeed =
        MathUtil.clamp(
            pitchSpeedController.calculate(driveSubsystem.getRoll(), 0), -MAX_SPEED, MAX_SPEED);

    updateDataLogs(xSpeed);
    driveSubsystem.drive(xSpeed, 0, 0);
  }

  @Override
  public boolean isFinished() {
    boolean atPitchSetpoint = setpointDebouncer.calculate(pitchSpeedController.atSetpoint());

    if (atPitchSetpoint) {
      driveSubsystem.setParkMode(atPitchSetpoint);
    }

    return atPitchSetpoint;
  }
}
