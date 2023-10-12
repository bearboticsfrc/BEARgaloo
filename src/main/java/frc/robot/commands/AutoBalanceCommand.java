package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalanceCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private static final double maxSpeed = 0.3;
  private static final double pitchSpeedControllerP = 0.015;
  private static final double pitchSpeedControllerI = 0;
  private static final double pitchSpeedControllerD = 0.0006;
  private static final double pitchSpeedControllerPositionTolerance = 2.1;

  private final PIDController pitchSpeedController =
      new PIDController(pitchSpeedControllerP, pitchSpeedControllerI, pitchSpeedControllerD);
  private final Debouncer setpointDebouncer = new Debouncer(0.1);

  private BooleanLogEntry booleanLog;
  private BooleanLogEntry setpointLog;
  private DoubleLogEntry speedLog;

  public AutoBalanceCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);

    DataLog log = DataLogManager.getLog();
    booleanLog = new BooleanLogEntry(log, "autobalance/active");
    setpointLog = new BooleanLogEntry(log, "autobalance/setpoint");
    speedLog = new DoubleLogEntry(log, "autobalance/speed");
    booleanLog.append(false);
    setpointLog.append(false);
    speedLog.append(0.0);

    setupPitchController();
  }

  @Override
  public void initialize() {
    booleanLog.append(true);
  }

  private void setupPitchController() {
    pitchSpeedController.setTolerance(pitchSpeedControllerPositionTolerance);
  }
  // 14 *
  @Override
  public void execute() {
    // drive forward
    // until pitch < 2
    // Pitch and roll got reversed somehow

    double pitch = driveSubsystem.getPitch();
    double xSpeed = MathUtil.clamp(pitchSpeedController.calculate(pitch, 0), -maxSpeed, maxSpeed);

    speedLog.append(xSpeed);
    driveSubsystem.drive(-xSpeed, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    boolean setpointDebounced = setpointDebouncer.calculate(pitchSpeedController.atSetpoint());

    driveSubsystem.setParkMode(setpointDebounced);
    setpointLog.append(pitchSpeedController.atSetpoint());
    booleanLog.append(setpointDebounced);
    return setpointDebounced;
  }
}
