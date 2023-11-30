package frc.robot.commands;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.constants.RobotConstants;
import java.util.Set;

/** An instant command to send the message to the DataLogManager */
public class LogStatusCommandWrapper extends Command {

  private Command command;
  private boolean isActive = false;

  private BooleanLogEntry logEntry =
      new BooleanLogEntry(
          DataLogManager.getLog(), "/command/CubeHunt", "Indicates the command is active");

  public LogStatusCommandWrapper(Command command) {
    this.command = command;
    RobotConstants.MANIPULATOR_SYSTEM_TAB.addBoolean(command.getName(), this::getActive);
    logEntry =
        new BooleanLogEntry(
            DataLogManager.getLog(),
            "/command/" + command.getName(),
            "Indicates the command is active");
    logEntry.append(false);
  }

  public boolean getActive() {
    return isActive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    command.initialize();
    isActive = true;
    logEntry.append(true);
  }

  public void execute() {
    command.execute();
  }

  public void end(boolean interrupted) {
    command.end(interrupted);
    isActive = false;
    logEntry.append(false);
  }

  public boolean isFinished() {
    return command.isFinished();
  }

  public Set<Subsystem> getRequirements() {
    return command.getRequirements();
  }

  public String getName() {
    return command.getName();
  }

  public String getSubsystem() {
    return command.getSubsystem();
  }

  public WrapperCommand ignoringDisable(boolean doesRunWhenDisabled) {
    return command.ignoringDisable(doesRunWhenDisabled);
  }

  public WrapperCommand handleInterrupt(Runnable handler) {
    return command.handleInterrupt(handler);
  }

  public void schedule() {
    command.schedule();
  }

  public void cancel() {
    command.cancel();
  }

  public boolean isScheduled() {
    return command.isScheduled();
  }

  public boolean hasRequirement(Subsystem requirement) {
    return command.hasRequirement(requirement);
  }

  public void initSendable(SendableBuilder builder) {
    command.initSendable(builder);
  }
}
