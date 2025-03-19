package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** An instant command to send the message to the DataLogManager */
public class LogCommand extends InstantCommand {
  private final String message;

  public LogCommand(String message) {
    this.message = message;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLogManager.log(message);
  }
}
