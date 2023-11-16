package frc.robot.auto.campaign;

import edu.wpi.first.wpilibj2.command.Command;

public abstract class Mission extends Command {
  public int points;
  public double runtime;

  public int getPoints() {
    return points;
  }

  public double getRuntime() {
    return runtime;
  }

  public abstract boolean isSuccess();
}
