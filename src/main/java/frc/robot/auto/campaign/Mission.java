package frc.robot.auto.campaign;

import edu.wpi.first.wpilibj2.command.CommandBase;

public abstract class Mission extends CommandBase {
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
