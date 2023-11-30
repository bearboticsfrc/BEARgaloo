package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class WristCalibrateCommand extends Command {
  private ManipulatorSubsystem manipulatorSubsystem;

  public WristCalibrateCommand(ManipulatorSubsystem manipulatorSubsystem) {
    this.manipulatorSubsystem = manipulatorSubsystem;
  }

  @Override
  public void initialize() {
    manipulatorSubsystem.setWristSpeed(-0.35);
  }

  @Override
  public boolean isFinished() {
    if (manipulatorSubsystem.isWristHome()) {
      manipulatorSubsystem.calibrateWrist();
      manipulatorSubsystem.setWristSpeed(0);
    }

    return manipulatorSubsystem.isWristHome();
  }
}
