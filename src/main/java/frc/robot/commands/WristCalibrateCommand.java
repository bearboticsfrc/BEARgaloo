package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class WristCalibrateCommand extends CommandBase {
  private ManipulatorSubsystem manipulatorSubsystem;

  public WristCalibrateCommand(ManipulatorSubsystem manipulatorSubsystem) {
    this.manipulatorSubsystem = manipulatorSubsystem;
  }

  @Override
  public void initialize() {
    manipulatorSubsystem.adjustWristHeight(-1);
  }

  @Override
  public boolean isFinished() {
    if (manipulatorSubsystem.isWristHome()) {
      manipulatorSubsystem.calibrateWrist();
    }

    return manipulatorSubsystem.isWristHome();
  }
}
