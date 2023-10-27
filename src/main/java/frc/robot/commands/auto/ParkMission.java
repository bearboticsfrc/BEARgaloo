package frc.robot.commands.auto;

import frc.robot.auto.campaign.Mission;
import frc.robot.subsystems.DriveSubsystem;

public class ParkMission extends Mission {
  private DriveSubsystem driveSubsystem;

  public ParkMission(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
  }

  @Override
  public void initialize() {
    driveSubsystem.setParkMode(true);
  }

  @Override
  public boolean isSuccess() {
    return true;
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
