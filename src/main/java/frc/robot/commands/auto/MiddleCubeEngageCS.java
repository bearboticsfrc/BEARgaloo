package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class MiddleCubeEngageCS {
  public static Command get(DriveSubsystem driveSubsystem, ManipulatorSubsystem manipulator) {
    return new SequentialCommandGroup(
        manipulator.getShootCubeCommand(),
        StraightToChargeStationFromMiddle.get(driveSubsystem),
        new AutoBalanceCommand(driveSubsystem),
        driveSubsystem.getDriveStopCommand());
  }
}
