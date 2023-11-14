package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class PlayFetchCommand {

  public static Command get(
      DriveSubsystem driveSubsystem, ManipulatorSubsystem manipulatorSubsystem) {

    return new SequentialCommandGroup(
            manipulatorSubsystem.getHomeAllCommand(),
            new WaitCommand(0.5), // should be wait until home
            manipulatorSubsystem.getShootCubeCommand(),
            new TurnAroundCommand(driveSubsystem),
            manipulatorSubsystem.getPickupPositionCommand(),
            manipulatorSubsystem.getCubeHuntCommand(driveSubsystem),
            manipulatorSubsystem.getHomeAllCommand())
        .withName("PlayFetch");
  }
}
