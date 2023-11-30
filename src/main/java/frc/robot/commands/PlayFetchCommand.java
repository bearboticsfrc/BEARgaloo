package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class PlayFetchCommand {

  public static Command get(
      DriveSubsystem driveSubsystem,
      ManipulatorSubsystem manipulatorSubsystem,
      Runnable cancelHook) {

    final Command command =
        new SequentialCommandGroup(
                new LogStatusCommandWrapper(
                    manipulatorSubsystem.getHomeAllCommand().withName("HomeCommand")),
                new WaitUntilCommand(manipulatorSubsystem::isWristHome),
                new LogStatusCommandWrapper(
                    manipulatorSubsystem.getShootCubeCommand().withName("ShootCommand")),
                // new WaitCommand(0.5),
                new TurnAroundCommand(driveSubsystem),
                // manipulatorSubsystem.getPickupPositionCommand(),
                // new WaitCommand(0.5),
                // new WaitUntilCommand(manipulatorSubsystem::isPickupReady),
                new LogCommand("Starting Hunting"),
                manipulatorSubsystem.getCubeHuntCommand(driveSubsystem),
                new LogCommand("End Hunting")
                // new WaitCommand(2.0)
                // new
                // LogStatusCommandWrapper(manipulatorSubsystem.getHoldCubeCommand().withName("HoldcubeCommand")),
                // new ConditionalCommand(
                // new InstantCommand(),
                // new InstantCommand(cancelHook::run),
                // manipulatorSubsystem::holdingCube),
                // new LogStatusCommandWrapper(
                // manipulatorSubsystem.getHomeAllCommand().withName("HomeCommand2"))
                )
            .withName("PlayFetch");

    return command;
  }
}
