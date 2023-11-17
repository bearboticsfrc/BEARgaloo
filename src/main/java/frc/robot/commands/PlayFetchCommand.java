package frc.robot.commands;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class PlayFetchCommand {

  public static Command get(
      DriveSubsystem driveSubsystem, ManipulatorSubsystem manipulatorSubsystem, Runnable cancelHook) {

    final Command command = new SequentialCommandGroup(
            manipulatorSubsystem.getHomeAllCommand(),
            new WaitUntilCommand(manipulatorSubsystem::isWristHome),
            manipulatorSubsystem.getShootCubeCommand(),
            new WaitCommand(0.5),
            new TurnAroundCommand(driveSubsystem),
            manipulatorSubsystem.getPickupPositionCommand(),
            new WaitUntilCommand(manipulatorSubsystem::isPickupReady),
            manipulatorSubsystem.getCubeHuntCommand(driveSubsystem).withTimeout(5.0),
            new ConditionalCommand(new InstantCommand(), 
                  new InstantCommand(() -> cancelHook.run()), 
                  manipulatorSubsystem::holdingCube),
            manipulatorSubsystem.getHomeAllCommand())
        .withName("PlayFetch");

    return command;
  }
}
