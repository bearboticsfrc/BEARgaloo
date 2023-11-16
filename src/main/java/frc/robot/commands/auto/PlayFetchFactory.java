package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.campaign.Campaign;
import frc.robot.auto.campaign.CommandMission;
import frc.robot.auto.campaign.MissionTree;
import frc.robot.commands.CubeHuntCommand;
import frc.robot.commands.LogCommand;
import frc.robot.commands.auto.missions.TurnAroundMission;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class PlayFetchFactory {
  static final String NAME = "Play Fetch!";

  public static Campaign get(DriveSubsystem driveSubsystem, ManipulatorSubsystem manipulatorSubsystem) {
    final MissionTree homeAllMission = new MissionTree(new CommandMission(getHomeAllCommand(manipulatorSubsystem)));
    final MissionTree cuebShootMisson = new MissionTree(new CommandMission(manipulatorSubsystem.getShootCubeCommand())).setSuccessNode(homeAllMission);
    final MissionTree turnAroundMission = new MissionTree(new CommandMission(new TurnAroundMission(driveSubsystem))).setSuccessNode(cuebShootMisson);
    final MissionTree pickupCubeMission = new MissionTree(new CommandMission(getPickupCubeCommand(driveSubsystem, manipulatorSubsystem))).setSuccessNode(turnAroundMission);

    return new Campaign(NAME, pickupCubeMission);
  }

  private static SequentialCommandGroup getHomeAllCommand(ManipulatorSubsystem manipulatorSubsystem) {
    final SequentialCommandGroup homeAllCommand = new SequentialCommandGroup(manipulatorSubsystem.getHomeAllCommand(), new WaitCommand(0.5));
    homeAllCommand.setName("Home All Mission");
    return homeAllCommand;
  }

  private static SequentialCommandGroup getPickupCubeCommand(DriveSubsystem driveSubsystem, ManipulatorSubsystem manipulatorSubsystem) {
    final SequentialCommandGroup pickupCubeCommand = new SequentialCommandGroup(manipulatorSubsystem.getPickupPositionCommand(), manipulatorSubsystem.getCubeHuntCommand(driveSubsystem));
    pickupCubeCommand.setName("Pickup Cube Mission");
    return pickupCubeCommand;
  }
}
