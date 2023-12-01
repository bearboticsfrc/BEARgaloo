package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.auto.campaign.Campaign;
import frc.robot.auto.campaign.CommandMission;
import frc.robot.auto.campaign.MissionTree;
import frc.robot.commands.auto.missions.TurnAroundMission;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class PlayFetchFactory {
  static final String NAME = "Play Fetch!";

  public static Campaign get(DriveSubsystem driveSubsystem, ManipulatorSubsystem manipulatorSubsystem) {
    MissionTree homeAllMission = new MissionTree(new CommandMission(getHomeAllCommand(manipulatorSubsystem)));
    final MissionTree pickupCubeMission = new MissionTree(new CommandMission(manipulatorSubsystem.getCubeHuntCommand(driveSubsystem).withTimeout(5)))
    .setSuccessNode(homeAllMission)
    .setFailureNode(new MissionTree(new CommandMission(getHomeAllCommand(manipulatorSubsystem))));
    
    final MissionTree turnAroundMission = new MissionTree(new CommandMission(new TurnAroundMission(driveSubsystem))).setSuccessNode(pickupCubeMission);
    final MissionTree cuebShootMisson = new MissionTree(new CommandMission(manipulatorSubsystem.getShootCubeCommand())).setSuccessNode(turnAroundMission);
    homeAllMission = homeAllMission.setSuccessNode(cuebShootMisson);

    // MissionTree campaignTree = new MissionTree(new CommandMission(manipulatorSubsystem.getCubeHuntCommand(driveSubsystem)))
    //   .chainSuccessNode(new MissionTree(new TurnAroundMission(driveSubsystem))
    //   .chainSuccessNode(new MissionTree(new CommandMission(manipulatorSubsystem.getShootCubeCommand()))
    //   .chainSuccessNode(new MissionTree(new CommandMission(getHomeAllCommand(manipulatorSubsystem))))));
    

    return new Campaign(NAME, homeAllMission);
  }

  private static WrapperCommand getHomeAllCommand(ManipulatorSubsystem manipulatorSubsystem) {
    return new SequentialCommandGroup(manipulatorSubsystem.getHomeAllCommand(), new WaitCommand(0.5)).withName("Home All Mission");
  }
}
