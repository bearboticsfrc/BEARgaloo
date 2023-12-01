package frc.robot.commands.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.campaign.Campaign;
import frc.robot.auto.campaign.CommandMission;
import frc.robot.auto.campaign.Mission;
import frc.robot.auto.campaign.MissionTree;
import frc.robot.commands.auto.missions.AutoBalanceMission;
import frc.robot.commands.auto.missions.ParkMission;
import frc.robot.commands.auto.missions.PathCommandMission;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class MiddleCubeEngageCSFactory {
  static final String NAME = "Middle Cube Engage Charge Station";

  public static Campaign get(
      DriveSubsystem driveSubsystem, ManipulatorSubsystem manipulatorSubsystem) {

    final MissionTree campaignTree =
        new MissionTree(new CommandMission(manipulatorSubsystem.getShootCubeCommand()))
            .chainSuccessNode(new MissionTree(getPathMission(driveSubsystem)))
            .chainSuccessNode(new MissionTree(new AutoBalanceMission(driveSubsystem))
            .chainSuccessNode(new MissionTree(new ParkMission(driveSubsystem))));

    return new Campaign(NAME, campaignTree);
  }

  public static Mission getPathMission(DriveSubsystem driveSubsystem) {
    PathPlannerPath path = PathPlannerPath.fromPathFile("StraightToChargeStationFromMiddle");

    return new CommandMission(
        new SequentialCommandGroup(
                new PathCommandMission(driveSubsystem, path), driveSubsystem.getDriveStopCommand())
            .withName("StraightToChargeStationFromMiddle"));
  }
}
