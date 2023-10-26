package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import frc.robot.auto.campaign.Campaign;
import frc.robot.auto.campaign.CommandMission;
import frc.robot.auto.campaign.Mission;
import frc.robot.auto.campaign.MissionTree;
import frc.robot.constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class MiddleCubeEngageCSCampaign {
  static final String NAME = "Middle Cube Engage Charge Station";

  public static Campaign get(DriveSubsystem driveSubsystem) {
    final MissionTree parkMissionNode = new MissionTree(new ParkMission(driveSubsystem));
    final MissionTree autoBalanceMissionNode =
        new MissionTree(new AutoBalanceMission(driveSubsystem)).setSuccessNode(parkMissionNode);
    final MissionTree pathMissionNode =
        new MissionTree(getPathMission(driveSubsystem)).setSuccessNode(autoBalanceMissionNode);

    return new Campaign(NAME, pathMissionNode);
  }

  public static Mission getPathMission(DriveSubsystem driveSubsystem) {
    PathPlannerTrajectory pathPlannerTrajectory =
        PathPlanner.loadPath(
            "StraightToChargeStationFromMiddle",
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new CommandMission(
        new PathCommandMission(driveSubsystem, pathPlannerTrajectory) {
          @Override
          public boolean isSuccess() {
            return true;
          }
        }.withName("StraightToChargeStationFromMiddle"));
  }
}
