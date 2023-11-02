package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import frc.robot.auto.campaign.Campaign;
import frc.robot.auto.campaign.CommandMission;
import frc.robot.auto.campaign.Mission;
import frc.robot.auto.campaign.MissionTree;
import frc.robot.commands.auto.missions.AutoBalanceMission;
import frc.robot.commands.auto.missions.ParkMission;
import frc.robot.commands.auto.missions.PathCommandMission;
import frc.robot.constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class MiddleCubeEngageCSFactory {
  static final String NAME = "Middle Cube Engage Charge Station";

  public static Campaign get(
      DriveSubsystem driveSubsystem, ManipulatorSubsystem manipulatorSubsystem) {

    final MissionTree parkMissionNode = new MissionTree(new ParkMission(driveSubsystem));

    final MissionTree autoBalanceMissionNode =
        new MissionTree(new AutoBalanceMission(driveSubsystem)).setSuccessNode(parkMissionNode);

    final MissionTree pathMissionNode =
        new MissionTree(getPathMission(driveSubsystem)).setSuccessNode(autoBalanceMissionNode);

    final MissionTree cubeShootMissionNode =
        new MissionTree(
                new CommandMission(
                    manipulatorSubsystem.getShootCubeCommand().withName("Shoot Mission")))
            .setSuccessNode(pathMissionNode);

    return new Campaign(NAME, cubeShootMissionNode);
  }

  public static Mission getPathMission(DriveSubsystem driveSubsystem) {
    PathPlannerTrajectory pathPlannerTrajectory =
        PathPlanner.loadPath(
            "StraightToChargeStationFromMiddle",
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new CommandMission(
        new PathCommandMission(driveSubsystem, pathPlannerTrajectory)
            .withName("StraightToChargeStationFromMiddle"));
  }
}
