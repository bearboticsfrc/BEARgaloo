package frc.robot.auto.campaign;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;

public class CampaignExecutor extends Command {
  private Campaign campaign;
  private MissionTree missions = null;
  private Mission currentMission = null;
  private int successes = 0;

  public CampaignExecutor(Campaign campaign) {
    DataLogManager.log("Campaign -> " + campaign.getName());
    this.campaign = campaign;
  }

  @Override
  public void initialize() {
    missions = campaign.getMissions();
    currentMission = missions.getNode();
    DataLogManager.log("initalizing command -> " + currentMission.getName());
    currentMission.initialize();
  }

  @Override
  public void execute() {
    currentMission.execute();
    
    if (!currentMission.isFinished() || missions == null) {
      return;
    } else {
      currentMission.end(false);
    }

    if (currentMission.isSuccess()) {
      successes += 1;
      MissionTree cMission = missions.getSuccessNode();
      String message =
          cMission == null
              ? currentMission.getName()
                  + " was successful. "
                  + "Successfully campaigned "
                  + successes
                  + " missions"
              : currentMission.getName()
                  + " was successful. executing mission -> "
                  + missions.getSuccessNode().getNode().getName();
      DataLogManager.log(message);

      missions = missions.getSuccessNode();
    } else {
      MissionTree cMission = missions.getFailureNode();
      String message =
          cMission == null
              ? currentMission.getName()
                  + " failed. "
                  + "Successfully campaigned "
                  + successes
                  + " missions"
              : currentMission.getName()
                  + " failed. executing mission -> "
                  + missions.getFailureNode().getNode().getName();
      DataLogManager.log(message);
      missions = missions.getFailureNode();
    }

    if (missions == null) {
      return; // TODO: Logic needs reworking here
    }

    currentMission = missions.getNode();
    currentMission.initialize();
  }

  @Override
  public void end(boolean interrupted) {
    if (currentMission != null) {
      currentMission.end(false);
    }
  }

  @Override
  public boolean isFinished() {
    return currentMission == null;
  }
}
