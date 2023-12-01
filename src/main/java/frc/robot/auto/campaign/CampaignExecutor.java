package frc.robot.auto.campaign;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;

public class CampaignExecutor extends Command {
  private Campaign campaign;
  private MissionTree nextMission = null;
  private Mission currentMission = null;
  private int successes = 0;

  public CampaignExecutor(Campaign campaign) {
    DataLogManager.log("Campaign -> " + campaign.getName());
    this.campaign = campaign;
  }

  @Override
  public void initialize() {
    nextMission = campaign.getMissions();
    currentMission = nextMission.getNode();

    DataLogManager.log("initalizing command -> " + currentMission.getName());
    currentMission.initialize();
  }

  @Override
  public void execute() {
    currentMission.execute();
    
    if (!currentMission.isFinished() || nextMission == null) {
      return;
    } else {
      currentMission.end(false);
    }

    if (currentMission.isSuccess()) {
      successes += 1;
      MissionTree cMission = nextMission.getSuccessNode();
      String message =
          cMission == null
              ? currentMission.getName()
                  + " was successful. "
                  + "Successfully campaigned "
                  + successes
                  + " nextMission"
              : currentMission.getName()
                  + " was successful. executing mission -> "
                  + nextMission.getSuccessNode().getNode().getName();
      DataLogManager.log(message);

      nextMission = nextMission.getSuccessNode();
    } else {
      MissionTree cMission = nextMission.getFailureNode();
      String message =
          cMission == null
              ? currentMission.getName()
                  + " failed. "
                  + "Successfully campaigned "
                  + successes
                  + " nextMission"
              : currentMission.getName()
                  + " failed. executing mission -> "
                  + nextMission.getFailureNode().getNode().getName();
      DataLogManager.log(message);
      nextMission = nextMission.getFailureNode();
    }

    if (nextMission == null) {
      currentMission = null;
      return; // TODO: Logic needs reworking here
    }

    currentMission = nextMission.getNode();
    currentMission.initialize();
  }

  @Override
  public void end(boolean interrupted) {
    DataLogManager.log("Ended campaign -> " + campaign.getName());
    if (currentMission != null) {
      currentMission.end(false);
    }
  }

  @Override
  public boolean isFinished() {
    return currentMission == null;
  }
}
