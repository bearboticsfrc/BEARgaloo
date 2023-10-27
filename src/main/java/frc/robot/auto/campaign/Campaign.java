package frc.robot.auto.campaign;

public class Campaign {
  private MissionTree missions;
  private String name;

  public Campaign(String name, MissionTree missions) {
    this.name = name;
    this.missions = missions;
  }

  public Campaign setMissionTree(MissionTree missions) {
    this.missions = missions;
    return this;
  }

  /** Schedule the campaign for running. */
  public void schedule() {}

  public double getTotalRuntime() {
    double sum = missions.getNode().getRuntime();
    MissionTree nextNode = missions.getSuccessNode();

    while (nextNode != null) {
      sum += nextNode.getNode().getRuntime();
      nextNode = nextNode.getSuccessNode();
    }

    return sum;
  }

  public double getTotalPoints() { // TODO: fix DRY
    double sum = missions.getNode().getPoints();
    MissionTree nextNode = missions.getSuccessNode();

    while (nextNode != null) {
      sum += nextNode.getNode().getPoints();
      nextNode = nextNode.getSuccessNode();
    }

    return sum;
  }

  public String getName() {
    return name;
  }

  public MissionTree getMissions() {
    return missions;
  }
}
