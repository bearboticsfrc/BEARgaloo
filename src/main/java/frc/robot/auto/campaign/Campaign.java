package frc.robot.auto.campaign;

import java.lang.reflect.InvocationTargetException;

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

  /**
   * Iterates over all success nodes in the campaign, and sums up all return values of the accessor specified by {@code name}.
   * @param name the simple name of a accessor which returns a double.
   * @return the sum of all accessors on each success node specificed by {@code name} of this tree.
   * @throws NoSuchMethodException
   * @throws IllegalAccessException
   * @throws InvocationTargetException
   */ // TODO: doc is worded poorly
  public double getTotal(String name) throws NoSuchMethodException, IllegalAccessException, InvocationTargetException {
    double sum = missions.getNode().getRuntime();
    MissionTree nextNode = missions.getSuccessNode();

    while (nextNode != null) {
      sum += (double) nextNode.getNode().getClass().getMethod(name).invoke(null);
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
