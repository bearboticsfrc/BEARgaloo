package frc.robot.auto.campaign;

public class MissionTree {
  private Mission node;
  private MissionTree successNode = null;
  private MissionTree failureNode = null;

  public MissionTree(Mission node) {
    this.node = node;
  }

  /**
   * @return the successNode
   */
  public MissionTree getSuccessNode() {
    return successNode;
  }

  /**
   * @param successNode the successNode to set
   */
  public MissionTree setSuccessNode(MissionTree successNode) {
    this.successNode = successNode;
    return this;
  }

  /**
   * @return the failureNode
   */
  public MissionTree getFailureNode() {
    return failureNode;
  }

  /**
   * @param failureNode the failureNode to set
   */
  public MissionTree setFailureNode(MissionTree failureNode) {
    this.failureNode = failureNode;
    return this;
  }

  /**
   * @return the node
   */
  public Mission getNode() {
    return node;
  }

  /**
   * @param node the node to set
   */
  public void setNode(Mission node) {
    this.node = node;
  }
}
