package frc.robot.subsystems.pose.vision;

import edu.wpi.first.math.geometry.Transform3d;

public class RobotCamera {
  private String niceName;
  private String cameraName;
  private Transform3d robotToCameraTransform;

  public RobotCamera(String niceName, String cameraName, Transform3d robotToCameraTransform) {
    this.niceName = niceName;
    this.cameraName = cameraName;
    this.robotToCameraTransform = robotToCameraTransform;
  }

  public String getNiceName() {
    return niceName;
  }

  public String getCameraName() {
    return cameraName;
  }

  public Transform3d getRobotToCameraTransform() {
    return robotToCameraTransform;
  }
}
