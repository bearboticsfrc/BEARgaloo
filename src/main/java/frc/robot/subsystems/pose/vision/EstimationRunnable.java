package frc.robot.subsystems.pose.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.VisionConstants;
import frc.robot.fms.AllianceColor;
import frc.robot.fms.AllianceReadyListener;
import java.io.IOException;
import java.util.concurrent.atomic.AtomicReference;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class EstimationRunnable implements Runnable, AllianceReadyListener {
  private final PhotonPoseEstimator photonPoseEstimator;

  private final PhotonCamera photonCamera;
  private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose =
      new AtomicReference<EstimatedRobotPose>();

  private AprilTagFieldLayout layout;

  public EstimationRunnable(String name, PhotonCamera cameraName, Transform3d robotToCamera) {
    this.photonCamera = cameraName;
    PhotonPoseEstimator photonPoseEstimator = null;
    try {
      layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      if (AllianceColor.alliance == Alliance.Red) {
        layout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
      } else {
        layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      }
      AllianceColor.addListener(this);
      if (photonCamera != null) {
        photonPoseEstimator =
            new PhotonPoseEstimator(
                layout, PoseStrategy.MULTI_TAG_PNP, photonCamera, robotToCamera);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
      }
    } catch (IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      photonPoseEstimator = null;
    }
    this.photonPoseEstimator = photonPoseEstimator;
  }

  @Override
  public void updateAllianceColor(Alliance alliance) {
    // should this also re-initialize the pose estimators ?
    layout.setOrigin(
        alliance == Alliance.Blue
            ? OriginPosition.kBlueAllianceWallRightSide
            : OriginPosition.kRedAllianceWallRightSide);
  }

  @Override
  public void run() {
    // Get AprilTag data
    if (photonPoseEstimator != null && photonCamera != null) {
      var photonResults = photonCamera.getLatestResult();
      if (photonResults.hasTargets()
          && (photonResults.targets.size() > 1
              || photonResults.targets.get(0).getPoseAmbiguity()
                  < VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD)) {
        photonPoseEstimator
            .update(photonResults)
            .ifPresent(
                estimatedRobotPose -> {
                  Pose3d estimatedPose = estimatedRobotPose.estimatedPose;
                  // Make sure the measurement is on the field
                  if (estimatedPose.getX() > 0.0
                      && estimatedPose.getX() <= VisionConstants.FIELD_LENGTH_METERS
                      && estimatedPose.getY() > 0.0
                      && estimatedPose.getY() <= VisionConstants.FIELD_WIDTH_METERS) {
                    atomicEstimatedRobotPose.set(estimatedRobotPose);
                  }
                });
      }
    }
  }

  public EstimatedRobotPose grabLatestEstimatedPose() {
    return atomicEstimatedRobotPose.getAndSet(null);
  }
}
