package frc.robot.subsystems.pose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.fms.AllianceReadyListener;
import frc.robot.location.FieldPositions;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.pose.vision.EstimationRunnable;
import frc.robot.subsystems.pose.vision.RobotCamera;
import frc.robot.subsystems.pose.vision.StringFormatting;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;

public class PoseEstimatorSubsystem extends SubsystemBase implements AllianceReadyListener {
  private AprilTagFieldLayout layout;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final DriveSubsystem driveSubsystem;

  private final StringLogEntry poseLog;
  private final DataLog dataLog;
  private final DoubleLogEntry poseXLog;
  private final DoubleLogEntry poseYLog;
  private final DoubleLogEntry poseAngleDegreesLog;

  

  private List<EstimationRunnable> estimationRunnables = new ArrayList<EstimationRunnable>();
  private List<Notifier> notifiers = new ArrayList<Notifier>();

  public PoseEstimatorSubsystem(DriveSubsystem driveSubsystem, FieldPositions fieldPositions) {
    this.driveSubsystem = driveSubsystem;

    poseEstimator =
        new SwerveDrivePoseEstimator(
            RobotConstants.DRIVE_KINEMATICS, // DriveConstants.DRIVE_KINEMATICS,
            driveSubsystem.getHeading(),
            driveSubsystem.getModulePositions(),
            getInitialPose2d(),
            VisionConstants.STATE_STD_DEVS,
            VisionConstants.VISION_STD_DEVS);

    for (RobotCamera robotCamera : VisionConstants.robotCameras) {
      PhotonCamera photonCamera = new PhotonCamera(robotCamera.getCameraName());

      EstimationRunnable estimatorRunnable =
          new EstimationRunnable(
              robotCamera.getNiceName(), photonCamera, robotCamera.getRobotToCameraTransform());
      estimationRunnables.add(estimatorRunnable);
      Notifier notifier = new Notifier(estimatorRunnable);
      notifiers.add(notifier);

      // Start PhotonVision thread
      notifier.setName(robotCamera.getNiceName());
      notifier.startPeriodic(0.02);
    }
    dataLog = DataLogManager.getLog();
    poseLog = new StringLogEntry(dataLog, "/vision/pose");
    poseXLog = new DoubleLogEntry(dataLog, "/vision/pose/x");
    poseYLog = new DoubleLogEntry(dataLog, "/vision/pose/y");
    poseAngleDegreesLog = new DoubleLogEntry(dataLog, "/vision/pose/angle");
  }

  private void logPose(Pose2d pose) {
    poseLog.append(StringFormatting.poseToString(pose));
    poseXLog.append(pose.getX());
    poseYLog.append(pose.getY());
    poseAngleDegreesLog.append(pose.getRotation().getDegrees());
  }

  private Pose2d getInitialPose2d() {
    return new Pose2d();
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void updateAllianceColor(Alliance alliance) {
    // should this also re-initialize the pose estimators ?
    layout.setOrigin(
        alliance == Alliance.Blue
            ? OriginPosition.kBlueAllianceWallRightSide
            : OriginPosition.kRedAllianceWallRightSide);

    // TODO Auto-generated method stub

  }
}
