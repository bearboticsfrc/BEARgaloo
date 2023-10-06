package frc.robot.subsystems.pose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.fms.AllianceColor;
import frc.robot.fms.AllianceReadyListener;
import frc.robot.location.FieldPositions;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.pose.vision.StringFormatting;

public class PoseEstimatorSubsystem extends SubsystemBase implements AllianceReadyListener {
  private AprilTagFieldLayout layout;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final DriveSubsystem driveSubsystem;

  private final StringLogEntry poseLog;
  private final DataLog dataLog;
  private final DoubleLogEntry poseXLog;
  private final DoubleLogEntry poseYLog;
  private final DoubleLogEntry poseAngleDegreesLog;
  private final Field2d field2d = new Field2d();

  public static final int RED_DOUBLE_SUBSTATION_FIDUCIAL_ID = 5;
  public static final int BLUE_DOUBLE_SUBSTATION_FIDUCIAL_ID = 4;

  public PoseEstimatorSubsystem(DriveSubsystem driveSubsystem, FieldPositions fieldPositions) {
    this.driveSubsystem = driveSubsystem;

    layout = fieldPositions.getLayout();

    AllianceColor.addListener(this);
    ShuffleboardTab tab = Shuffleboard.getTab("Vision");

    poseEstimator =
        new SwerveDrivePoseEstimator(
            RobotConstants.DRIVE_KINEMATICS, // DriveConstants.DRIVE_KINEMATICS,
            driveSubsystem.getHeading(),
            driveSubsystem.getModulePositions(),
            getInitialPose2d(),
            VisionConstants.STATE_STD_DEVS,
            VisionConstants.VISION_STD_DEVS);

    tab.addString("Pose", () -> StringFormatting.poseToString(getPose()))
        .withPosition(0, 0)
        .withSize(2, 1);
    tab.addString("Drive Pose", () -> StringFormatting.poseToString(driveSubsystem.getPose()))
        .withPosition(0, 1)
        .withSize(2, 1);
    tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4).withWidget(BuiltInWidgets.kField);

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

  @Override
  public void periodic() {
    updateOdometry();
    estimatorChecker();

    field2d.setRobotPose(getPose());
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

  public void updateOdometry() {
    poseEstimator.update(driveSubsystem.getHeading(), driveSubsystem.getModulePositions());
  }

  public Pose2d getSubstationPose() {
    int doubleStationFiducialId = FieldPositions.BLUE_DOUBLE_SUBSTATION_FIDUCIAL_ID;
    if (AllianceColor.alliance == Alliance.Red) {
      doubleStationFiducialId = FieldPositions.RED_DOUBLE_SUBSTATION_FIDUCIAL_ID;
    }
    return layout.getTagPose(doubleStationFiducialId).get().toPose2d();
  }

  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(
        driveSubsystem.getHeading(), driveSubsystem.getModulePositions(), newPose);
  }

  public String getPoseString() {
    return getPoseString(getPose());
  }

  public String getPoseString(Pose2d pose) {
    return String.format(
        "(%.2f, %.2f) %.2f degrees", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }

  public double getDistanceToSubstationWall() {
    Transform2d transform = new Transform2d(getPose(), getSubstationPose());
    return transform.getX() - VisionConstants.HALF_ROBOT_LENGTH;
  }

  public double getLateralToSubstationWall() {
    Transform2d transform = new Transform2d(getPose(), getSubstationPose());
    return transform.getY();
  }

  public void configureLimelightHelpers() {
    LimelightResults llresults = LimelightHelpers.getLatestResults(VisionConstants.limelightName);

    int numAprilTags = llresults.targetingResults.targets_Fiducials.length;
  }

  public Pose3d getRobotFieldSpacPose3d() {
    LimelightTarget_Fiducial limelightTargetFiducial = new LimelightTarget_Fiducial();

    Pose3d botPose3dTargetSpace = limelightTargetFiducial.getRobotPose_FieldSpace();
    return new Pose3d();
  }

  public Pose2d getRobotFieldSpace2d() {
    return new Pose2d();
  }

  public void estimatorChecker() {
    Pose2d visionPose = getRobotFieldSpacPose3d().toPose2d();

    if (visionPose != null) {
      Pose2d adjustedPose =
          new Pose2d(visionPose.getX(), visionPose.getY(), driveSubsystem.getHeading());
      double[] botpose = LimelightHelpers.getBotPose(VisionConstants.limelightName);

      logPose(adjustedPose);
      poseEstimator.addVisionMeasurement(adjustedPose, botpose[6]); // need timestamp and confidence
    }
  }
}
