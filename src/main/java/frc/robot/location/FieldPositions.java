package frc.robot.location;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.fms.AllianceColor;
import frc.robot.fms.AllianceReadyListener;
import frc.robot.subsystems.pose.vision.StringFormatting;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public class FieldPositions implements AllianceReadyListener {

  private static final Rotation2d ROTATION_180 = Rotation2d.fromDegrees(180.0);

  private static double GRID_EDGE_TO_TAG = 0.361696;
  private static double ROBOT_LENGTH = .76;
  private static double BUFFER_TO_GRID = .08;
  private static double BLUE_X_POSITION;
  private static double RED_X_POSITION;
  private static double DISTANCE_BETWEEN_NODES = 0.5588;
  private static double BLUE_FIRST_NODE_Y;
  private static double RED_FIRST_NODE_Y;

  public static Pose2d blueStagingMark1 = new Pose2d(7.12, .92, new Rotation2d());
  public static Pose2d blueStagingMark2 = new Pose2d(7.12, 2.14, new Rotation2d());
  public static Pose2d blueStagingMark3 = new Pose2d(7.12, 3.36, new Rotation2d());
  public static Pose2d blueStagingMark4 = new Pose2d(7.12, 4.58, new Rotation2d());

  public static Pose2d redStagingMark1 = new Pose2d(9.42, .92, new Rotation2d());
  public static Pose2d redStagingMark2 = new Pose2d(9.42, 2.14, new Rotation2d());
  public static Pose2d redStagingMark3 = new Pose2d(9.42, 3.36, new Rotation2d());
  public static Pose2d redStagingMark4 = new Pose2d(9.42, 4.58, new Rotation2d());

  private final Map<Integer, Pose2d> redGridPoseMap = new HashMap<Integer, Pose2d>();
  private final Map<Integer, Pose2d> blueGridPoseMap = new HashMap<Integer, Pose2d>();

  public final List<Pose2d> redPoseList = new ArrayList<Pose2d>();
  public final List<Pose2d> bluePoseList = new ArrayList<Pose2d>();

  public final List<Pose2d> redSubstationPoseList = new ArrayList<Pose2d>();
  public final List<Pose2d> blueSubstationPoseList = new ArrayList<Pose2d>();

  public static final int RED_DOUBLE_SUBSTATION_FIDUCIAL_ID = 5;
  public static final int BLUE_DOUBLE_SUBSTATION_FIDUCIAL_ID = 4;

  public static double FIELD_LENGTH = 16.54175;

  private AprilTagFieldLayout layout;

  public FieldPositions() {
    this(AllianceColor.alliance == Alliance.Red);
  }

  public FieldPositions(boolean redOrigin) {
    initializeLayoutAndPoses(redOrigin);
    AllianceColor.addListener(this);
  }

  private void initializeLayoutAndPoses(boolean redOrigin) {
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      setOrigin(redOrigin);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
    }
    RED_FIRST_NODE_Y = layout.getTagPose(3).get().getY() - DISTANCE_BETWEEN_NODES;
    double GRID_TAG_RED_X_POSITION = layout.getTagPose(3).get().getX();
    RED_X_POSITION =
        GRID_TAG_RED_X_POSITION + GRID_EDGE_TO_TAG + (ROBOT_LENGTH / 2) + BUFFER_TO_GRID; // 1

    BLUE_FIRST_NODE_Y = layout.getTagPose(8).get().getY() - DISTANCE_BETWEEN_NODES;
    double GRID_TAG_BLUE_X_POSITION = layout.getTagPose(8).get().getX();
    BLUE_X_POSITION =
        GRID_TAG_BLUE_X_POSITION + GRID_EDGE_TO_TAG + (ROBOT_LENGTH / 2) + BUFFER_TO_GRID;

    redGridPoseMap.clear();
    blueGridPoseMap.clear();
    redPoseList.clear();
    bluePoseList.clear();
    populateGridPoseMap();
    populateGlobalPoseMap();
    populateSubstationList();
  }

  public AprilTagFieldLayout getLayout() {
    return layout;
  }

  public void updateAllianceColor(Alliance alliance) {
    initializeLayoutAndPoses(AllianceColor.alliance == Alliance.Red);
  }

  public Map<Integer, Pose2d> getGridPoseMap() {
    return (AllianceColor.alliance == DriverStation.Alliance.Blue
        ? blueGridPoseMap
        : redGridPoseMap);
  }

  public void populateGridPoseMap() {
    for (int i = 1; i <= 9; i++) {
      blueGridPoseMap.put(
          i,
          new Pose2d(
              BLUE_X_POSITION,
              BLUE_FIRST_NODE_Y + (DISTANCE_BETWEEN_NODES * (9 - i)),
              ROTATION_180));
      redGridPoseMap.put(
          i,
          new Pose2d(
              RED_X_POSITION, RED_FIRST_NODE_Y + (DISTANCE_BETWEEN_NODES * (9 - i)), ROTATION_180));
    }
  }

  public void populateGlobalPoseMap() {
    for (Pose2d pose : redGridPoseMap.values()) {
      redPoseList.add(pose);
    }
    for (Pose2d pose : blueGridPoseMap.values()) {
      bluePoseList.add(pose);
    }
    bluePoseList.add(getBlueLoadingStationInside());
    bluePoseList.add(getBlueLoadingStationOutside());
    redPoseList.add(getRedLoadingStationInside());
    redPoseList.add(getRedLoadingStationOutside());
  }

  public void populateSubstationList() {
    blueSubstationPoseList.add(getBlueLoadingStationOutside());
    redSubstationPoseList.add(getRedLoadingStationOutside());
  }

  public List<Pose2d> getPoseList() {
    if (AllianceColor.alliance == Alliance.Blue) {
      return bluePoseList;
    }
    return redPoseList;
  }

  public List<Pose2d> getSubstationList() {
    if (AllianceColor.alliance == Alliance.Blue) {
      return blueSubstationPoseList;
    }
    return redSubstationPoseList;
  }

  // for blue inside, want 15,6
  // "x": 16.178784,
  // "y": 6.749796,
  public Pose2d getBlueLoadingStationInside() {
    Pose2d tagPose = layout.getTagPose(BLUE_DOUBLE_SUBSTATION_FIDUCIAL_ID).get().toPose2d();
    Transform2d tagToPositionTransform =
        new Transform2d(new Translation2d(1.0, .75), Rotation2d.fromDegrees(180.0));
    return tagPose.plus(tagToPositionTransform);
  }

  public Pose2d getBlueLoadingStationOutside() {
    Pose2d tagPose = layout.getTagPose(BLUE_DOUBLE_SUBSTATION_FIDUCIAL_ID).get().toPose2d();
    Transform2d tagToPositionTransform =
        new Transform2d(new Translation2d(1.0, -.75), Rotation2d.fromDegrees(180.0));
    return tagPose.plus(tagToPositionTransform);
  }

  public Pose2d getRedLoadingStationInside() {
    Pose2d tagPose = layout.getTagPose(RED_DOUBLE_SUBSTATION_FIDUCIAL_ID).get().toPose2d();
    Transform2d tagToPositionTransform =
        new Transform2d(new Translation2d(1.0, -.75), Rotation2d.fromDegrees(180.0));
    return tagPose.plus(tagToPositionTransform);
  }

  public Pose2d getRedLoadingStationOutside() {
    Pose2d tagPose = layout.getTagPose(RED_DOUBLE_SUBSTATION_FIDUCIAL_ID).get().toPose2d();
    Transform2d tagToPositionTransform =
        new Transform2d(new Translation2d(1.0, .75), Rotation2d.fromDegrees(180.0));
    return tagPose.plus(tagToPositionTransform);
  }

  public Pose2d getLoadingStationInside() {
    if (AllianceColor.alliance == Alliance.Blue) {
      return getBlueLoadingStationInside();
    }
    return getRedLoadingStationInside();
  }

  public Pose2d getLoadingStationOutside() {
    if (AllianceColor.alliance == Alliance.Blue) {
      return getBlueLoadingStationOutside();
    }
    return getRedLoadingStationOutside();
  }

  public Pose2d getSubstationAprilTagPose() {
    int doubleStationFiducialId = BLUE_DOUBLE_SUBSTATION_FIDUCIAL_ID;
    if (AllianceColor.alliance == Alliance.Red) {
      doubleStationFiducialId = RED_DOUBLE_SUBSTATION_FIDUCIAL_ID;
    }
    return layout.getTagPose(doubleStationFiducialId).get().toPose2d();
  }

  public Pose2d getRedStagingMark1() {
    return new Pose2d(9.51, 4.61, new Rotation2d());
  }

  public Pose2d getRedStagingMark2() {
    return new Pose2d(9.51, 3.37, new Rotation2d());
  }

  public Pose2d getRedStagingMark3() {
    return new Pose2d(9.51, 2.14, new Rotation2d());
  }

  public Pose2d getRedStagingMark4() {
    return new Pose2d(9.51, 0.91, new Rotation2d());
  }

  public Optional<Pose3d> getTagPose(int tag) {
    return layout.getTagPose(tag);
  }

  public void setOrigin(boolean red) {
    if (red) {
      layout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
    } else {
      layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    }
  }

  /**
   * Just some test routines to get my head around the pose translations.
   *
   * @param args
   */
  public static void main(String args[]) {
    FieldPositions blueFieldPositions = new FieldPositions();

    FieldPositions redFieldPositions = new FieldPositions(true);

    System.out.println("Blue origin");
    System.out.println("BLUE");
    for (Pose2d pose : blueFieldPositions.bluePoseList) {
      System.out.println(StringFormatting.poseToString(pose));
    }

    System.out.println("------------------------");

    System.out.println("Red origin");
    System.out.println("RED");
    for (Pose2d pose : redFieldPositions.redPoseList) {
      System.out.println(StringFormatting.poseToString(pose));
    }

    System.out.println("------------------------");

    AllianceColor.alliance = Alliance.Blue;
    FieldPositions fieldPositions = new FieldPositions();
    System.out.println("Blue origin");
    for (Pose2d pose : fieldPositions.getPoseList()) {
      System.out.println(StringFormatting.poseToString(pose));
    }
    for (Pose2d pose : fieldPositions.redPoseList) {
      System.out.println(
          StringFormatting.poseToString(pose)
              + " -> "
              + StringFormatting.poseToString(AllianceFlipUtil.apply(pose)));
    }

    AllianceColor.alliance = Alliance.Red;
    fieldPositions = new FieldPositions();

    System.out.println("Red origin");
    System.out.println("RED");
    for (Pose2d pose : fieldPositions.getPoseList()) {
      System.out.println(StringFormatting.poseToString(pose));
    }
    System.out.println(" blue pose list .... ");

    for (Pose2d pose : fieldPositions.bluePoseList) {
      System.out.println(
          StringFormatting.poseToString(pose)
              + " -> "
              + StringFormatting.poseToString(AllianceFlipUtil.apply(pose)));
    }

    Pose2d myPose = new Pose2d(1, 1, new Rotation2d(0.0));
    System.out.println(
        StringFormatting.poseToString(myPose)
            + " -> "
            + StringFormatting.poseToString(AllianceFlipUtil.apply(myPose)));

    System.out.println(
        "Tag 1  RED" + StringFormatting.poseToString(redFieldPositions.getTagPose(1).get()));
    System.out.println(
        "Tag 1 BLUE" + StringFormatting.poseToString(blueFieldPositions.getTagPose(1).get()));

    System.out.println(
        "Tag 8  RED" + StringFormatting.poseToString(redFieldPositions.getTagPose(8).get()));
    System.out.println(
        "Tag 8 BLUE" + StringFormatting.poseToString(blueFieldPositions.getTagPose(8).get()));

    System.out.println(
        "Tag 4  RED" + StringFormatting.poseToString(redFieldPositions.getTagPose(4).get()));
    System.out.println(
        "Tag 4 BLUE" + StringFormatting.poseToString(blueFieldPositions.getTagPose(4).get()));

    Pose2d endPose =
        LocationHelper.getPoseByDistanceAndAngleToPose(
            AllianceFlipUtil.apply(FieldPositions.blueStagingMark2),
            1.2,
            Rotation2d.fromDegrees(-35.0));

    System.out.println(
        StringFormatting.poseToString(AllianceFlipUtil.apply(FieldPositions.blueStagingMark2))
            + " -> "
            + StringFormatting.poseToString(endPose));

    Pose2d endPose2 =
        LocationHelper.transformYAxisForAllianceColor(
            new Pose2d(5.07, .93, Rotation2d.fromDegrees(90.0)));
    Pose2d endPose3 = new Pose2d(5.07, .93, Rotation2d.fromDegrees(90.0));
    System.out.println(" endPose3 = " + StringFormatting.poseToString(endPose3));
    System.out.println(" endPose2 = " + StringFormatting.poseToString(endPose2));
    Pose2d cubeNodePose = new Pose2d(2.3, 4.42, Rotation2d.fromDegrees(180.0));
    System.out.println(
        " cubeNodePose = "
            + StringFormatting.poseToString(cubeNodePose)
            + " -> "
            + StringFormatting.poseToString(
                LocationHelper.transformYAxisForAllianceColor(cubeNodePose)));

    Pose2d mark1attack =
        LocationHelper.getPoseByDistanceAndAngleToPose(
            FieldPositions.blueStagingMark1, 1.2, Rotation2d.fromDegrees(0.0));

    Pose2d mark2attack =
        LocationHelper.getPoseByDistanceAndAngleToPose(
            FieldPositions.blueStagingMark2, 1.2, Rotation2d.fromDegrees(35.0));

    Pose2d mark3attack =
        LocationHelper.getPoseByDistanceAndAngleToPose(
            FieldPositions.blueStagingMark3, 1.6, Rotation2d.fromDegrees(-45.0));
    System.out.println(
        " blue staging mark 1 = "
            + StringFormatting.poseToString(FieldPositions.blueStagingMark1)
            + " -> "
            + StringFormatting.poseToString(mark1attack));

    System.out.println(
        " blue staging mark 2 = "
            + StringFormatting.poseToString(FieldPositions.blueStagingMark2)
            + " -> "
            + StringFormatting.poseToString(mark2attack));

    System.out.println(
        " blue staging mark 3 = "
            + StringFormatting.poseToString(FieldPositions.blueStagingMark3)
            + " -> "
            + StringFormatting.poseToString(mark3attack));
  }
}
