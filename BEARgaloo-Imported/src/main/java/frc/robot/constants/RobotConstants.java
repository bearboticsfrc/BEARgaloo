package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class RobotConstants {
  public static final double TRACK_WIDTH = 0.521;
  public static final double WHEEL_BASE = 0.521;
  public static final double WHEEL_DIAMETER = 0.07691;
  public static final int PIGEON_CAN_ID = 24;

  public static final ShuffleboardTab MANIPULATOR_SYSTEM_TAB =
      Shuffleboard.getTab("Manipulator System");

  // Order here must line-up with SwerveCorner order!!
  public static final SwerveDriveKinematics DRIVE_KINEMATICS =
      new SwerveDriveKinematics(
          new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // FL
          new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // FR
          new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // BL
          new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)); // BR

  public static double ROBOT_RADIUS = new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2).getNorm();

  public enum SwerveCorner {
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT,
  }
}
