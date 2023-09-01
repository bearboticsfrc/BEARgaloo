package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class RobotConstants {
  public static final double TRACK_WIDTH = 0.521;
  public static final double WHEEL_BASE = 0.521;
  public static final int PIGEON_CAN_ID = 0;

  public static final SwerveDriveKinematics DRIVE_KINEMATICS =
      new SwerveDriveKinematics(
          new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // front left
          new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // front right
          new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // back left
          new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)); // back right
}
