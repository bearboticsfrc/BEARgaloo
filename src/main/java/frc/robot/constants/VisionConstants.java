package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.pose.vision.RobotCamera;

public class VisionConstants {
  public static final Vector<N3> STATE_STD_DEVS =
      VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(0)); // these are wrong will have to test
  public static final Vector<N3> VISION_STD_DEVS =
      VecBuilder.fill(1.5, 1.5, Units.degreesToRadians(0)); // these are wrong will have to test

  public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
  public static final double FIELD_LENGTH_METERS = 16.54175;
  public static final double FIELD_WIDTH_METERS = 8.0137;
  public static final RobotCamera[] robotCameras = {};

  public static final Matrix<N3, N1> VISION_MEASUREMENT_STD_DEVS =
      Matrix.mat(Nat.N3(), Nat.N1()).fill(1, 1, 1 * Math.PI);

  public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
  public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
  public static final double TAG_PRESENCE_WEIGHT = 10;
  public static final double NOISY_DISTANCE_METERS = 2.5;
  public static final double DISTANCE_WEIGHT = 7;

  public static final double HALF_ROBOT_LENGTH = .44;
}
