package frc.robot.constants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  public static final Vector<N3> STATE_STD_DEVS =
      VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(0)); // these are wrong will have to test
  public static final Vector<N3> VISION_STD_DEVS =
      VecBuilder.fill(1.5, 1.5, Units.degreesToRadians(0)); // these are wrong will have to test
  public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
  public static final double FIELD_LENGTH_METERS = 16.54175;
  public static final double FIELD_WIDTH_METERS = 8.0137;
}
