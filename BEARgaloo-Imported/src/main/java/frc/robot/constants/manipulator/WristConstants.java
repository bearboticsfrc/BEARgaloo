package frc.robot.constants.manipulator;

public class WristConstants {
  public static class Motor {
    public static final String NAME = "Wrist Motor";
    public static final int MOTOR_PORT = 13;
    public static final int CURRENT_LIMT = 80;
    public static final boolean INVERTED = true;
    public static final boolean ENCODER_INVERTED = true;

    public static class MotorPid {
      public static final double P = 0.025;
    }
  }

  public static int WRIST_LIMIT_SWITCH_PORT = 9;

  public enum WristPositions {
    HOME(-2), // TODO: hack, fix it
    TOP(10),
    HIGH(17),
    MIDDLE(24),
    BOTTOM(33.5);

    private final double position;

    private WristPositions(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }
  }
}
