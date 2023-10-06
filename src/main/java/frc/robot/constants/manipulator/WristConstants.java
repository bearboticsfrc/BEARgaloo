package frc.robot.constants.manipulator;

public class WristConstants {
  public static class Motor {
    public static final String MODULE_NAME = "Wrist Motor";
    public static final int MOTOR_PORT = 13;
    public static final int CURRENT_LIMT = 20;
    public static final boolean INVERTED = true;
    public static final boolean ENCODER_INVERTED = true;

    public static class MotorPid {
      public static final double P = 0.001;
    }
  }

  public enum WristPositions {
    HOME(0),
    TOP(-1),
    MIDDLE(-1),
    BOTTOM(1.7);

    private final double position;

    private WristPositions(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }
  }
}
