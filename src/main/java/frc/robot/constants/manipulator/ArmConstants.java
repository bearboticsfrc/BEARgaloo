package frc.robot.constants.manipulator;

public class ArmConstants {
  public static class Motor {
    public static final String NAME = "Arm Motor";
    public static final int MOTOR_PORT = 12;
    public static final int CURRENT_LIMT = 30;
    public static final boolean INVERTED = false;
    public static final boolean ENCODER_INVERTED = true;

    public static class MotorPid {
      public static final double P = 0.001;
    }
  }

  public static class FollowerMotor {
    public static final String NAME = "Arm Follower Motor";
    public static final int MOTOR_PORT = 11;
    public static final int CURRENT_LIMT = 30;
    public static final boolean INVERTED = true;
    public static final boolean ENCODER_INVERTED = true;

    public static class MotorPid {
      public static final double P = 0.001;
    }
  }

  public enum ArmPositions {
    TOP(-1),
    BOTTOM(-1);

    private final double position;

    private ArmPositions(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }
  }
}
