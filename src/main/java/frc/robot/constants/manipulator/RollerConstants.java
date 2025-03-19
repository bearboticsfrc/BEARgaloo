package frc.robot.constants.manipulator;

public class RollerConstants {
  public static class Motor {
    public static final String NAME = "Roller Motor";
    public static final int MOTOR_PORT = 10;
    public static final int CURRENT_LIMT = 60;
    public static final boolean INVERTED = false;
    public static final boolean ENCODER_INVERTED = true;
  }

  public enum RollerSpeed {
    INTAKE(-0.5),
    MEDIUM(0.5),
    OFF(0),
    RELEASE(0.7);

    private final double speed;

    private RollerSpeed(double speed) {
      this.speed = speed;
    }

    public double getSpeed() {
      return speed;
    }
  }
}
