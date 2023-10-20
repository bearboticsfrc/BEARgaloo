package frc.robot.constants;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.util.RateLimiter;

public class DriveConstants {
  public static final ShuffleboardTab DRIVE_SYSTEM_TAB = Shuffleboard.getTab("Drive System");
  public static final ShuffleboardTab COMPETITION_TAB = Shuffleboard.getTab("Competition");

  public static final int DRIVER_CONTROLLER_PORT = 0;
  public static final int OPERATOR_CONTROLLER_PORT = 1;

  // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
  // This changes the drive speed of the module (a pinion gear with more teeth will result in a
  // robot that drives faster).
  public static final int DRIVE_MOTOR_PINION_TEETH = 14;
  public static final double NEO_FREE_SPEED_RPM = 5676;

  /*
   * Max free spin for the NEO motor (taken from docs)
   */
  public static final double MAX_MOTOR_FREE_SPEED_RPM = 5676.0;

  // Calculations required for driving motor conversion factors and feed forward
  public static final double DRIVE_MOTOR_FREE_SPEED_RPS = NEO_FREE_SPEED_RPM / 60;
  public static final double WHEEL_CIRUM = RobotConstants.WHEEL_DIAMETER * Math.PI;
  // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
  // bevel pinion
  public static final double DRIVE_GEAR_REDUCTION = (45.0 * 22) / (DRIVE_MOTOR_PINION_TEETH * 15);
  public static final double DRIVE_WHEEL_FREE_SPEED_RPS =
      (DRIVE_MOTOR_FREE_SPEED_RPS * WHEEL_CIRUM) / DRIVE_GEAR_REDUCTION;

  /** Max drive velocity in meters/sec */
  public static final double MAX_VELOCITY =
      (MAX_MOTOR_FREE_SPEED_RPM / 60.0 * RobotConstants.WHEEL_DIAMETER * Math.PI)
          / DRIVE_GEAR_REDUCTION;

  public static final double DRIVE_VELOCITY = MAX_VELOCITY / 2;

  /** The max drive angular velocity in radians/sec */
  public static final double MAX_ANGULAR_VELOCITY =
      MAX_VELOCITY / Math.hypot(RobotConstants.TRACK_WIDTH / 2.0, RobotConstants.WHEEL_BASE / 2);

  /** The drive motor encoder position conversion factor in meters */
  public static final double ENCODER_POSITION_FACTOR =
      (RobotConstants.WHEEL_DIAMETER * Math.PI) / DRIVE_GEAR_REDUCTION;

  /** The drive motor encoder velocity conversion factor in meters/sec */
  public static final double ENCODER_VELOCITY_FACTOR =
      ((RobotConstants.WHEEL_DIAMETER * Math.PI) / RobotConstants.WHEEL_DIAMETER) / 60.0;

  /** Value in amperage to limit the drive neo motor with <b>setSmartCurrentLimit<b> */
  public static final int DRIVE_CURRENT_LIMIT = 40;

  /** Value in amperage to limit the pivot neo motor with <b>setSmartCurrentLimit<b> */
  public static final int PIVOT_CURRENT_LIMIT = 12;

  /** Voltage compensation on the SPARK MAX via <b>enableVoltageCompensation<b> */
  public static final double NOMINAL_VOLTAGE = 12.0;

  public static final double MAX_ACCELERATION_PER_SECOND = 4;
  public static final double MAX_DECELERATION_PER_SECOND = 4;
  public static final double MAX_ANGULAR_ACCELERATION_PER_SECOND = 6 * (Math.PI * 2);
  public static final double MAX_ANGULAR_DECELERATION_PER_SECOND = 20;

  public static final RateLimiter X_ACCELERATION_LIMITER =
      new RateLimiter(MAX_ACCELERATION_PER_SECOND, MAX_DECELERATION_PER_SECOND);

  public static final RateLimiter Y_ACCELERATION_LIMITER =
      new RateLimiter(MAX_ACCELERATION_PER_SECOND, MAX_DECELERATION_PER_SECOND);

  public static final RateLimiter TURNING_ACCELERATION_LIMITER =
      new RateLimiter(MAX_ANGULAR_ACCELERATION_PER_SECOND, MAX_ANGULAR_DECELERATION_PER_SECOND);

  public enum SpeedMode {
    TURBO(2),
    NORMAL(1),
    TURTLE(0.5);

    private final double maxSpeedMultiplier;

    private SpeedMode(double maxSpeedMultiplier) {
      this.maxSpeedMultiplier = maxSpeedMultiplier;
    }

    public double getMaxSpeedMultiplier() {
      return maxSpeedMultiplier;
    }

    public double getMaxSpeed() {
      return DRIVE_VELOCITY * getMaxSpeedMultiplier();
    }
  }
}
