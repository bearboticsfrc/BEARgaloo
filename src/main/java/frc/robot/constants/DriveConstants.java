package frc.robot.constants;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.util.RateLimiter;

public class DriveConstants {
  public static final ShuffleboardTab DRIVE_SUBSYSTEM_TAB = Shuffleboard.getTab("Drive System");

  // Reduction configurations for the SDS MK4i L2 module
  public static final double DRIVE_GEAR_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
  public static final double WHEEL_DIAMETER_METERS = 0.09728;
  public static final double STEER_DRIVE_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);

  public static final double POSITION_CONVERSION_FACTOR = 2.0 * Math.PI * STEER_DRIVE_REDUCTION;
  public static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR / 60;

  /*
   * Max free spin for the NEO motor (taken from docs)
   */
  public static final double MAX_MOTOR_FREE_SPEED_RPM = 5676.0;

  /** Max drive velocity in meters/sec */
  public static final double MAX_VELOCITY =
      MAX_MOTOR_FREE_SPEED_RPM / 60.0 * DRIVE_GEAR_REDUCTION * WHEEL_DIAMETER_METERS * Math.PI;

  /** The max drive angular velocity in radians/sec */
  public static final double MAX_ANGULAR_VELOCITY =
      MAX_VELOCITY / Math.hypot(RobotConstants.TRACK_WIDTH / 2.0, RobotConstants.WHEEL_BASE / 2.0);

  /** The drive motor encoder position conversion factor in meters */
  public static final double ENCODER_POSITION_FACTOR =
      (WHEEL_DIAMETER_METERS * Math.PI) / DRIVE_GEAR_REDUCTION;

  /** The drive motor encoder velocity conversion factor in meters/sec */
  public static final double ENCODER_VELOCITY_FACTOR =
      ((WHEEL_DIAMETER_METERS * Math.PI) / WHEEL_DIAMETER_METERS) / 60.0;

  /** Value in amperage to limit the drive neo motor with <b>setSmartCurrentLimit<b> */
  public static final int DRIVE_CURRENT_LIMIT = 40;

  /** Value in amperage to limit the pivot neo motor with <b>setSmartCurrentLimit<b> */
  public static final int PIVOT_CURRENT_LIMIT = 12;

  /** Voltage compensation on the SPARK MAX via <b>enableVoltageCompensation<b> */
  public static final double NOMINAL_VOLTAGE = 12.0;

  public static final double MAX_ACCELERATION_PER_SECOND = 4;
  public static final double MAX_DECELERATION_PER_SECOND = 4;
  public static final double MAX_ANGULAR_ACCELERATION_PER_SECOND = 10;
  public static final double MAX_ANGULAR_DECELERATION_PER_SECOND = 20;

  public static final RateLimiter X_ACCELERATION_LIMITER =
      new RateLimiter(MAX_ACCELERATION_PER_SECOND, MAX_DECELERATION_PER_SECOND);

  public static final RateLimiter Y_ACCELERATION_LIMITER =
      new RateLimiter(MAX_ACCELERATION_PER_SECOND, MAX_DECELERATION_PER_SECOND);

  public static final RateLimiter TURNING_ACCELERATION_LIMITER =
      new RateLimiter(MAX_ANGULAR_ACCELERATION_PER_SECOND, MAX_ANGULAR_DECELERATION_PER_SECOND);

  public enum SpeedMode {
    TURBO(2.0),
    NORMAL(1.0),
    TURTLE(0.5);

    private final double maxSpeed;

    private SpeedMode(double maxSpeed) {
      this.maxSpeed = maxSpeed;
    }

    public double getMaxSpeed() {
      return maxSpeed;
    }
  }
}
