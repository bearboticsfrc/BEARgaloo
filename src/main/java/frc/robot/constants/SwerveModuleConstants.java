package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
  public static class FrontLeftConstants {
    public static final String MODULE_NAME = "FL";
    public static final Rotation2d PARK_ANGLE = Rotation2d.fromDegrees(135);
    public static final Rotation2d CHASSIS_ANGULAR_OFFSET = Rotation2d.fromRadians(-Math.PI / 2);

    public static class DriveMotor {
      public static final String NAME = MODULE_NAME + " Drive";
      public static final int MOTOR_PORT = 3;
      public static final int CURRENT_LIMT = 50;
      public static final boolean INVERTED = false;
      public static final boolean ENCODER_INVERTED = false;

      public static class MotorPid {
        public static final double P = 0.04;
        public static final double Ff = 1 / DriveConstants.DRIVE_WHEEL_FREE_SPEED_RPS;
      }
    }

    public static class PivotMotor {
      public static final String NAME = MODULE_NAME + " Pivot";
      public static final int MOTOR_PORT = 8;
      public static final int CURRENT_LIMT = 20;
      public static final boolean INVERTED = false;
      public static final boolean ENCODER_INVERTED = true;

      public static class MotorPid {
        public static final double P = 1;
        public static final boolean POSITION_PID_WRAPPING_ENABLED = true;
        public static final int POSITION_PID_WRAPPING_MIN = 0;
        public static final double POSITION_PID_WRAPPING_MAX = 2 * Math.PI;
      }
    }
  }

  public static class FrontRightConstants {
    public static final String MODULE_NAME = "FR";
    public static final Rotation2d PARK_ANGLE = Rotation2d.fromDegrees(45);
    public static final Rotation2d CHASSIS_ANGULAR_OFFSET = Rotation2d.fromRadians(0);

    public static class DriveMotor {
      public static final String NAME = MODULE_NAME + " Drive";
      public static final int MOTOR_PORT = 20;
      public static final int CURRENT_LIMT = 50;
      public static final boolean INVERTED = false;
      public static final boolean ENCODER_INVERTED = false;

      public static class MotorPid {
        public static final double P = 0.04;
        public static final double Ff = 1 / DriveConstants.DRIVE_WHEEL_FREE_SPEED_RPS;
      }
    }

    public static class PivotMotor {
      public static final String NAME = MODULE_NAME + " Pivot";
      public static final int MOTOR_PORT = 15;
      public static final int CURRENT_LIMT = 20;
      public static final boolean INVERTED = false;
      public static final boolean ENCODER_INVERTED = true;

      public static class MotorPid {
        public static final double P = 1;
        public static final boolean POSITION_PID_WRAPPING_ENABLED = true;
        public static final int POSITION_PID_WRAPPING_MIN = 0;
        public static final double POSITION_PID_WRAPPING_MAX = 2 * Math.PI;
      }
    }
  }

  public static class BackLeftConstants {
    public static final String MODULE_NAME = "BL";
    public static final Rotation2d PARK_ANGLE = Rotation2d.fromDegrees(225);
    public static final Rotation2d CHASSIS_ANGULAR_OFFSET = Rotation2d.fromRadians(Math.PI);

    public static class DriveMotor {
      public static final String NAME = MODULE_NAME + " Drive";
      public static final int MOTOR_PORT = 2;
      public static final int CURRENT_LIMT = 50;
      public static final boolean INVERTED = false;
      public static final boolean ENCODER_INVERTED = false;

      public static class MotorPid {
        public static final double P = 0.04;
        public static final double Ff = 1 / DriveConstants.DRIVE_WHEEL_FREE_SPEED_RPS;
      }
    }

    public static class PivotMotor {
      public static final String NAME = MODULE_NAME + " Pivot";
      public static final int MOTOR_PORT = 6;
      public static final int CURRENT_LIMT = 20;
      public static final boolean INVERTED = false;
      public static final boolean ENCODER_INVERTED = true;

      public static class MotorPid {
        public static final double P = 1;
        public static final boolean POSITION_PID_WRAPPING_ENABLED = true;
        public static final int POSITION_PID_WRAPPING_MIN = 0;
        public static final double POSITION_PID_WRAPPING_MAX = 2 * Math.PI;
      }
    }
  }

  public static class BackRightConstants {
    public static final String MODULE_NAME = "BR";
    public static final Rotation2d PARK_ANGLE = Rotation2d.fromDegrees(135);
    public static final Rotation2d CHASSIS_ANGULAR_OFFSET = Rotation2d.fromRadians(Math.PI / 2);

    public static class DriveMotor {
      public static final String NAME = MODULE_NAME + " Drive";
      public static final int MOTOR_PORT = 21;
      public static final int CURRENT_LIMT = 50;
      public static final boolean INVERTED = false;
      public static final boolean ENCODER_INVERTED = false;

      public static class MotorPid {
        public static final double P = 0.04;
        public static final double Ff = 1 / DriveConstants.DRIVE_WHEEL_FREE_SPEED_RPS;
      }
    }

    public static class PivotMotor {
      public static final String NAME = MODULE_NAME + " Pivot";
      public static final int MOTOR_PORT = 17;
      public static final int CURRENT_LIMT = 20;
      public static final boolean INVERTED = false;
      public static final boolean ENCODER_INVERTED = true;

      public static class MotorPid {
        public static final double P = 1;
        public static final boolean POSITION_PID_WRAPPING_ENABLED = true;
        public static final int POSITION_PID_WRAPPING_MIN = 0;
        public static final double POSITION_PID_WRAPPING_MAX = 2 * Math.PI;
      }
    }
  }
}
