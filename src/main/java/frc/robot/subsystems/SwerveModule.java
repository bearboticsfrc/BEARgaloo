package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.util.MotorConfig;
import frc.robot.util.MotorConfig.MotorBuilder;
import java.util.HashMap;

/** A SwerveModules consists of a drive motor and a steer motor */
public class SwerveModule {
  private String moduleName;

  private CANSparkMax driveMotor;
  private CANSparkMax pivotMotor;

  private RelativeEncoder driveMotorEncoder;
  private AbsoluteEncoder pivotMotorEncoder;

  private SparkMaxPIDController driveMotorPIDController;
  private SparkMaxPIDController pivotMotorPIDController;

  private Rotation2d referenceAngle = new Rotation2d();
  private Rotation2d parkedAngle;
  private Rotation2d chassisAngularOffset;

  private boolean parked = false;

  private final boolean SHUFFLEBOARD_ENABLED = false;

  private HashMap<String, DoubleLogEntry> dataLogs = new HashMap<String, DoubleLogEntry>();

  public SwerveModule(SwerveModuleBuilder swerveModule, ShuffleboardTab shuffleboardTab) {
    this.moduleName = swerveModule.getModuleName();
    this.parkedAngle = swerveModule.getParkAngle();
    this.chassisAngularOffset = swerveModule.getChassisAngularOffset();

    this.driveMotor =
        new CANSparkMax(
            swerveModule.getDriveMotor().getMotorPort(), CANSparkMaxLowLevel.MotorType.kBrushless);

    this.pivotMotor =
        new CANSparkMax(
            swerveModule.getPivotMotor().getMotorPort(), CANSparkMaxLowLevel.MotorType.kBrushless);

    this.driveMotorEncoder = driveMotor.getEncoder();
    this.pivotMotorEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);

    MotorConfig.fromMotorConstants(driveMotor, driveMotorEncoder, swerveModule.getDriveMotor())
        .configureMotor()
        .configurePID(swerveModule.getDriveMotor().getMotorPID())
        .burnFlash();

    MotorConfig.fromMotorConstants(pivotMotor, pivotMotorEncoder, swerveModule.getPivotMotor())
        .configureMotor()
        .configurePID(swerveModule.getPivotMotor().getMotorPID())
        .burnFlash();

    this.driveMotorPIDController = driveMotor.getPIDController();
    this.pivotMotorPIDController = pivotMotor.getPIDController();

    setupShuffleboardTab(shuffleboardTab);
    setupDataLogging(DataLogManager.getLog());
  }

  /**
   * @param shuffleboardTab The shuffleboard tab to use
   */
  private void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    if (SHUFFLEBOARD_ENABLED) {
      shuffleboardTab
          .addNumber(String.format("%s Vel", moduleName), this::getDriveVelocity)
          .withSize(1, 1);
      shuffleboardTab
          .addNumber(String.format("%s Drive Out", moduleName), driveMotor::getAppliedOutput)
          .withSize(1, 1);
      shuffleboardTab
          .addNumber(String.format("%s Pos", moduleName), this::getDistance)
          .withSize(1, 1);
      shuffleboardTab
          .addNumber(String.format("%s Steer Deg", moduleName), () -> getSteerAngle().getDegrees())
          .withSize(1, 1);
      shuffleboardTab
          .addNumber(String.format("%s AE Deg", moduleName), () -> getAbsoluteAngle().getDegrees())
          .withSize(1, 1);
      shuffleboardTab
          .addNumber(String.format("%s Ref Deg", moduleName), () -> referenceAngle.getDegrees())
          .withSize(1, 1);
    }
  }

  private void setupDataLogging(DataLog log) {
    for (String motorType : new String[] {"DRIVE", "PIVOT"}) {
      if (motorType.equals("PIVOT")) {
        dataLogs.put(
            String.format("%s_MOTOR_POSITION", motorType),
            new DoubleLogEntry(
                log,
                String.format("/drive/%s/%s_motor/current", moduleName, motorType.toLowerCase())));
      }

      String pathMotorType = motorType.toLowerCase();

      dataLogs.put(
          String.format("%s_MOTOR_CURRENT", motorType),
          new DoubleLogEntry(
              log, String.format("/drive/%s/%s_motor/current", moduleName, pathMotorType)));

      dataLogs.put(
          String.format("%s_MOTOR_VELOCITY", motorType),
          new DoubleLogEntry(
              log, String.format("/drive/%s/%s_motor/current", moduleName, pathMotorType)));
      dataLogs.put(
          String.format("%s_MOTOR_APPLIED_OUTPUT", motorType),
          new DoubleLogEntry(
              log, String.format("/drive/%s/%s_motor/current", moduleName, pathMotorType)));
      dataLogs.put(
          String.format("%s_MOTOR_TEMPERATURE", motorType),
          new DoubleLogEntry(
              log, String.format("/drive/%s/%s_motor/current", moduleName, pathMotorType)));
      dataLogs.put(
          String.format("%s_MOTOR_CURRENT", motorType),
          new DoubleLogEntry(
              log, String.format("/drive/%s/%s_motor/current", moduleName, pathMotorType)));
      dataLogs.put(
          String.format("%s_MOTOR_POSITION", motorType),
          new DoubleLogEntry(
              log, String.format("/drive/%s/%s_motor/current", moduleName, pathMotorType)));
      dataLogs.put(
          String.format("%s_MOTOR_VELOCITY", motorType),
          new DoubleLogEntry(
              log, String.format("/drive/%s/%s_motor/current", moduleName, pathMotorType)));
      dataLogs.put(
          String.format("%s_MOTOR_APPLIED_OUTPUT", motorType),
          new DoubleLogEntry(
              log, String.format("/drive/%s/%s_motor/current", moduleName, pathMotorType)));
      dataLogs.put(
          String.format("%s_MOTOR_TEMPERATURE", motorType),
          new DoubleLogEntry(
              log, String.format("/drive/%s/%s_motor/current", moduleName, pathMotorType)));
    }
  }

  public void updateDataLogs() {
    CANSparkMax[] motors = new CANSparkMax[] {driveMotor, pivotMotor};

    for (int iteration = 0; iteration < motors.length; iteration++) {
      String motorType = iteration == 0 ? "DRIVE" : "PIVOT";
      MotorFeedbackSensor motorEncoder = iteration == 0 ? driveMotorEncoder : pivotMotorEncoder;
      CANSparkMax motor = motors[iteration];

      if (motorType == "PIVOT") {
        dataLogs
            .get(motorType + "_MOTOR_POSITION")
            .append(((AbsoluteEncoder) motorEncoder).getPosition());
      }

      dataLogs.get(motorType + "_MOTOR_CURRENT").append(motor.getOutputCurrent());
      dataLogs.get(motorType + "_MOTOR_VELOCITY").append(motor.getEncoder().getVelocity());
      dataLogs.get(motorType + "_MOTOR_APPLIED_OUTPUT").append(motor.getAppliedOutput());
      dataLogs.get(motorType + "_MOTOR_TEMPERATURE").append(motor.getMotorTemperature());
    }
  }

  public Rotation2d getParkedAngle() {
    return parkedAngle;
  }

  /**
   * A Rotation2d representation of the angle of the steer absolute encoder
   *
   * @return The angle
   */
  public Rotation2d getAbsoluteAngle() {
    return Rotation2d.fromRadians(pivotMotorEncoder.getPosition());
  }

  /**
   * Returns the current steer angle
   *
   * @return Rotation2d of the angle
   */
  public Rotation2d getSteerAngle() {
    return Rotation2d.fromRadians(pivotMotorEncoder.getPosition());
  }

  /**
   * Returns the current drive veloticty
   *
   * @return The veloticty
   */
  public double getDriveVelocity() {
    return driveMotorEncoder.getVelocity();
  }

  /**
   * Returns a Rotation2d representation of the position of the drive encoder
   *
   * @return The position
   */
  public double getDistance() {
    return driveMotorEncoder.getPosition();
  }

  /**
   * Returns the position of the swerve module
   *
   * @return The position
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveMotorEncoder.getPosition(),
        new Rotation2d(pivotMotorEncoder.getPosition() - chassisAngularOffset.getRadians()));
  }

  public void setParked(boolean mode) {
    parked = mode;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), getSteerAngle());
  }

  /**
   * Sets the steer angle in radians
   *
   * @param state The state of the swerve module
   */
  public void set(SwerveModuleState state) {
    if (parked) {
      return;
    }

    SwerveModuleState desiredState =
        new SwerveModuleState(state.speedMetersPerSecond, state.angle.plus(chassisAngularOffset));
    desiredState = SwerveModuleState.optimize(desiredState, getSteerAngle());

    pivotMotorPIDController.setReference(desiredState.angle.getRadians(), ControlType.kPosition);
    driveMotorPIDController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);

    referenceAngle = desiredState.angle;
  }

  public static class SwerveModuleBuilder {
    private String moduleName;
    private Rotation2d parkAngle;
    private Rotation2d chassisAngularOffset;
    private MotorBuilder driveMotor;
    private MotorBuilder pivotMotor;

    public String getModuleName() {
      return moduleName;
    }

    public SwerveModuleBuilder setModuleName(String moduleName) {
      this.moduleName = moduleName;
      return this;
    }

    public Rotation2d getParkAngle() {
      return parkAngle;
    }

    public SwerveModuleBuilder setParkAngle(Rotation2d parkAngle) {
      this.parkAngle = parkAngle;
      return this;
    }

    public MotorBuilder getDriveMotor() {
      return driveMotor;
    }

    public SwerveModuleBuilder setDriveMotor(MotorBuilder driveMotor) {
      this.driveMotor = driveMotor;
      return this;
    }

    public MotorBuilder getPivotMotor() {
      return pivotMotor;
    }

    public SwerveModuleBuilder setPivotMotor(MotorBuilder pivotMotor) {
      this.pivotMotor = pivotMotor;
      return this;
    }

    public Rotation2d getChassisAngularOffset() {
      return chassisAngularOffset;
    }

    public SwerveModuleBuilder setChassisAngularOffset(Rotation2d chassisAngularOffset) {
      this.chassisAngularOffset = chassisAngularOffset;
      return this;
    }
  }
}
