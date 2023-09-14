package frc.robot.util;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import frc.robot.constants.DriveConstants;

public class MotorConfig {
  private CANSparkMax motor;
  private MotorFeedbackSensor motorEncoder;
  private String moduleName;
  private String name;
  private boolean motorInverted;
  private boolean encoderInverted;
  private double nominalVoltage;
  private int currentLimit;

  /**
   * @param motor Specific motor to configure
   * @param motorEncoder Motor encoder for this motor
   * @param inverted Whether this motor should be inverted or not
   * @param nominalVoltage The nominal voltage compansation for this motor
   * @param currentLimit The current limit for this motor
   */
  public MotorConfig(
      CANSparkMax motor,
      MotorFeedbackSensor motorEncoder,
      String moduleName,
      String name,
      boolean motorInverted,
      boolean encoderInverted,
      double nominalVoltage,
      int currentLimit) {
    this.motor = motor;
    this.motorEncoder = motorEncoder;
    this.moduleName = moduleName;
    this.name = name;
    this.motorInverted = motorInverted;
    this.encoderInverted = encoderInverted;
    this.nominalVoltage = nominalVoltage;
    this.currentLimit = currentLimit;
  }

  /*
   * Generic configuration method
   */
  public MotorConfig configureMotor() {
    motor.setInverted(motorInverted);

    RevUtil.checkRevError(motor.setIdleMode(CANSparkMax.IdleMode.kBrake));
    RevUtil.checkRevError(motor.enableVoltageCompensation(nominalVoltage));
    RevUtil.checkRevError(motor.setSmartCurrentLimit(currentLimit));
    RevUtil.setPeriodicFramePeriodHigh(motor, String.format("%s - %s", moduleName, name));

    if (motorEncoder instanceof RelativeEncoder) {
      RelativeEncoder encoder = (RelativeEncoder) motorEncoder;
      RevUtil.checkRevError(motor.getPIDController().setFeedbackDevice(encoder));

      RevUtil.checkRevError(
          encoder.setPositionConversionFactor(DriveConstants.DRIVING_POSITION_CONVERSION_FACTOR));
      RevUtil.checkRevError(
          encoder.setVelocityConversionFactor(DriveConstants.DRIVING_VELOCITY_CONVERSION_FACTOR));

      ((RelativeEncoder) motorEncoder).setPosition(0);
    } else if (motorEncoder instanceof AbsoluteEncoder) {
      // Since we use AbsoluteEncoder with our
      // pivot motor it's safe to assume here.
      // Won't be the case for 100% of builds although.
      AbsoluteEncoder encoder = (AbsoluteEncoder) motorEncoder; // silly cast for silly java

      RevUtil.checkRevError(encoder.setInverted(encoderInverted));
      RevUtil.checkRevError(motor.getPIDController().setFeedbackDevice(encoder));

      RevUtil.checkRevError(
          encoder.setPositionConversionFactor(DriveConstants.PIVOT_POSITION_CONVERSION_FACTOR));
      RevUtil.checkRevError(
          encoder.setVelocityConversionFactor(DriveConstants.PIVOT_VELOCITY_CONVERSION_FACTOR));
    }

    return this;
  }

  public MotorConfig configurePID(MotorPIDBuilder motorPid) {
    SparkMaxPIDController motorPIDController = motor.getPIDController();
    RevUtil.checkRevError(motorPIDController.setP(motorPid.getP()));
    RevUtil.checkRevError(motorPIDController.setI(motorPid.getI()));
    RevUtil.checkRevError(motorPIDController.setD(motorPid.getD()));
    RevUtil.checkRevError(motorPIDController.setFF(motorPid.getFf()));
    RevUtil.checkRevError(
        motorPIDController.setOutputRange(motorPid.getMinOutput(), motorPid.getMaxOutput()));

    boolean positionPidWrappingEnabled = motorPid.isPositionPidWrappingEnabled();

    if (positionPidWrappingEnabled) {
      RevUtil.checkRevError(
          motorPIDController.setPositionPIDWrappingEnabled(positionPidWrappingEnabled));
      RevUtil.checkRevError(
          motorPIDController.setPositionPIDWrappingMinInput(motorPid.getPositionPidWrappingMin()));
      RevUtil.checkRevError(
          motorPIDController.setPositionPIDWrappingMaxInput(motorPid.getPositionPidWrappingMax()));
    }

    return this;
  }

  public void burnFlash() {
    RevUtil.checkRevError(motor.burnFlash());

    // Burn settings onto motor flash
    // might not work, needs a delay after setting values
  }

  public static MotorConfig fromMotorConstants(
      CANSparkMax motor, MotorFeedbackSensor motorEncoder, MotorBuilder constants) {
    return new MotorConfig(
        motor,
        motorEncoder,
        constants.getModuleName(),
        constants.getName(),
        constants.isMotorInverted(),
        constants.isEncoderInverted(),
        constants.getNominalVoltage(),
        constants.getCurrentLimit());
  }

  public static class MotorPIDBuilder {
    private double p = 0;
    private double i = 0;
    private double d = 0;
    private double ff = 0;
    private double minOutput = -1;
    private double maxOutput = 1;
    private boolean positionPidWrappingEnabled = false;
    private double positionPidWrappingMin = -1;
    private double positionPidWrappingMax = 1;

    public double getP() {
      return p;
    }

    public MotorPIDBuilder setP(double p) {
      this.p = p;
      return this;
    }

    public double getI() {
      return i;
    }

    public MotorPIDBuilder setI(double i) {
      this.i = i;
      return this;
    }

    public double getD() {
      return d;
    }

    public MotorPIDBuilder setD(double d) {
      this.d = d;
      return this;
    }

    public double getFf() {
      return ff;
    }

    public MotorPIDBuilder setFf(double ff) {
      this.ff = ff;
      return this;
    }

    public double getMinOutput() {
      return minOutput;
    }

    public MotorPIDBuilder setMinOutput(double minOutput) {
      this.minOutput = minOutput;
      return this;
    }

    public double getMaxOutput() {
      return maxOutput;
    }

    public MotorPIDBuilder setMaxOutput(double maxOutput) {
      this.maxOutput = maxOutput;
      return this;
    }

    public boolean isPositionPidWrappingEnabled() {
      return positionPidWrappingEnabled;
    }

    public MotorPIDBuilder setPositionPidWrappingEnabled(boolean enabled) {
      this.positionPidWrappingEnabled = enabled;
      return this;
    }

    public double getPositionPidWrappingMin() {
      return positionPidWrappingMin;
    }

    public MotorPIDBuilder setPositionPidWrappingMin(double positionPidWrappingMin) {
      this.positionPidWrappingMin = positionPidWrappingMin;
      return this;
    }

    public double getPositionPidWrappingMax() {
      return positionPidWrappingMax;
    }

    public MotorPIDBuilder setPositionPidWrappingMax(double positionPidWrappingMax) {
      this.positionPidWrappingMax = positionPidWrappingMax;
      return this;
    }
  }

  public static class MotorBuilder {
    private String name;
    private String moduleName;
    private int motorPort;
    private boolean motorInverted;
    private boolean encoderInverted;
    private MotorPIDBuilder motorPID;
    private double nominalVoltage = 12;
    private int currentLimit;

    public String getName() {
      return name;
    }

    public MotorBuilder setName(String name) {
      this.name = name;
      return this;
    }

    public int getMotorPort() {
      return motorPort;
    }

    public MotorBuilder setMotorPort(int motorPort) {
      this.motorPort = motorPort;
      return this;
    }

    public MotorPIDBuilder getMotorPID() {
      return motorPID;
    }

    public MotorBuilder setMotorPID(MotorPIDBuilder motorPID) {
      this.motorPID = motorPID;
      return this;
    }

    public String getModuleName() {
      return moduleName;
    }

    public MotorBuilder setModuleName(String moduleName) {
      this.moduleName = moduleName;
      return this;
    }

    public double getNominalVoltage() {
      return nominalVoltage;
    }

    public MotorBuilder setNominalVoltage(double nominalVoltage) {
      this.nominalVoltage = nominalVoltage;
      return this;
    }

    public int getCurrentLimit() {
      return currentLimit;
    }

    public MotorBuilder setCurrentLimit(int currentLimit) {
      this.currentLimit = currentLimit;
      return this;
    }

    /**
     * @return the motorInverted
     */
    public boolean isMotorInverted() {
      return motorInverted;
    }

    /**
     * @param motorInverted the motorInverted to set
     */
    public MotorBuilder setMotorInverted(boolean motorInverted) {
      this.motorInverted = motorInverted;
      return this;
    }

    /**
     * @return the encoderInverted
     */
    public boolean isEncoderInverted() {
      return encoderInverted;
    }

    /**
     * @param encoderInverted the encoderInverted to set
     */
    public MotorBuilder setEncoderInverted(boolean encoderInverted) {
      this.encoderInverted = encoderInverted;
      return this;
    }
  }
}
