package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.manipulator.RollerConstants.RollerSpeed;
import frc.robot.util.MotorConfig;
import frc.robot.util.MotorConfig.MotorBuilder;
import java.util.HashMap;
import java.util.Map.Entry;
import java.util.function.DoubleSupplier;

public class RollerSubsystem extends SubsystemBase {
  private MedianFilter medianFilter = new MedianFilter(10);
  private String name;
  private RelativeEncoder motorEncoder;
  private CANSparkMax motor;
  private boolean isHoldingCube = false;

  private HashMap<String, DoubleLogEntry> dataLogs = new HashMap<String, DoubleLogEntry>();

  public RollerSubsystem(MotorBuilder motorConstants) {
    this.name = motorConstants.getName();

    this.motor =
        new CANSparkMax(motorConstants.getMotorPort(), CANSparkMaxLowLevel.MotorType.kBrushless);

    this.motorEncoder = motor.getEncoder();
    MotorConfig.fromMotorConstants(motor, motorEncoder, motorConstants)
        .configureMotor()
        .burnFlash();

    setupShuffleboardTab(RobotConstants.MANIPULATOR_SYSTEM_TAB);
    setupDataLogging(DataLogManager.getLog());
  }

  /**
   * @param shuffleboardTab The shuffleboard tab to use
   */
  private void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    shuffleboardTab
        .addNumber(String.format("%s Pos", name), this.motorEncoder::getPosition)
        .withSize(2, 1);
    shuffleboardTab
        .addNumber(String.format("%s Amps", name), this.motor::getOutputCurrent)
        .withSize(2, 1);
    shuffleboardTab
        .addNumber(String.format("%s Output", name), this.motor::getAppliedOutput)
        .withSize(2, 1);
    shuffleboardTab.addBoolean(String.format("%s Cube?", name), this::hasCube).withSize(1, 1);
    shuffleboardTab
        .addNumber(String.format("%s temp", name), this.motor::getMotorTemperature)
        .withSize(2, 1);
  }

  /**
   * Initalize data logging.
   *
   * @param log The log to use
   */
  private void setupDataLogging(DataLog log) {
    final String LOG_PATH_BASE = "/roll/motor/%s"; // "/roller/%s"
    final String[] LOGS =
        new String[] {"POSITION", "CURRENT", "VELOCITY", "APPLIED_OUTPUT", "TEMPERATURE"};

    for (String logName : LOGS) {
      dataLogs.put(logName, new DoubleLogEntry(log, LOG_PATH_BASE.formatted(logName)));
    }
  }

  public boolean hasCube() {
    double adjustedOutput = medianFilter.calculate(motor.getOutputCurrent());
    return motorEncoder.getVelocity() < 0 && (adjustedOutput > 40);
  }

  public boolean isHoldingCube() {
    return isHoldingCube;
  }
  /**
   * Sets the roller speed.
   *
   * @param speed An enum representing the set speed.
   */
  public void set(RollerSpeed speed) {
    motor.set(speed.getSpeed());
  }

  @Override
  public void periodic() {
    if (motor.get() < 0 && hasCube()) {
      motor.set(0);
      isHoldingCube = true;
    }

    if (motor.get() > 0) {
      isHoldingCube = false;
    }

    updateDataLogs();
  }

  /** Updates the data logs */
  public void updateDataLogs() {
    for (Entry<String, DoubleLogEntry> entry : dataLogs.entrySet()) {
      DoubleSupplier propertySupplier = getPropertySupplier(entry.getKey());
      entry.getValue().append(propertySupplier.getAsDouble());
    }
  }

  /**
   * Returns the respective getter for <b>property</b>
   *
   * @param property The property
   * @return The getter, wrapped as a DoubleSupplier
   */
  public DoubleSupplier getPropertySupplier(String property) {
    switch (property) {
      case "POSITION":
        return motor.getEncoder()::getPosition;
      case "CURRENT":
        return motor::getOutputCurrent;
      case "VELOCITY":
        return motor.getEncoder()::getVelocity;
      case "APPLIED_OUTPUT":
        return motor::getAppliedOutput;
      case "TEMPERATURE":
        return motor::getMotorTemperature;
      default:
        throw new IllegalArgumentException("Unknown motor property: " + property);
    }
  }
}
