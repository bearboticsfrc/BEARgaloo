package frc.robot.subsystems.manipulator;

import com.revrobotics.spark.SparkBase;
import bearlib.motor.deserializer.MotorParser;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.manipulator.WristConstants;
import frc.robot.constants.manipulator.WristConstants.WristPositions;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map.Entry;
import java.util.function.DoubleSupplier;

public class WristSubsystem extends SubsystemBase {
  private String name;
  private RelativeEncoder motorEncoder;
  private SparkClosedLoopController motorPid;
  private SparkBase motor;

  private double targetPosition = 0;
  private final DigitalInput limitSwitch = new DigitalInput(WristConstants.WRIST_LIMIT_SWITCH_PORT);

  private HashMap<String, DoubleLogEntry> dataLogs = new HashMap<String, DoubleLogEntry>();

  public WristSubsystem() {

      File directory = new File(Filesystem.getDeployDirectory(), "motors/Manipulator/Wrist");
  
      try {
        motor =
            new MotorParser(directory)
                .withMotor("motor.json")
                .withPidf("pidf.json")
                .configureAsync();
  
        motorPid = motor.getClosedLoopController();
        motorEncoder = motor.getEncoder();
      } catch (IOException exception) {
        throw new RuntimeException("Failed to configure wrist motor(s): ", exception);
      }
    

    setupShuffleboardTab(RobotConstants.MANIPULATOR_SYSTEM_TAB);
    setupDataLogging(DataLogManager.getLog());
  }

  /**
   * @param shuffleboardTab The shuffleboard tab to use
   */
  private void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    shuffleboardTab.addBoolean(String.format("%s Home?", name), this::isHome).withSize(1, 1);
    shuffleboardTab
        .addNumber(String.format("%s Pos", name), this.motorEncoder::getPosition)
        .withSize(1, 1);
    shuffleboardTab
        .addNumber(String.format("%s Amps", name), this.motor::getOutputCurrent)
        .withSize(1, 1);
    shuffleboardTab
        .addNumber(String.format("%s Setpoint", name), this::getTargetPosition)
        .withSize(1, 1);
    shuffleboardTab
        .addNumber(String.format("%s Temp", name), this.motor::getMotorTemperature)
        .withSize(1, 1);
  }

  /**
   * Setup the data logging for this manipulator
   *
   * @param log Which log to append to
   */
  private void setupDataLogging(DataLog log) { // TODO: Abstract into own lib
    final String LOG_PATH_BASE = "/wrist/motor/%s"; // "/wrist/%s"
    final String[] LOGS =
        new String[] {"POSITION", "CURRENT", "VELOCITY", "APPLIED_OUTPUT", "TEMPERATURE"};

    for (String logName : LOGS) {
      dataLogs.put(logName, new DoubleLogEntry(log, LOG_PATH_BASE.formatted(logName)));
    }
  }

  public void set(WristPositions position) {
    targetPosition = position.getPosition();
    motorPid.setReference(position.getPosition(), ControlType.kPosition);
  }

  public void setReference(double position) {
    if (position < WristPositions.HOME.getPosition()
        || position > WristPositions.BOTTOM.getPosition()) {
      return;
    }

    targetPosition = position;
    motorPid.setReference(position, ControlType.kPosition);
  }

  /**
   * Applies a speed to the wrist motor.
   *
   * @param speed The speed to apply to the wrist motor.
   */
  public void set(double speed) {
    motor.set(speed);
  }

  public void calibrate() {
    motorEncoder.setPosition(0);
  }

  public double getTargetPosition() {
    return targetPosition;
  }

  public boolean isHome() {
    return limitSwitch.get();
  }

  @Override
  public void periodic() {
    if (isHome() && motorEncoder.getPosition() != 0) {
      motorEncoder.setPosition(0);
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
