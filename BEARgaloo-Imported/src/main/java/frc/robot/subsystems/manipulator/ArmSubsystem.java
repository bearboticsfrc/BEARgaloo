package frc.robot.subsystems.manipulator;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import bearlib.motor.deserializer.MotorParser;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.manipulator.ArmConstants.ArmPositions;
import java.util.HashMap;
import java.util.Map.Entry;
import java.util.function.DoubleSupplier;
import java.io.IOException;
import java.io.File;

public class ArmSubsystem extends SubsystemBase {
  private String name;
  private RelativeEncoder motorEncoder;
  private SparkClosedLoopController motorPid;
  private SparkBase motor;
  private SparkBase followerMotor;

  private HashMap<String, DoubleLogEntry> dataLogs = new HashMap<String, DoubleLogEntry>();

  public ArmSubsystem() {

      File directory = new File(Filesystem.getDeployDirectory(), "motors/elevator");
  
      try {
        motor =
            new MotorParser(directory)
                .withMotor("leader.json")
                .withPidf("raisepidf.json",0)
                .withPidf("lowerpidf.json", 1)
                .configureAsync();
  
          followerMotor =
                new MotorParser(directory)
                    .withMotor("follower.json")
                    .withPidf("followerpidf.json")
                    .configureAsync();
  
        motorEncoder = motor.getEncoder();
        motorPid = motor.getClosedLoopController();
      } catch (IOException exception) {
        throw new RuntimeException("Failed to configure arm motor(s): ", exception);
      }



    setupShuffleboardTab(RobotConstants.MANIPULATOR_SYSTEM_TAB);
    setupDataLogging(DataLogManager.getLog());
  }

  /**
   * @param shuffleboardTab The shuffleboard tab to use
   */
  private void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    shuffleboardTab
        .addNumber(String.format("%s Pos", name), this.motorEncoder::getPosition)
        .withSize(1, 1);

    shuffleboardTab
        .addNumber(String.format("%s M Amps", name), this.motor::getAppliedOutput)
        .withSize(1, 1);
    shuffleboardTab
        .addNumber(String.format("%s F Amps", name), this.followerMotor::getAppliedOutput)
        .withSize(1, 1);
  }

  /**
   * Initalize data logging.
   *
   * @param log The log to use
   */
  private void setupDataLogging(DataLog log) {
    final String LOG_PATH_BASE = "/arm/motor/%s";
    final String[] LOGS =
        new String[] {"POSITION", "CURRENT", "VELOCITY", "APPLIED_OUTPUT", "TEMPERATURE"};

    for (String logName : LOGS) {
      dataLogs.put(logName, new DoubleLogEntry(log, LOG_PATH_BASE.formatted(logName)));
    }
  }

  public void set(ArmPositions position) {
    ClosedLoopSlot slot = position == ArmPositions.HIGH ? ClosedLoopSlot.kSlot0 : ClosedLoopSlot.kSlot1;
    set(position, slot);
  }

  public void set(ArmPositions position, ClosedLoopSlot slot) {
    motorPid.setReference(position.getPosition(), ControlType.kPosition, slot);
  }

  public boolean isHome() {
    return motorEncoder.getPosition() < .3;
  }

  @Override
  public void periodic() {
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
