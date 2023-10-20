package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.manipulator.WristConstants;
import frc.robot.constants.manipulator.WristConstants.WristPositions;
import frc.robot.util.MotorConfig;
import frc.robot.util.MotorConfig.MotorBuilder;

public class WristSubsystem extends SubsystemBase {
  private String name;
  private RelativeEncoder motorEncoder;
  private SparkMaxPIDController motorPid;
  private CANSparkMax motor;
  private double targetPosition = 0.0;
  private final DigitalInput limitSwitch = new DigitalInput(WristConstants.WRIST_LIMIT_SWITCH_PORT);

  public WristSubsystem(MotorBuilder motorConstants) {
    this.name = motorConstants.getName(); // TODO: Use name AND moduleName

    this.motor =
        new CANSparkMax(motorConstants.getMotorPort(), CANSparkMaxLowLevel.MotorType.kBrushless);

    this.motorEncoder = motor.getEncoder();
    this.motorPid = motor.getPIDController();

    MotorConfig.fromMotorConstants(motor, motorEncoder, motorConstants)
        .configureMotor()
        .configurePID(motorConstants.getMotorPID())
        .burnFlash();

    setupShuffleboardTab(RobotConstants.MANIPULATOR_SYSTEM_TAB);
    // setupDataLogging(DataLogManager.getLog()); TODO: impl
  }

  @Override
  public void periodic() {
    if (isHome() && motorEncoder.getPosition() != 0) {
      motorEncoder.setPosition(0);
    }
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

  public void set(WristPositions position) {
    targetPosition = position.getPosition();
    motorPid.setReference(position.getPosition(), ControlType.kPosition);
  }

  public void set(double position) {
    if (position < WristPositions.HOME.getPosition()
        || position > WristPositions.BOTTOM.getPosition()) {
      return;
    }

    targetPosition = position;
    motorPid.setReference(position, ControlType.kPosition);
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
}
