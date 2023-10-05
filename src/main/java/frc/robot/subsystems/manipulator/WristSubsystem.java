package frc.robot.subsystems.manipulator;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.manipulator.WristConstants.WristPositions;
import frc.robot.util.MotorConfig;
import frc.robot.util.MotorConfig.MotorBuilder;

public class WristSubsystem extends SubsystemBase {
  private String moduleName;
  private AbsoluteEncoder motorEncoder;
  private SparkMaxPIDController motorPid;
  private CANSparkMax motor;

  public WristSubsystem(MotorBuilder motorConstants) {
    this.moduleName = motorConstants.getModuleName();

    this.motor =
        new CANSparkMax(motorConstants.getMotorPort(), CANSparkMaxLowLevel.MotorType.kBrushless);

    this.motorEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    this.motorPid = motor.getPIDController();

    MotorConfig.fromMotorConstants(motor, motorEncoder, motorConstants)
        .configureMotor()
        .configurePID(motorConstants.getMotorPID())
        .burnFlash();

    setupShuffleboardTab(RobotConstants.MANIPULATOR_SYSTEM_TAB);
    // setupDataLogging(DataLogManager.getLog()); TODO: impl
  }

  /**
   * @param shuffleboardTab The shuffleboard tab to use
   */
  private void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    shuffleboardTab
        .addNumber(String.format("%s AE Pos", moduleName), this.motorEncoder::getPosition)
        .withSize(1, 1);
  }

  public void set(WristPositions position) {
    motorPid.setReference(position.getPosition(), ControlType.kPosition);
  }
}
