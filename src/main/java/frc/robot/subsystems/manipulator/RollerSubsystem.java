package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.manipulator.RollerConstants.RollerSpeed;
import frc.robot.util.MotorConfig;
import frc.robot.util.MotorConfig.MotorBuilder;

public class RollerSubsystem extends SubsystemBase {
  private String moduleName;
  private RelativeEncoder motorEncoder;
  private CANSparkMax motor;

  public RollerSubsystem(MotorBuilder motorConstants) {
    this.moduleName = motorConstants.getName();

    this.motor =
        new CANSparkMax(motorConstants.getMotorPort(), CANSparkMaxLowLevel.MotorType.kBrushless);

    this.motorEncoder = motor.getEncoder();
    MotorConfig.fromMotorConstants(motor, motorEncoder, motorConstants)
        .configureMotor()
        .burnFlash();

    setupShuffleboardTab(RobotConstants.MANIPULATOR_SYSTEM_TAB);
    // setupDataLogging(DataLogManager.getLog()); TODO: impl
  }

  /**
   * @param shuffleboardTab The shuffleboard tab to use
   */
  private void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    shuffleboardTab
        .addNumber(String.format("%s Pos", moduleName), this.motorEncoder::getPosition)
        .withSize(2, 1);
    shuffleboardTab
        .addNumber(String.format("%s Amps", moduleName), this.motor::getOutputCurrent)
        .withSize(2, 1);
    shuffleboardTab
        .addNumber(String.format("%s Output", moduleName), this.motor::getAppliedOutput)
        .withSize(2, 1);
    shuffleboardTab.addBoolean(String.format("%s Cube?", moduleName), this::hasCube).withSize(1, 1);
  }

  private boolean hasCube() {
    return motor.getOutputCurrent() > 40;
  }
  /**
   * Sets the roller speed.
   *
   * @param speed An enum representing the set speed.
   */
  public void set(RollerSpeed speed) {
    motor.set(speed.getSpeed());
  }
}
