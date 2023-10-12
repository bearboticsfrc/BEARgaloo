package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.manipulator.RollerConstants.RollerSpeed;
import frc.robot.util.MotorConfig;
import frc.robot.util.MotorConfig.MotorBuilder;

public class RollerSubsystem extends SubsystemBase {
  private MedianFilter medianFilter = new MedianFilter(10);
  private String name;
  private RelativeEncoder motorEncoder;
  private CANSparkMax motor;

  public RollerSubsystem(MotorBuilder motorConstants) {
    this.name = motorConstants.getName();

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

  public boolean hasCube() {
    double adjustedOutput = medianFilter.calculate(motor.getOutputCurrent());

    return motorEncoder.getVelocity() < 0 && (adjustedOutput > 40);
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
    } // intake
  }
}
