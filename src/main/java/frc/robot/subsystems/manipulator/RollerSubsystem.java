package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.manipulator.RollerConstants.RollerSpeed;
import frc.robot.util.MotorConfig;
import frc.robot.util.MotorConfig.MotorBuilder;

public class RollerSubsystem extends SubsystemBase {
  private String moduleName;
  private RelativeEncoder motorEncoder;
  private SparkMaxPIDController motorPid;
  private CANSparkMax motor;

  public RollerSubsystem(MotorBuilder motorConstants) {
    this.moduleName = motorConstants.getName();

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

  /**
   * @param shuffleboardTab The shuffleboard tab to use
   */
  private void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    shuffleboardTab
        .addNumber(String.format("%s Pos", moduleName), this.motorEncoder::getPosition)
        .withSize(1, 1);
    shuffleboardTab
        .addNumber(String.format("%s Vel", moduleName), this.motorEncoder::getVelocity)
        .withSize(1, 1);
  }

  /**
   * Sets the roller speed.
   *
   * @param speed An enum representing the set speed.
   */
  public void set(RollerSpeed speed) {
    motorPid.setReference(speed.getSpeed(), ControlType.kPosition);
  }
}
