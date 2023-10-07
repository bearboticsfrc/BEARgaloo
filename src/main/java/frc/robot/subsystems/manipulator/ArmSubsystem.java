package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.manipulator.ArmConstants.ArmPositions;
import frc.robot.util.MotorConfig;
import frc.robot.util.MotorConfig.MotorBuilder;

public class ArmSubsystem extends SubsystemBase {
  private String name;
  private RelativeEncoder motorEncoder;
  private SparkMaxPIDController motorPid;
  private CANSparkMax motor;
  private CANSparkMax followerMotor;

  public ArmSubsystem(MotorBuilder motorConstants, MotorBuilder followerMotorConstants) {
    this.name = motorConstants.getName();

    this.motor =
        new CANSparkMax(motorConstants.getMotorPort(), CANSparkMaxLowLevel.MotorType.kBrushless);
    this.followerMotor =
        new CANSparkMax(
            followerMotorConstants.getMotorPort(), CANSparkMaxLowLevel.MotorType.kBrushless);

    this.motorEncoder = motor.getEncoder();
    this.motorPid = motor.getPIDController();

    followerMotor.follow(motor, true);

    MotorConfig.fromMotorConstants(motor, motorEncoder, motorConstants)
        .configureMotor()
        .configurePID(motorConstants.getMotorPid(0), 0)
        .configurePID(motorConstants.getMotorPid(1), 1)
        .burnFlash();

    MotorConfig.fromMotorConstants(
            followerMotor, followerMotor.getEncoder(), followerMotorConstants)
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
        .withSize(1, 1);

    shuffleboardTab
        .addNumber(String.format("%s m Amps", name), this.motor::getAppliedOutput)
        .withSize(1, 1);
    shuffleboardTab
        .addNumber(String.format("%s f Amps", name), this.followerMotor::getAppliedOutput)
        .withSize(1, 1);
  }

  public void set(ArmPositions position) {
    int slot = position == ArmPositions.HIGH ? 0 : 1;
    motorPid.setReference(position.getPosition(), ControlType.kPosition, slot);
  }
}
