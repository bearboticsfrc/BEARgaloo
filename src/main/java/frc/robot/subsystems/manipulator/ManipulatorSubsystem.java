package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.manipulator.ArmConstants;
import frc.robot.constants.manipulator.ArmConstants.ArmPositions;
import frc.robot.constants.manipulator.RollerConstants;
import frc.robot.constants.manipulator.RollerConstants.RollerSpeed;
import frc.robot.constants.manipulator.WristConstants;
import frc.robot.constants.manipulator.WristConstants.WristPositions;
import frc.robot.util.MotorConfig.MotorBuilder;
import frc.robot.util.MotorConfig.MotorPIDBuilder;

public class ManipulatorSubsystem extends SubsystemBase {
  private final ArmSubsystem armSubsystem;
  private final WristSubsystem wristSubsystem;
  private final RollerSubsystem rollerSubsystem;

  public ManipulatorSubsystem() {
    armSubsystem = getArmSubsystem();
    wristSubsystem = getWristSubsystem();
    rollerSubsystem = getRollerSubsystem();
  }

  private ArmSubsystem getArmSubsystem() {
    MotorPIDBuilder armMotorPidRaise =
        new MotorPIDBuilder().setP(ArmConstants.Motor.MotorPidRaise.P);
    MotorPIDBuilder armMotorPidLower =
        new MotorPIDBuilder().setP(ArmConstants.Motor.MotorPidLower.P);

    MotorBuilder armMotorConfig =
        new MotorBuilder()
            .setName(ArmConstants.Motor.NAME)
            .setMotorPort(ArmConstants.Motor.MOTOR_PORT)
            .setCurrentLimit(ArmConstants.Motor.CURRENT_LIMT)
            .setMotorInverted(ArmConstants.Motor.INVERTED)
            .setEncoderInverted(ArmConstants.Motor.ENCODER_INVERTED)
            .setMotorPid(armMotorPidRaise, 0)
            .setMotorPid(armMotorPidLower, 1);

    MotorBuilder armFollowerMotorConfig =
        new MotorBuilder()
            .setName(ArmConstants.FollowerMotor.NAME)
            .setMotorPort(ArmConstants.FollowerMotor.MOTOR_PORT)
            .setCurrentLimit(ArmConstants.FollowerMotor.CURRENT_LIMT)
            .setMotorInverted(ArmConstants.FollowerMotor.INVERTED)
            .setEncoderInverted(ArmConstants.FollowerMotor.ENCODER_INVERTED);

    return new ArmSubsystem(armMotorConfig, armFollowerMotorConfig);
  }

  private WristSubsystem getWristSubsystem() {
    MotorPIDBuilder wristMotorPid = new MotorPIDBuilder().setP(WristConstants.Motor.MotorPid.P);

    MotorBuilder wristMotorConfig =
        new MotorBuilder()
            .setName(WristConstants.Motor.NAME)
            .setMotorPort(WristConstants.Motor.MOTOR_PORT)
            .setCurrentLimit(WristConstants.Motor.CURRENT_LIMT)
            .setMotorInverted(WristConstants.Motor.INVERTED)
            .setEncoderInverted(WristConstants.Motor.ENCODER_INVERTED)
            .setMotorPID(wristMotorPid);

    return new WristSubsystem(wristMotorConfig);
  }

  private RollerSubsystem getRollerSubsystem() {
    MotorBuilder rollerMotorConfig =
        new MotorBuilder()
            .setName(RollerConstants.Motor.NAME)
            .setMotorPort(RollerConstants.Motor.MOTOR_PORT)
            .setCurrentLimit(RollerConstants.Motor.CURRENT_LIMT)
            .setMotorInverted(RollerConstants.Motor.INVERTED)
            .setEncoderInverted(RollerConstants.Motor.ENCODER_INVERTED);

    return new RollerSubsystem(rollerMotorConfig);
  }

  public CommandBase getRollerRunCommand(RollerSpeed speed) {
    return new InstantCommand(() -> rollerSubsystem.set(speed));
  }

  public CommandBase getWristRunCommand(WristPositions position) {
    return new InstantCommand(() -> wristSubsystem.set(position));
  }

  public void adjustWristHeight(double direction) {
    if (direction < 0.01 && direction > -0.01) return;
    double position = wristSubsystem.getPosition();
    position = position + (direction * 0.1);
    wristSubsystem.set(position);
  }

  public CommandBase getArmRunCommand(ArmPositions position) {
    return new InstantCommand(() -> armSubsystem.set(position));
  }

  public CommandBase getHomeAllCommand() {
    return new ParallelCommandGroup(
        getWristRunCommand(WristPositions.HOME), getArmRunCommand(ArmPositions.HOME));
  }

  public CommandBase getHighScoreCommand() {
    return new ParallelCommandGroup(
        getWristRunCommand(WristPositions.TOP), getArmRunCommand(ArmPositions.HIGH));
  }

  public CommandBase getMidScoreCommand() {
    return new ParallelCommandGroup(
        getWristRunCommand(WristPositions.TOP), getArmRunCommand(ArmPositions.HOME));
  }
}
