package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.CubeHuntCommand;
import frc.robot.constants.AutoConstants.ScorePosition;
import frc.robot.constants.manipulator.ArmConstants;
import frc.robot.constants.manipulator.ArmConstants.ArmPositions;
import frc.robot.constants.manipulator.RollerConstants;
import frc.robot.constants.manipulator.RollerConstants.RollerSpeed;
import frc.robot.constants.manipulator.WristConstants;
import frc.robot.constants.manipulator.WristConstants.WristPositions;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.MotorConfig.MotorBuilder;
import frc.robot.util.MotorConfig.MotorPIDBuilder;
import java.util.HashMap;

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
    return new InstantCommand(() -> rollerSubsystem.set(speed), rollerSubsystem);
  }

  public CommandBase getWristRunCommand(WristPositions position) {
    return new InstantCommand(() -> wristSubsystem.set(position), wristSubsystem);
  }

  public CommandBase getArmRunCommand(ArmPositions position) {
    return new InstantCommand(() -> armSubsystem.set(position), armSubsystem);
  }

  public void adjustWristHeight(double direction) {
    if (Math.abs(direction) < 0.01) {
      return;
    }

    double position = wristSubsystem.getTargetPosition() + (direction * 0.1);
    wristSubsystem.set(position);
  }

  public CommandBase getHomeAllCommand() {
    return new ParallelCommandGroup(
        getWristRunCommand(WristPositions.HOME),
        getArmRunCommand(ArmPositions.HOME),
        getRollerRunCommand(RollerSpeed.OFF),
        new WaitUntilCommand(armSubsystem::isHome),
        new InstantCommand(() -> armSubsystem.set(ArmPositions.HOME, 0)));
  }

  public CommandBase getShelfScoreCommand(ScorePosition position) {
    ArmPositions armPosition =
        position == ScorePosition.HIGH ? ArmPositions.HIGH : ArmPositions.HOME;

    return new ParallelCommandGroup(
        getWristRunCommand(WristPositions.HIGH), getArmRunCommand(armPosition));
  }

  public Command getShootCubeCommand() {
    return new SequentialCommandGroup(
        getRollerRunCommand(RollerSpeed.RELEASE),
        new WaitCommand(.2),
        getRollerRunCommand(RollerSpeed.OFF));
  }

  public Command getPickupPositionCommand() {
    return new SequentialCommandGroup(
        getRollerRunCommand(RollerSpeed.INTAKE), getWristRunCommand(WristPositions.BOTTOM));
  }

  public Command getCubeHuntCommand(DriveSubsystem driveSubsystem) {
    return new SequentialCommandGroup(
        getPickupPositionCommand(),
        new WaitCommand(0.04),
        new CubeHuntCommand(driveSubsystem, this::hasCube).withTimeout(2.0));
  }

  public boolean hasCube() {
    return rollerSubsystem.hasCube();
  }

  public HashMap<String, Command> getEventMap() {
    // TODO: see if can be refactored
    HashMap<String, Command> eventMap = new HashMap<>();

    eventMap.put("lowerWrist", getWristRunCommand(WristPositions.BOTTOM));
    eventMap.put("startRollers", getRollerRunCommand(RollerSpeed.INTAKE));

    return eventMap;
  }
}
