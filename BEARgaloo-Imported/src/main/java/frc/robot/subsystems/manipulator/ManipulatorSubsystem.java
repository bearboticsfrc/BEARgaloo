package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.manipulator.ArmConstants.ArmPositions;
import frc.robot.constants.manipulator.RollerConstants.RollerSpeed;
import frc.robot.constants.manipulator.WristConstants.WristPositions;
import java.util.HashMap;
import java.util.Map;

import com.revrobotics.spark.ClosedLoopSlot;

import frc.robot.constants.AutoConstants.ScorePosition;

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

    
    return new ArmSubsystem();
  }

  private WristSubsystem getWristSubsystem() {
    return new WristSubsystem();
  }

  private RollerSubsystem getRollerSubsystem() {
    return new RollerSubsystem();
  }

  public Command getRollerRunCommand(RollerSpeed speed) {
    return new InstantCommand(() -> rollerSubsystem.set(speed), rollerSubsystem);
  }

  public Command getWristRunCommand(WristPositions position) {
    return new InstantCommand(() -> wristSubsystem.set(position), wristSubsystem);
  }

  public Command getArmRunCommand(ArmPositions position) {
    return new InstantCommand(() -> armSubsystem.set(position), armSubsystem);
  }

  public void adjustWristHeight(double direction) {
    if (Math.abs(direction) < 0.01) {
      return;
    }

    double position = wristSubsystem.getTargetPosition() + (direction * 0.1);
    wristSubsystem.setReference(position);
  }

  public void setWristSpeed(double speed) {
    wristSubsystem.set(speed);
  }

  public Command getHomeAllCommand() {
    return new ParallelCommandGroup(
        getWristRunCommand(WristPositions.HOME),
        getArmRunCommand(ArmPositions.HOME),
        getRollerRunCommand(RollerSpeed.OFF),
        new WaitUntilCommand(armSubsystem::isHome),
        new InstantCommand(() -> armSubsystem.set(ArmPositions.HOME, ClosedLoopSlot.kSlot0)));
  }

  public Command getShelfScoreCommand(ScorePosition position) {
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
        getWristRunCommand(WristPositions.BOTTOM), getRollerRunCommand(RollerSpeed.INTAKE));
  }
/* 
  public Command getCubeHuntCommand(DriveSubsystem driveSubsystem) {
    return new SequentialCommandGroup(
        getPickupPositionCommand(), new CubeHuntCommand(driveSubsystem, this::hasCube));
  }
*/
  public boolean hasCube() {
    return rollerSubsystem.hasCube();
  }

  public boolean isWristHome() {
    return wristSubsystem.isHome();
  }

  public void calibrateWrist() {
    wristSubsystem.calibrate();
  }

  public HashMap<String, Command> getEventMap() {
    return new HashMap<>(
        Map.of(
            "lowerWrist", getWristRunCommand(WristPositions.BOTTOM),
            "startRollers", getRollerRunCommand(RollerSpeed.INTAKE)));
  }
}
