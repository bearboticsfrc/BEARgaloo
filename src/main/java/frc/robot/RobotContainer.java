// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.auto.missions.AutoBalanceMission;
import frc.robot.commands.auto.missions.ParkMission;
import frc.robot.constants.AutoConstants.ScorePosition;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.DriveConstants.SpeedMode;
import frc.robot.constants.manipulator.RollerConstants.RollerSpeed;
import frc.robot.constants.manipulator.WristConstants.WristPositions;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.NoSuchElementException;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem();

  private final CommandXboxController driverController =
      new CommandXboxController(DriveConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController =
      new CommandXboxController(DriveConstants.OPERATOR_CONTROLLER_PORT);

  private boolean isTeleop = false;
  private Map<String, Command> missionsMap;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveSubsystem.setDefaultCommand(getDefaultCommand());
    setManipulatorDefaultCommand();
    configureControllerMappings();
    buildAutoList();

    missionsMap = new HashMap<>();
  }

  private RunCommand getDefaultCommand() {
    return new RunCommand(
        () ->
            driveSubsystem.drive(
                -MathUtil.applyDeadband(driverController.getLeftY(), 0.1),
                -MathUtil.applyDeadband(driverController.getLeftX(), 0.1),
                -MathUtil.applyDeadband(driverController.getRightX(), 0.1)),
        driveSubsystem);
  }

  public void setTeleop(boolean mode) {
    isTeleop = mode;
  }

  public void teleopInit() {
    driveSubsystem.setParkMode(false);
    driveSubsystem.setSpeedMode(SpeedMode.NORMAL);
  }

  private void setManipulatorDefaultCommand() {
    manipulatorSubsystem.setDefaultCommand(
        new RunCommand(
            () -> {
              manipulatorSubsystem.adjustWristHeight(
                  MathUtil.applyDeadband(operatorController.getRightY(), 0.2));
              /*  manipulatorSubsystem.setDefaultCommand(
              new RunCommand(
                  () ->
                      manipulatorSubsystem.runRollerDefault(
                          MathUtil.applyDeadband(operatorController.getLeftTriggerAxis(), 0.1),
                          MathUtil.applyDeadband(
                              operatorController.getRightTriggerAxis(), 0.1)),
                  manipulatorSubsystem));
                  */
            },
            manipulatorSubsystem));
  }

  public void configureControllerMappings() {
    configureDriverController();
    configureOperatorController();
  }

  private void configureDriverController() {
    driverController.a().onTrue(new InstantCommand(driveSubsystem::zeroHeading));

    driverController
        .b()
        .onTrue(new InstantCommand(() -> driveSubsystem.setParkMode(true)))
        .onFalse(new InstantCommand(() -> driveSubsystem.setParkMode(false)));

    driverController
        .x()
        .whileTrue(manipulatorSubsystem.getCubeHuntCommand(driveSubsystem))
        .onFalse(manipulatorSubsystem.getRollerRunCommand(RollerSpeed.OFF));

    driverController
        .leftBumper()
        .onTrue(new InstantCommand(() -> driveSubsystem.setFieldRelative(false)))
        .onFalse(new InstantCommand(() -> driveSubsystem.setFieldRelative(true)));

    driverController
        .leftTrigger(0.1)
        .onTrue(new InstantCommand(() -> driveSubsystem.setSpeedMode(SpeedMode.TURBO)))
        .onFalse(new InstantCommand(() -> driveSubsystem.setSpeedMode((SpeedMode.NORMAL))));

    driverController
        .rightTrigger(0.1)
        .onTrue(new InstantCommand(() -> driveSubsystem.setSpeedMode(SpeedMode.TURTLE)))
        .onFalse(new InstantCommand(() -> driveSubsystem.setSpeedMode((SpeedMode.NORMAL))));

    new Trigger(() -> manipulatorSubsystem.hasCube() && isTeleop)
        .onTrue(
            new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 1))
                .andThen(new WaitCommand(1))
                .andThen(
                    new InstantCommand(
                        () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0))));
  }

  public void configureOperatorController() {
    operatorController.a().onTrue(manipulatorSubsystem.getWristRunCommand(WristPositions.BOTTOM));
    operatorController.y().onTrue(manipulatorSubsystem.getHomeAllCommand());
    operatorController.x().onTrue(manipulatorSubsystem.getShelfScoreCommand(ScorePosition.HIGH));

    operatorController
        .leftTrigger(0.1)
        .onTrue(manipulatorSubsystem.getRollerRunCommand(RollerSpeed.RELEASE))
        .onFalse(manipulatorSubsystem.getRollerRunCommand(RollerSpeed.OFF));

    operatorController
        .rightTrigger(0.1)
        .onTrue(manipulatorSubsystem.getRollerRunCommand(RollerSpeed.INTAKE))
        .onFalse(manipulatorSubsystem.getRollerRunCommand(RollerSpeed.OFF));

    operatorController
        .povDown()
        .onTrue(manipulatorSubsystem.getWristRunCommand(WristPositions.BOTTOM));

    operatorController
        .povUp()
        .onTrue(manipulatorSubsystem.getShelfScoreCommand(ScorePosition.HIGH));

    operatorController
        .povLeft()
        .onTrue(manipulatorSubsystem.getShelfScoreCommand(ScorePosition.MIDDLE));

    operatorController
        .povRight()
        .onTrue(manipulatorSubsystem.getWristRunCommand(WristPositions.HIGH));

    operatorController.leftBumper().onTrue(manipulatorSubsystem.getShootCubeCommand());
  }

  /**
   * Gets a mission by name, fuzzily.
   *
   * @param name The mission name, can either be the index or name.
   * @return The mission.
   */
  public Command getMission(String name) {
    for (Entry<String, Command> missionEntry : missionsMap.entrySet()) {
      String[] normalized = missionEntry.getKey().split(" - ");

      if (normalized[0] == name) {
        return missionEntry.getValue();
      } else if (normalized[1].toLowerCase() == name.toLowerCase()) {
        return missionEntry.getValue();
      }
    }

    throw new NoSuchElementException(name + " Could not be fuzzily matched.");
  }

  /**
   * Gets a mission by name, fuzzily.
   *
   * @param name The mission name, can either be the index or name.
   * @return The mission.
   */
  public Command getMission(int index) {
    return getMission(String.valueOf(index));
  }

  public Command getMission(String name, boolean fuzzy) {
    if (fuzzy) {
      return getMission(name);
    }

    return missionsMap.get(name);
  }

  private void addMission(String name, Command command) {
    missionsMap.put(name, command);
  }

  private void buildAutoList() {
    addMission("0 - NoOp", new InstantCommand());
    addMission("1 - Park", new ParkMission(driveSubsystem));
    addMission("2 - Balance", new AutoBalanceMission(driveSubsystem));
  }
}
