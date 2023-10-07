// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auto.DropCubeBottomExitCommunity;
import frc.robot.commands.auto.DropCubeTopExitCommunity;
import frc.robot.commands.auto.LeaveCommunityBottom;
import frc.robot.commands.auto.LeaveCommunityTop;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.DriveConstants.SpeedMode;
import frc.robot.constants.manipulator.ArmConstants.ArmPositions;
import frc.robot.constants.manipulator.RollerConstants.RollerSpeed;
import frc.robot.constants.manipulator.WristConstants.WristPositions;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import java.util.ArrayList;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private List<Pair<String, Command>> autoList = new ArrayList<Pair<String, Command>>();
  private final DriveSubsystem robotDrive = new DriveSubsystem();
  private boolean isTeleop = false;
  private final ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem();

  private final CommandXboxController driverController =
      new CommandXboxController(DriveConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController =
      new CommandXboxController(DriveConstants.OPERATOR_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveSubsystem.setDefaultCommand(getDefaultCommand());
    setManipulatorDefaultCommand();
    configureControllerMappings();
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

  private void setManipulatorDefaultCommand() {
    manipulatorSubsystem.setDefaultCommand(
        new RunCommand(
            () -> {
              // manipulator.runRollerDefault(
              //    MathUtil.applyDeadband(
              //        squareWithSign(operatorXboxController.getLeftTriggerAxis()), 0.1),
              //    MathUtil.applyDeadband(
              //        squareWithSign(operatorXboxController.getRightTriggerAxis()), 0.1));

              manipulatorSubsystem.adjustWristHeight(
                  MathUtil.applyDeadband(operatorController.getRightY(), 0.2));
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
        .leftBumper()
        .onTrue(new InstantCommand(() -> driveSubsystem.setFieldRelative(false)))
        .onFalse(new InstantCommand(() -> driveSubsystem.setFieldRelative(true)));

    driverController
        .leftTrigger(0.1)
        .onTrue(new InstantCommand(() -> driveSubsystem.setSpeedMode(SpeedMode.TURBO)))
        .onFalse(new InstantCommand(() -> driveSubsystem.setSpeedMode((SpeedMode.TURTLE))));

    driverController
        .rightTrigger(0.1)
        .onTrue(new InstantCommand(() -> driveSubsystem.setSpeedMode(SpeedMode.TURTLE)))
        .onFalse(new InstantCommand(() -> driveSubsystem.setSpeedMode((SpeedMode.TURBO))));
  }

  public void configureOperatorController() {
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
    operatorController.povUp().onTrue(manipulatorSubsystem.getHighScoreCommand());
    operatorController.povLeft().onTrue(manipulatorSubsystem.getMidScoreCommand());
    operatorController
        .povRight()
        .onTrue(manipulatorSubsystem.getWristRunCommand(WristPositions.HIGH));
    operatorController.b().onTrue(manipulatorSubsystem.getWristRunCommand(WristPositions.BOTTOM));
    operatorController.y().onTrue(manipulatorSubsystem.getHomeAllCommand());

    operatorController.x().onTrue(manipulatorSubsystem.getArmRunCommand(ArmPositions.HIGH));

    operatorController.a().onTrue(manipulatorSubsystem.getArmRunCommand(ArmPositions.HOME));
  }
  private void addToAutoList(String name, Command command) {
    autoList.add(new Pair<String, Command>(name, command));
  }

  private void buildAutoList() {
    addToAutoList("1-DropCubeBottomExitCommunity", DropCubeBottomExitCommunity.get(robotDrive));
    addToAutoList("2-DropCubeTopmmunity", DropCubeTopExitCommunity.get(robotDrive));
    addToAutoList("3-LeaveCommunityBottom", LeaveCommunityBottom.get(robotDrive));
    addToAutoList("4-LeaveCommunityTop", LeaveCommunityTop.get(robotDrive));
  }

  public Command getAutonomousCommand() {
    return new RunCommand(() -> "".isEmpty()); // TODO: Implement
  }
}
