// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auto.DropCubeBottomExitCommunity;
import frc.robot.commands.auto.DropCubeTopExitCommunity;
import frc.robot.commands.auto.LeaveCommunityBottom;
import frc.robot.commands.auto.LeaveCommunityTop;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
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

  private final CommandXboxController driverController =
      new CommandXboxController(DriveConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController =
      new CommandXboxController(DriveConstants.OPERATOR_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    setDefaultCommand();
  }

  private void setDefaultCommand() {
    RunCommand defaultCommand =
        new RunCommand(
            () ->
                robotDrive.drive(
                    -MathUtil.applyDeadband(driverController.getLeftY(), 0.1),
                    -MathUtil.applyDeadband(driverController.getLeftX(), 0.1),
                    -MathUtil.applyDeadband(driverController.getRightX(), 0.1)),
            robotDrive);

    robotDrive.setDefaultCommand(defaultCommand);
  }

  public void configureControllerMappings() {
    configureDriverController();
    configureOperatorController();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureDriverController() {
    // Driver controller
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureOperatorController() {
    // operator controller
  }

  public void autonomousInit() {
    isTeleop = false;
    robotDrive.setParkMode(false);
  }

  public void teleopInit() {
    isTeleop = true;
    robotDrive.setParkMode(false);
  }

  public void disabledInit() {
    isTeleop = false;
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
