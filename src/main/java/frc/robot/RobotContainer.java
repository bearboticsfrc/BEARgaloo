// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.DriveConstants.SpeedMode;
import frc.robot.constants.manipulator.ArmConstants;
import frc.robot.constants.manipulator.RollerConstants;
import frc.robot.constants.manipulator.WristConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.ArmSubsystem;
import frc.robot.subsystems.manipulator.RollerSubsystem;
import frc.robot.util.MotorConfig.MotorBuilder;
import frc.robot.util.MotorConfig.MotorPIDBuilder;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ArmSubsystem armSubsystem;
  private final RollerSubsystem rollerSubsystem;
  private final CommandXboxController driverController =
      new CommandXboxController(DriveConstants.DRIVER_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveSubsystem.setDefaultCommand(getDefaultCommand());

    armSubsystem = getArmSubsystem();
    rollerSubsystem = getRollerSubsystem();

    configureControllerMappings();
  }

  private ArmSubsystem getArmSubsystem() {
    MotorPIDBuilder armMotorPid = new MotorPIDBuilder().setP(ArmConstants.Motor.MotorPid.P);

    MotorBuilder armMotorConfig =
        new MotorBuilder()
            .setName(ArmConstants.Motor.NAME)
            .setMotorPort(ArmConstants.Motor.MOTOR_PORT)
            .setCurrentLimit(ArmConstants.Motor.CURRENT_LIMT)
            .setMotorInverted(ArmConstants.Motor.INVERTED)
            .setEncoderInverted(ArmConstants.Motor.ENCODER_INVERTED)
            .setMotorPID(armMotorPid);

    MotorBuilder armFollowerMotorConfig =
        new MotorBuilder()
            .setName(ArmConstants.FollowerMotor.NAME)
            .setMotorPort(ArmConstants.FollowerMotor.MOTOR_PORT)
            .setCurrentLimit(ArmConstants.FollowerMotor.CURRENT_LIMT)
            .setMotorInverted(ArmConstants.FollowerMotor.INVERTED)
            .setEncoderInverted(ArmConstants.FollowerMotor.ENCODER_INVERTED)
            .setMotorPID(armMotorPid);

    return new ArmSubsystem(armMotorConfig, armFollowerMotorConfig);
  }

  private RollerSubsystem getRollerSubsystem() {
    MotorPIDBuilder rollerMotorPid = new MotorPIDBuilder().setP(RollerConstants.Motor.MotorPid.P);

    MotorBuilder rollerMotorConfig =
        new MotorBuilder()
            .setName(RollerConstants.Motor.NAME)
            .setMotorPort(RollerConstants.Motor.MOTOR_PORT)
            .setCurrentLimit(RollerConstants.Motor.CURRENT_LIMT)
            .setMotorInverted(RollerConstants.Motor.INVERTED)
            .setEncoderInverted(RollerConstants.Motor.ENCODER_INVERTED)
            .setMotorPID(rollerMotorPid);

    return new RollerSubsystem(rollerMotorConfig);
  }

  private RollerSubsystem getWristSubsystem() {
    MotorPIDBuilder wristMotorPid = new MotorPIDBuilder().setP(WristConstants.Motor.MotorPid.P);

    MotorBuilder wristMotorConfig =
        new MotorBuilder()
            .setName(WristConstants.Motor.NAME)
            .setMotorPort(WristConstants.Motor.MOTOR_PORT)
            .setCurrentLimit(WristConstants.Motor.CURRENT_LIMT)
            .setMotorInverted(WristConstants.Motor.INVERTED)
            .setEncoderInverted(WristConstants.Motor.ENCODER_INVERTED)
            .setMotorPID(wristMotorPid);

    return new RollerSubsystem(wristMotorConfig);
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

  public void configureControllerMappings() {
    configureDriverController();
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
}
