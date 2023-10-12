// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.DriveConstants.SpeedMode;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.SwerveCorner;
import frc.robot.constants.SwerveModuleConstants.BackLeftConstants;
import frc.robot.constants.SwerveModuleConstants.BackRightConstants;
import frc.robot.constants.SwerveModuleConstants.FrontLeftConstants;
import frc.robot.constants.SwerveModuleConstants.FrontRightConstants;
import frc.robot.subsystems.SwerveModule.SwerveModuleBuilder;
import frc.robot.util.CTREUtil;
import frc.robot.util.MotorConfig.MotorBuilder;
import frc.robot.util.MotorConfig.MotorPIDBuilder;
import java.util.Arrays;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Map;

/** Controls the four swerve modules for autonomous and teleoperated modes. */
public class DriveSubsystem extends SubsystemBase {
  // Linked to maintain order.
  private final LinkedHashMap<SwerveCorner, SwerveModule> swerveModules = new LinkedHashMap<>();
  private final WPI_PigeonIMU pigeonImu = new WPI_PigeonIMU(RobotConstants.PIGEON_CAN_ID);

  private final SwerveDriveOdometry odometry;
  private GenericEntry competitionTabMaxSpeedEntry;

  private double maxSpeed = DriveConstants.DRIVE_VELOCITY;
  private boolean fieldRelativeMode = true;

  public DriveSubsystem() {
    CTREUtil.checkCtreError(pigeonImu.configFactoryDefault());

    for (SwerveCorner corner : SwerveCorner.values()) {
      swerveModules.put(
          corner,
          new SwerveModule(getSwerveConfigForCorner(corner), DriveConstants.DRIVE_SYSTEM_TAB));
    }

    odometry =
        new SwerveDriveOdometry(
            RobotConstants.DRIVE_KINEMATICS, getHeading(), getModulePositions());

    zeroHeading();
    setupShuffleboardTab();
  }

  public void zeroHeading() {
    pigeonImu.reset();
  }

  private void setupShuffleboardTab() {
    competitionTabMaxSpeedEntry =
        DriveConstants.COMPETITION_TAB
            .add("Maximum Drive Speed", maxSpeed)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withSize(2, 1)
            .withPosition(5, 1)
            .withProperties(Map.of("min", 0, "max", maxSpeed * 2))
            .getEntry();

    DriveConstants.COMPETITION_TAB.addNumber("Pigeon Heading", () -> getHeading().getDegrees());
    DriveConstants.DRIVE_SYSTEM_TAB.addDouble("Pitch", this::getPitch);
    DriveConstants.DRIVE_SYSTEM_TAB.addDouble("Roll", this::getRoll);
    DriveConstants.DRIVE_SYSTEM_TAB.addBoolean("Field Relative?", () -> fieldRelativeMode);
  }

  @Override
  public void periodic() {
    odometry.update(getHeading(), getModulePositions());

    maxSpeed = competitionTabMaxSpeedEntry.getDouble(DriveConstants.MAX_VELOCITY);

    for (SwerveModule module : swerveModules.values()) {
      module.updateDataLogs();
    }
  }

  private SwerveModuleBuilder getSwerveConfigForCorner(SwerveCorner corner) {
    switch (corner) {
      case FRONT_LEFT:
        return getFrontLeftSwerveConfig();
      case BACK_LEFT:
        return getBackLeftSwerveConfig();
      case FRONT_RIGHT:
        return getFrontRightSwerveConfig();
      case BACK_RIGHT:
        return getBackRightSwerveConfig();
      default:
        throw new IllegalArgumentException("Unknown corner: " + corner);
    }
  }

  private SwerveModuleBuilder getFrontLeftSwerveConfig() {
    MotorPIDBuilder driveMotorPid =
        new MotorPIDBuilder()
            .setP(FrontLeftConstants.DriveMotor.MotorPid.P)
            .setFf(FrontLeftConstants.DriveMotor.MotorPid.Ff);

    MotorPIDBuilder pivotMotorPid =
        new MotorPIDBuilder()
            .setP(FrontLeftConstants.PivotMotor.MotorPid.P)
            .setPositionPidWrappingEnabled(
                FrontLeftConstants.PivotMotor.MotorPid.POSITION_PID_WRAPPING_ENABLED)
            .setPositionPidWrappingMin(
                FrontLeftConstants.PivotMotor.MotorPid.POSITION_PID_WRAPPING_MIN)
            .setPositionPidWrappingMax(
                FrontLeftConstants.PivotMotor.MotorPid.POSITION_PID_WRAPPING_MAX);

    MotorBuilder driveConfig =
        new MotorBuilder()
            .setName(FrontLeftConstants.DriveMotor.NAME)
            .setMotorPort(FrontLeftConstants.DriveMotor.MOTOR_PORT)
            .setCurrentLimit(FrontLeftConstants.DriveMotor.CURRENT_LIMT)
            .setMotorInverted(FrontLeftConstants.DriveMotor.INVERTED)
            .setEncoderInverted(FrontLeftConstants.DriveMotor.ENCODER_INVERTED)
            .setPositionConversionFactor(FrontLeftConstants.DriveMotor.POSITION_CONVERSION_FACTOR)
            .setVelocityConversionFactor(FrontLeftConstants.DriveMotor.VELOCITY_CONVERSION_FACTOR)
            .setMotorPID(driveMotorPid);

    MotorBuilder pivotConfig =
        new MotorBuilder()
            .setName(FrontLeftConstants.PivotMotor.NAME)
            .setMotorPort(FrontLeftConstants.PivotMotor.MOTOR_PORT)
            .setCurrentLimit(FrontLeftConstants.PivotMotor.CURRENT_LIMT)
            .setMotorInverted(FrontLeftConstants.PivotMotor.INVERTED)
            .setEncoderInverted(FrontLeftConstants.PivotMotor.ENCODER_INVERTED)
            .setPositionConversionFactor(FrontLeftConstants.PivotMotor.POSITION_CONVERSION_FACTOR)
            .setVelocityConversionFactor(FrontLeftConstants.PivotMotor.VELOCITY_CONVERSION_FACTOR)
            .setMotorPID(pivotMotorPid);

    SwerveModuleBuilder moduleConfig =
        new SwerveModuleBuilder()
            .setModuleName(FrontLeftConstants.MODULE_NAME)
            .setParkAngle(FrontLeftConstants.PARK_ANGLE)
            .setChassisAngularOffset(FrontLeftConstants.CHASSIS_ANGULAR_OFFSET)
            .setDriveMotor(driveConfig)
            .setPivotMotor(pivotConfig);

    return moduleConfig;
  }

  private SwerveModuleBuilder getBackLeftSwerveConfig() {
    MotorPIDBuilder driveMotorPid =
        new MotorPIDBuilder()
            .setP(BackLeftConstants.DriveMotor.MotorPid.P)
            .setFf(BackLeftConstants.DriveMotor.MotorPid.Ff);

    MotorPIDBuilder pivotMotorPid =
        new MotorPIDBuilder()
            .setP(BackLeftConstants.PivotMotor.MotorPid.P)
            .setPositionPidWrappingEnabled(
                BackLeftConstants.PivotMotor.MotorPid.POSITION_PID_WRAPPING_ENABLED)
            .setPositionPidWrappingMin(
                BackLeftConstants.PivotMotor.MotorPid.POSITION_PID_WRAPPING_MIN)
            .setPositionPidWrappingMax(
                BackLeftConstants.PivotMotor.MotorPid.POSITION_PID_WRAPPING_MAX);

    MotorBuilder driveConfig =
        new MotorBuilder()
            .setName(BackLeftConstants.DriveMotor.NAME)
            .setMotorPort(BackLeftConstants.DriveMotor.MOTOR_PORT)
            .setCurrentLimit(BackLeftConstants.DriveMotor.CURRENT_LIMT)
            .setMotorInverted(BackLeftConstants.DriveMotor.INVERTED)
            .setEncoderInverted(BackLeftConstants.DriveMotor.ENCODER_INVERTED)
            .setPositionConversionFactor(BackLeftConstants.DriveMotor.POSITION_CONVERSION_FACTOR)
            .setVelocityConversionFactor(BackLeftConstants.DriveMotor.VELOCITY_CONVERSION_FACTOR)
            .setMotorPID(driveMotorPid);

    MotorBuilder pivotConfig =
        new MotorBuilder()
            .setName(BackLeftConstants.PivotMotor.NAME)
            .setMotorPort(BackLeftConstants.PivotMotor.MOTOR_PORT)
            .setCurrentLimit(BackLeftConstants.PivotMotor.CURRENT_LIMT)
            .setMotorInverted(BackLeftConstants.PivotMotor.INVERTED)
            .setEncoderInverted(BackLeftConstants.PivotMotor.ENCODER_INVERTED)
            .setPositionConversionFactor(BackLeftConstants.PivotMotor.POSITION_CONVERSION_FACTOR)
            .setVelocityConversionFactor(BackLeftConstants.PivotMotor.VELOCITY_CONVERSION_FACTOR)
            .setMotorPID(pivotMotorPid);

    SwerveModuleBuilder moduleConfig =
        new SwerveModuleBuilder()
            .setModuleName(BackLeftConstants.MODULE_NAME)
            .setParkAngle(BackLeftConstants.PARK_ANGLE)
            .setChassisAngularOffset(BackLeftConstants.CHASSIS_ANGULAR_OFFSET)
            .setDriveMotor(driveConfig)
            .setPivotMotor(pivotConfig);

    return moduleConfig;
  }

  private SwerveModuleBuilder getFrontRightSwerveConfig() {
    MotorPIDBuilder driveMotorPid =
        new MotorPIDBuilder()
            .setP(FrontRightConstants.DriveMotor.MotorPid.P)
            .setFf(FrontRightConstants.DriveMotor.MotorPid.Ff);

    MotorPIDBuilder pivotMotorPid =
        new MotorPIDBuilder()
            .setP(FrontRightConstants.PivotMotor.MotorPid.P)
            .setPositionPidWrappingEnabled(
                FrontRightConstants.PivotMotor.MotorPid.POSITION_PID_WRAPPING_ENABLED)
            .setPositionPidWrappingMin(
                FrontRightConstants.PivotMotor.MotorPid.POSITION_PID_WRAPPING_MIN)
            .setPositionPidWrappingMax(
                FrontRightConstants.PivotMotor.MotorPid.POSITION_PID_WRAPPING_MAX);

    MotorBuilder driveConfig =
        new MotorBuilder()
            .setName(FrontRightConstants.DriveMotor.NAME)
            .setMotorPort(FrontRightConstants.DriveMotor.MOTOR_PORT)
            .setCurrentLimit(FrontRightConstants.DriveMotor.CURRENT_LIMT)
            .setMotorInverted(FrontRightConstants.DriveMotor.INVERTED)
            .setEncoderInverted(FrontRightConstants.DriveMotor.ENCODER_INVERTED)
            .setPositionConversionFactor(FrontRightConstants.DriveMotor.POSITION_CONVERSION_FACTOR)
            .setVelocityConversionFactor(FrontRightConstants.DriveMotor.VELOCITY_CONVERSION_FACTOR)
            .setMotorPID(driveMotorPid);

    MotorBuilder pivotConfig =
        new MotorBuilder()
            .setName(FrontRightConstants.PivotMotor.NAME)
            .setMotorPort(FrontRightConstants.PivotMotor.MOTOR_PORT)
            .setCurrentLimit(FrontRightConstants.PivotMotor.CURRENT_LIMT)
            .setMotorInverted(FrontRightConstants.PivotMotor.INVERTED)
            .setEncoderInverted(FrontRightConstants.PivotMotor.ENCODER_INVERTED)
            .setPositionConversionFactor(FrontRightConstants.PivotMotor.POSITION_CONVERSION_FACTOR)
            .setVelocityConversionFactor(FrontRightConstants.PivotMotor.VELOCITY_CONVERSION_FACTOR)
            .setMotorPID(pivotMotorPid);

    SwerveModuleBuilder moduleConfig =
        new SwerveModuleBuilder()
            .setModuleName(FrontRightConstants.MODULE_NAME)
            .setParkAngle(FrontRightConstants.PARK_ANGLE)
            .setChassisAngularOffset(FrontRightConstants.CHASSIS_ANGULAR_OFFSET)
            .setDriveMotor(driveConfig)
            .setPivotMotor(pivotConfig);

    return moduleConfig;
  }

  private SwerveModuleBuilder getBackRightSwerveConfig() {
    MotorPIDBuilder driveMotorPid =
        new MotorPIDBuilder()
            .setP(BackRightConstants.DriveMotor.MotorPid.P)
            .setFf(BackRightConstants.DriveMotor.MotorPid.Ff);

    MotorPIDBuilder pivotMotorPid =
        new MotorPIDBuilder()
            .setP(BackRightConstants.PivotMotor.MotorPid.P)
            .setPositionPidWrappingEnabled(
                BackRightConstants.PivotMotor.MotorPid.POSITION_PID_WRAPPING_ENABLED)
            .setPositionPidWrappingMin(
                BackRightConstants.PivotMotor.MotorPid.POSITION_PID_WRAPPING_MIN)
            .setPositionPidWrappingMax(
                BackRightConstants.PivotMotor.MotorPid.POSITION_PID_WRAPPING_MAX);

    MotorBuilder driveConfig =
        new MotorBuilder()
            .setName(BackRightConstants.DriveMotor.NAME)
            .setMotorPort(BackRightConstants.DriveMotor.MOTOR_PORT)
            .setCurrentLimit(BackRightConstants.DriveMotor.CURRENT_LIMT)
            .setMotorInverted(BackRightConstants.DriveMotor.INVERTED)
            .setEncoderInverted(BackRightConstants.DriveMotor.ENCODER_INVERTED)
            .setPositionConversionFactor(BackRightConstants.DriveMotor.POSITION_CONVERSION_FACTOR)
            .setVelocityConversionFactor(BackRightConstants.DriveMotor.VELOCITY_CONVERSION_FACTOR)
            .setMotorPID(driveMotorPid);

    MotorBuilder pivotConfig =
        new MotorBuilder()
            .setName(BackRightConstants.PivotMotor.NAME)
            .setMotorPort(BackRightConstants.PivotMotor.MOTOR_PORT)
            .setCurrentLimit(BackRightConstants.PivotMotor.CURRENT_LIMT)
            .setMotorInverted(BackRightConstants.PivotMotor.INVERTED)
            .setEncoderInverted(BackRightConstants.PivotMotor.ENCODER_INVERTED)
            .setPositionConversionFactor(BackRightConstants.PivotMotor.POSITION_CONVERSION_FACTOR)
            .setVelocityConversionFactor(BackRightConstants.PivotMotor.VELOCITY_CONVERSION_FACTOR)
            .setMotorPID(pivotMotorPid);

    SwerveModuleBuilder moduleConfig =
        new SwerveModuleBuilder()
            .setModuleName(BackRightConstants.MODULE_NAME)
            .setParkAngle(BackRightConstants.PARK_ANGLE)
            .setChassisAngularOffset(BackRightConstants.CHASSIS_ANGULAR_OFFSET)
            .setDriveMotor(driveConfig)
            .setPivotMotor(pivotConfig);

    return moduleConfig;
  }

  public double getPitch() {
    return pigeonImu.getPitch();
  }

  public double getRoll() {
    return pigeonImu.getRoll();
  }

  /**
   * Get an array of swerve modules in order.
   *
   * <p>This order is detemrined by {@link SwerveCorner}
   *
   * @return An array containing the swerve modules, ordered.
   */
  private SwerveModule[] getSwerveModules() {
    return (SwerveModule[]) swerveModules.values().toArray(new SwerveModule[4]);
  }

  /**
   * Activates or deactivates the park mode for the robot.
   *
   * <p>When park mode is activated, each wheel is locked in an opposing configuration, preventing
   * any movement.
   *
   * @param enabled true to activate park mode, false to deactivate.
   */
  public void setParkMode(boolean enabled) {
    for (SwerveModule module : getSwerveModules()) {
      if (!enabled) {
        module.setParked(false);
        continue;
      }

      SwerveModuleState state = new SwerveModuleState(0, module.getParkedAngle());

      module.set(state);
      module.setParked(true);
    }
  }

  /**
   * Multiplies the maximum speed for the robot based on the specified speed mode.
   *
   * @param mode The specified speed mode set by {@link SpeedMode}.
   */
  public void setSpeedMode(SpeedMode mode) {
    maxSpeed =
        Math.max(
            Math.min(maxSpeed * mode.getMaxSpeedMultiplier(), DriveConstants.MAX_VELOCITY),
            SpeedMode.TURTLE.getMaxSpeed());

    competitionTabMaxSpeedEntry.setDouble(maxSpeed);
  }

  /**
   * Sets whether the robot's movement is interpreted as field-relative or robot-relative.
   *
   * <p>{@code true} to enable field-relative mode, where the robot's movement is based on the
   * field's coordinates. {@code false} for robot-relative mode, where the robot's movement is based
   * on its own coordinates regardless of field orientation.
   *
   * @param mode Whether field-relative is enabled or not.
   */
  public void setFieldRelative(boolean mode) {
    fieldRelativeMode = mode;
  }

  /**
   * Drives the robot using joystick inputs, with the default pivot mode set to CENTER, and adjusts
   * the movement interpretation based on the current field-relative mode setting.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   */
  public void drive(double xSpeed, double ySpeed, double rot) {
    drive(xSpeed, ySpeed, rot, fieldRelativeMode);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    xSpeed = DriveConstants.X_ACCELERATION_LIMITER.calculate(xSpeed) * maxSpeed;
    ySpeed = DriveConstants.Y_ACCELERATION_LIMITER.calculate(ySpeed) * maxSpeed;

    rot =
        DriveConstants.TURNING_ACCELERATION_LIMITER.calculate(rot)
            * DriveConstants.MAX_ANGULAR_ACCELERATION_PER_SECOND;

    if (maxSpeed == SpeedMode.TURTLE.getMaxSpeed()) {
      rot /= 18;
    } else if (maxSpeed == SpeedMode.TURBO.getMaxSpeed()) {
      rot /= 4;
    }

    System.out.println("xSpeed = " + xSpeed + "\nrot = " + rot);

    SwerveModuleState[] swerveModuleStates =
        RobotConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));

    setModuleStates(swerveModuleStates);
  }

  /** Turn off the drive motors */
  public void stop() {
    drive(0, 0, 0);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param swerveModuleStates The desired swerve module states, ordered.
   */
  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);
    Iterator<SwerveModuleState> stateIterator = Arrays.asList(swerveModuleStates).iterator();

    for (SwerveModule module : swerveModules.values()) {
      module.set(stateIterator.next());
    }
  }

  /**
   * Returns the state of every swerve module.
   *
   * @return The states.
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    int index = 0;
    for (SwerveModule module : swerveModules.values()) {
      states[index++] = module.getState();
    }
    return states;
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading as a Rotation2d
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(
        MathUtil.inputModulus(pigeonImu.getRotation2d().getDegrees(), 0, 360));
  }

  /**
   * Returns the position of every swerve module.
   *
   * @return The positions.
   */
  public SwerveModulePosition[] getModulePositions() {
    return (SwerveModulePosition[])
        Arrays.stream(getSwerveModules())
            .map(module -> module.getPosition())
            .toArray(SwerveModulePosition[]::new);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getHeading(), getModulePositions(), pose);
    // pigeon2.addYaw(pose.getRotation().getDegrees());
  }

  /**
   * Resets the odometry to the specified pose of a state in a PathPlanner trajectory.
   *
   * @param state The state of the PathPlanner trajectory to contstruct a pose.
   */
  public void resetOdometry(PathPlannerState state) {
    resetOdometry(new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation));
  }
}
