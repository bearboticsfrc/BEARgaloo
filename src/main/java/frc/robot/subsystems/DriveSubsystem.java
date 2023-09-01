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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.SwerveModuleConstants.BackLeftConstants;
import frc.robot.constants.SwerveModuleConstants.BackRightConstants;
import frc.robot.constants.SwerveModuleConstants.FrontLeftConstants;
import frc.robot.constants.SwerveModuleConstants.FrontRightConstants;
import frc.robot.subsystems.SwerveModule.SwerveModuleBuilder;
import frc.robot.util.CTREUtil;
import frc.robot.util.MotorConfig.MotorBuilder;
import frc.robot.util.MotorConfig.MotorPIDBuilder;
import java.util.Map;

/** Controls the four swerve modules for autonomous and teleoperated modes. */
public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule frontLeft;
  private final SwerveModule backLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backRight;

  // The Pigeon IMU sensor
  public final WPI_PigeonIMU pigeon2 = new WPI_PigeonIMU(RobotConstants.PIGEON_CAN_ID);

  private double maxSpeed = DriveConstants.MAX_VELOCITY / 2;
  private double previousSpeed = maxSpeed;

  // Odometry class for tracking robot pose
  private SwerveDriveOdometry odometry;

  private ShuffleboardTab driveSystemTab = Shuffleboard.getTab("Drive System");
  private ShuffleboardTab competitionTab = Shuffleboard.getTab("Competition");
  private GenericEntry competitionTabMaxSpeedEntry;

  private boolean fieldRelativeMode = true;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    CTREUtil.checkCtreError(pigeon2.configFactoryDefault());

    frontLeft = new SwerveModule(getFrontLeftSwerveConfig(), driveSystemTab);
    backLeft = new SwerveModule(getBackLeftSwerveConfig(), driveSystemTab);
    frontRight = new SwerveModule(getFrontRightSwerveConfig(), driveSystemTab);
    backRight = new SwerveModule(getBackRightSwerveConfig(), driveSystemTab);

    odometry =
        new SwerveDriveOdometry(
            RobotConstants.DRIVE_KINEMATICS, getHeading(), getModulePositions());

    zeroHeading();
    setupShuffleboardTab();
  }

  private void setupShuffleboardTab() {
    competitionTabMaxSpeedEntry =
        competitionTab
            .add("Maximum Drive Speed", maxSpeed)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withSize(2, 1)
            .withPosition(5, 1)
            .withProperties(Map.of("min", 0, "max", maxSpeed))
            .getEntry();

    competitionTab.addNumber("pigeon2 heading", () -> getHeading().getDegrees());

    driveSystemTab.addDouble("Pitch", this::getPitch);
    driveSystemTab.addDouble("Roll", this::getRoll);
    driveSystemTab.addBoolean("Field Relative?", () -> fieldRelativeMode);
  }

  private SwerveModuleBuilder getFrontLeftSwerveConfig() {
    MotorPIDBuilder driveMotorPid =
        new MotorPIDBuilder().setP(FrontLeftConstants.DriveMotor.MotorPid.P);

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
            .setInverted(FrontLeftConstants.DriveMotor.INVERTED)
            .setMotorPID(driveMotorPid);

    MotorBuilder pivotConfig =
        new MotorBuilder()
            .setName(FrontLeftConstants.PivotMotor.NAME)
            .setMotorPort(FrontLeftConstants.PivotMotor.MOTOR_PORT)
            .setInverted(FrontLeftConstants.PivotMotor.INVERTED)
            .setMotorPID(pivotMotorPid);

    SwerveModuleBuilder moduleConfig =
        new SwerveModuleBuilder()
            .setModuleName(FrontLeftConstants.MODULE_NAME)
            .setParkAngle(FrontLeftConstants.PARK_ANGLE)
            .setDriveMotor(driveConfig)
            .setPivotMotor(pivotConfig);

    return moduleConfig;
  }

  private SwerveModuleBuilder getBackLeftSwerveConfig() {
    MotorPIDBuilder driveMotorPid =
        new MotorPIDBuilder().setP(BackLeftConstants.DriveMotor.MotorPid.P);

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
            .setInverted(BackLeftConstants.DriveMotor.INVERTED)
            .setMotorPID(driveMotorPid);

    MotorBuilder pivotConfig =
        new MotorBuilder()
            .setName(BackLeftConstants.PivotMotor.NAME)
            .setMotorPort(BackLeftConstants.PivotMotor.MOTOR_PORT)
            .setInverted(BackLeftConstants.PivotMotor.INVERTED)
            .setMotorPID(pivotMotorPid);

    SwerveModuleBuilder moduleConfig =
        new SwerveModuleBuilder()
            .setModuleName(BackLeftConstants.MODULE_NAME)
            .setParkAngle(BackLeftConstants.PARK_ANGLE)
            .setDriveMotor(driveConfig)
            .setPivotMotor(pivotConfig);

    return moduleConfig;
  }

  private SwerveModuleBuilder getFrontRightSwerveConfig() {
    MotorPIDBuilder driveMotorPid =
        new MotorPIDBuilder().setP(FrontRightConstants.DriveMotor.MotorPid.P);

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
            .setInverted(FrontRightConstants.DriveMotor.INVERTED)
            .setMotorPID(driveMotorPid);

    MotorBuilder pivotConfig =
        new MotorBuilder()
            .setName(FrontRightConstants.PivotMotor.NAME)
            .setMotorPort(FrontRightConstants.PivotMotor.MOTOR_PORT)
            .setInverted(FrontRightConstants.PivotMotor.INVERTED)
            .setMotorPID(pivotMotorPid);

    SwerveModuleBuilder moduleConfig =
        new SwerveModuleBuilder()
            .setModuleName(FrontRightConstants.MODULE_NAME)
            .setParkAngle(FrontRightConstants.PARK_ANGLE)
            .setDriveMotor(driveConfig)
            .setPivotMotor(pivotConfig);

    return moduleConfig;
  }

  private SwerveModuleBuilder getBackRightSwerveConfig() {
    MotorPIDBuilder driveMotorPid =
        new MotorPIDBuilder().setP(BackRightConstants.DriveMotor.MotorPid.P);

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
            .setInverted(BackRightConstants.DriveMotor.INVERTED)
            .setMotorPID(driveMotorPid);

    MotorBuilder pivotConfig =
        new MotorBuilder()
            .setName(BackRightConstants.PivotMotor.NAME)
            .setMotorPort(BackRightConstants.PivotMotor.MOTOR_PORT)
            .setInverted(BackRightConstants.PivotMotor.INVERTED)
            .setMotorPID(pivotMotorPid);

    SwerveModuleBuilder moduleConfig =
        new SwerveModuleBuilder()
            .setModuleName(BackRightConstants.MODULE_NAME)
            .setParkAngle(BackRightConstants.PARK_ANGLE)
            .setDriveMotor(driveConfig)
            .setPivotMotor(pivotConfig);

    return moduleConfig;
  }

  public double getPitch() {
    return pigeon2.getPitch();
  }

  public double getRoll() {
    return pigeon2.getRoll();
  }

  @Override
  public void periodic() {
    odometry.update(getHeading(), getModulePositions());

    maxSpeed = competitionTabMaxSpeedEntry.getDouble(DriveConstants.MAX_VELOCITY);

    frontLeft.updateDataLogs();
    backLeft.updateDataLogs();
    frontRight.updateDataLogs();
    backRight.updateDataLogs();
  }

  public void restorePreviousSpeed() {
    maxSpeed = previousSpeed;
  }

  public void setMaxSpeed() {
    previousSpeed = maxSpeed;
    maxSpeed = DriveConstants.MAX_VELOCITY;
    competitionTabMaxSpeedEntry.setDouble(maxSpeed);
  }

  public void setTurtleMode(boolean mode) {
    maxSpeed = mode ? 0.5 : DriveConstants.MAX_VELOCITY / 2;

    competitionTabMaxSpeedEntry.setDouble(maxSpeed);
  }

  /**
   * @param turboMode
   */
  public void setTurboMode(boolean mode) {
    // speed = mode ? Math.min(speed * 2.0,
    // DriveConstants.MAX_VELOCITY) : speed / 2;
    maxSpeed = mode ? DriveConstants.MAX_VELOCITY : maxSpeed / 2.0;

    competitionTabMaxSpeedEntry.setDouble(maxSpeed);
  }

  /**
   * Sets the robot to be field relative or not.
   *
   * @param mode The mode.
   */
  public void setFieldRelative(boolean mode) {
    fieldRelativeMode = mode;
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
   * Returns the state of every swerve module.
   *
   * @return The states.
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()
    };
  }

  /**
   * Resets the odometry to the specified pose of a state in a PathPlanner trajectory.
   *
   * @param state The state of the PathPlanner trajectory to contstruct a pose.
   */
  public void resetOdometry(PathPlannerState state) {
    resetOdometry(new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation));
  }

  /**
   * Drive robot using Joystick inputs, default to CENTER pivot, and sets field relative to the
   * current fieldRelativeMode setting.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   */
  public void drive(double xSpeed, double ySpeed, double rot) {
    this.drive(xSpeed, ySpeed, rot, fieldRelativeMode);
  }

  public Command getDriveSubsystemStopCommand() {
    return new InstantCommand(() -> stop());
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
    xSpeed =
        DriveConstants.X_ACCELERATION_LIMITER.calculate(xSpeed)
            * maxSpeed; // DriveConstants.MAX_VELOCITY;
    ySpeed =
        DriveConstants.Y_ACCELERATION_LIMITER.calculate(ySpeed)
            * maxSpeed; // DriveConstants.MAX_VELOCITY;

    if (maxSpeed == 0.5) {
      rot =
          DriveConstants.TURNING_ACCELERATION_LIMITER.calculate(rot)
              * DriveConstants.MAX_ANGULAR_ACCELERATION_PER_SECOND
              / 4.0;
    } else {
      rot =
          DriveConstants.TURNING_ACCELERATION_LIMITER.calculate(rot)
              * DriveConstants.MAX_ANGULAR_ACCELERATION_PER_SECOND;
    }
    var swerveModuleStates =
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
   * @param swerveModuleStates The desired swerve module states.
   */
  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);

    frontLeft.set(swerveModuleStates[0]);
    frontRight.set(swerveModuleStates[1]);
    backLeft.set(swerveModuleStates[2]);
    backRight.set(swerveModuleStates[3]);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    pigeon2.reset();
    // if (odometry != null) {
    // resetOdometry(getPose());
    // }
  }

  public void setHeadingOffest(double offset) {
    System.out.println("#################  Adding offset " + offset);
    pigeon2.addYaw(offset);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading as a Rotation2d
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(
        MathUtil.inputModulus(pigeon2.getRotation2d().getDegrees(), 0, 360));
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second.
   */
  public double getTurnRate() {
    return pigeon2.getRate();
  }

  /**
   * Returns the position of every swerve module.
   *
   * @return The positions.
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition(),
    };
  }

  public void setParkMode(boolean enabled) {
    for (SwerveModule module : new SwerveModule[] {frontLeft, frontRight, backLeft, backRight}) {
      if (!enabled) {
        module.setParked(false);
        continue;
      }

      SwerveModuleState state = module.getState();

      state.speedMetersPerSecond = 0;
      state.angle = module.getParkedAngle();

      module.set(state);
      module.setParked(true);
    }
  }
}
