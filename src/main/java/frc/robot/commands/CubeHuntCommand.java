package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.LimelightHelpers;
import java.util.function.BooleanSupplier;

public class CubeHuntCommand extends Command {

  private final DriveSubsystem driveSubsystem;
  private final BooleanSupplier hasCube;
  private final String LIMELIGHT_NAME = "limelight";
  private final PIDController rotSpeedController = new PIDController(0.001, 0, 0);
  private final PIDController xSpeedController = new PIDController(0.1, 0, 0);
  private final BooleanLogEntry logEntry =
      new BooleanLogEntry(
          DataLogManager.getLog(), "/command/CubeHunt", "Indicates the command is active");
  private boolean isActive = false;

  public CubeHuntCommand(DriveSubsystem driveSubsystem, BooleanSupplier hasCube, String name) {
    this.driveSubsystem = driveSubsystem;
    this.hasCube = hasCube;

    RobotConstants.MANIPULATOR_SYSTEM_TAB.addBoolean(name, this::getActive);

    addRequirements(driveSubsystem);
    rotSpeedController.setTolerance(2);
    xSpeedController.setTolerance(2);
    logEntry.append(false);
  }

  public boolean getActive() {
    return isActive;
  }

  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex(LIMELIGHT_NAME, 0);
    isActive = true;
    logEntry.append(true);
  }

  @Override
  public void execute() {
    if (!LimelightHelpers.getTV(LIMELIGHT_NAME)) {
      // driveSubsystem.drive(0, 0, 0);
      return;
    }

    // double id = LimelightHelpers.getNeuralClassID(LIMELIGHT_NAME);
    // double [] colors = LimelightHelpers.getTargetColor(LIMELIGHT_NAME);

    // System.out.println("neural classid == " + id + " color = " + colors[0]);

    // if (id < 0.0) {
    // return;
    // }

    // if ( id != 0.0 ) {
    // System.out.println("neural classid == " + id);
    // return;
    // }
    // System.out.println(" id = " + id);
    double targetY = LimelightHelpers.getTY(LIMELIGHT_NAME);
    double targetX = LimelightHelpers.getTX(LIMELIGHT_NAME);

    double xSpeed = -xSpeedController.calculate(targetY, 0);
    double rot = rotSpeedController.calculate(targetX, 0);

    if (xSpeedController.atSetpoint()) {
      xSpeed += 1;
    }

    driveSubsystem.drive(xSpeed, 0, rot, false);
  }

  @Override
  public boolean isFinished() {
    return hasCube.getAsBoolean();
  }

  @Override
  public void end(boolean interrupted) {
    isActive = false;
    logEntry.append(false);
    driveSubsystem.drive(0, 0, 0);
  }
}
