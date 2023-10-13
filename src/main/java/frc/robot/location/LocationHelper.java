package frc.robot.location;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.LogCommand;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.fms.AllianceColor;
import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.vision.StringFormatting;

public class LocationHelper {

  public static PathPlannerTrajectory generateTrajectory(
      Pose2d robotPose, Pose2d target, Translation2d currentSpeed) {
    double travelSpeed = currentSpeed.getNorm();

    Translation2d robotToTargetTranslation =
        target.getTranslation().minus(robotPose.getTranslation());

    PathPlannerTrajectory trajectory =
        PathPlanner.generatePath(
            new PathConstraints(2, 4),
            new PathPoint(
                robotPose.getTranslation(),
                getDirection(robotToTargetTranslation),
                robotPose.getRotation(),
                travelSpeed),
            new PathPoint(
                target.getTranslation(),
                getDirection(robotToTargetTranslation),
                target.getRotation()));
    return trajectory;
  }

  public static Command followTrajectoryCommand(
      PathPlannerTrajectory trajectory, boolean isFirstPath, DriveSubsystem driveSubsystem) {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              DataLogManager.log(
                  "Starting trajectory from "
                      + StringFormatting.poseToString(trajectory.getInitialHolonomicPose())
                      + " to "
                      + StringFormatting.poseToString(trajectory.getEndState().poseMeters));
              if (isFirstPath) {
                driveSubsystem.resetOdometry(trajectory.getInitialHolonomicPose());
              }
            }),
        new PPSwerveControllerCommand(
            trajectory,
            driveSubsystem::getPose,
            RobotConstants.DRIVE_KINEMATICS,
            AutoConstants.X_SPEED_CONTROLLER,
            AutoConstants.Y_SPEED_CONTROLLER,
            AutoConstants.THETA_SPEED_CONTROLLER,
            driveSubsystem::setModuleStates,
            false,
            driveSubsystem),
        new LogCommand("Finished Trajectory"));
  }

  // direction utilities
  public static Rotation2d getDirection(Transform2d transform) {
    return getDirection(transform.getTranslation());
  }

  public static Rotation2d getDirection(Translation2d transform) {
    return new Rotation2d(transform.getX(), transform.getY());
  }

  public static double getDistance(Transform2d transform) {
    return getDistance(transform.getTranslation());
  }

  public static double getDistance(Translation2d transform) {
    return transform.getNorm();
  }

  public static Pose2d transformYAxisForAllianceColor(Pose2d pose) {
    if (AllianceColor.alliance == Alliance.Blue) {
      return pose;
    }
    Translation2d transformedTranslation =
        new Translation2d(pose.getX(), VisionConstants.FIELD_WIDTH_METERS - pose.getY());
    Rotation2d transformedHolonomicRotation = pose.getRotation().times(-1);
    return new Pose2d(transformedTranslation, transformedHolonomicRotation);
  }

  public static Rotation2d transformHeadingForAllianceColor(Rotation2d rotation) {
    if (AllianceColor.alliance == Alliance.Blue) {
      return rotation;
    }
    return rotation.times(-1.0);
  }

  public static Pose2d getPoseByDistanceAndAngleToPose(
      Pose2d pose, double distance, Rotation2d angle) {
    Translation2d translation = new Translation2d(distance, angle);
    return pose.transformBy(new Transform2d(translation.unaryMinus(), angle));
  }

  public static Translation2d getFieldRelativeLinearSpeedsMPS(DriveSubsystem driveSubsystem) {
    ChassisSpeeds robotRelativeSpeeds =
        RobotConstants.DRIVE_KINEMATICS.toChassisSpeeds(driveSubsystem.getModuleStates());
    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            robotRelativeSpeeds.vxMetersPerSecond,
            robotRelativeSpeeds.vyMetersPerSecond,
            robotRelativeSpeeds.omegaRadiansPerSecond,
            driveSubsystem.getPose().getRotation().unaryMinus());
    Translation2d translation =
        new Translation2d(
            fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
    if (getDistance(translation) < 0.01) {
      return new Translation2d();
    } else {
      return translation;
    }
  }

  public static double getDistanceToPose(Pose2d fromPose, Pose2d toPose) {
    double poseX = fromPose.getX();
    double poseY = fromPose.getY();
    double x = toPose.getX();
    double y = toPose.getY();
    double xDist = poseX - x;
    double yDist = poseY - y;
    double toProcess = xDist + yDist;
    toProcess *= toProcess;
    toProcess = Math.sqrt(toProcess);
    return toProcess;
  }

  public static Transform3d normalizeCameraAngle(Transform3d cameraToTarget) {
    double angle = Math.atan(cameraToTarget.getZ() / cameraToTarget.getX());
    double theta = -angle + Units.degreesToRadians(20);
    double hyp =
        Math.sqrt(
            (cameraToTarget.getX() * cameraToTarget.getX())
                + (cameraToTarget.getZ() * cameraToTarget.getZ()));
    double x_prime = hyp * Math.cos(theta);
    double z_prime = -hyp * Math.sin(theta);

    //    System.out.println("Camera_to_target:" +
    // StringFormatting.transformToString(cameraToTarget));
    //    System.out.println("ratio = "+cameraToTarget.getZ() / cameraToTarget.getX());
    //    System.out.println("angle = "+Units.radiansToDegrees(angle)+" theta = " +
    // Units.radiansToDegrees(theta) + "  hyp = "+hyp+" x_prime = " + x_prime);

    return new Transform3d(
        new Translation3d(x_prime, cameraToTarget.getY(), z_prime), cameraToTarget.getRotation());
  }
}
