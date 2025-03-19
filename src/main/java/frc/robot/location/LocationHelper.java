package frc.robot.location;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.fms.AllianceColor;
import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.PoseEstimatorSubsystem;

public class LocationHelper {

  /*   public static PathPlannerTrajectory generateTrajectory(
        Pose2d robotPose, Pose2d target, Translation2d currentSpeed) {
      Translation2d robotToTargetTranslation =
          target.getTranslation().minus(robotPose.getTranslation());

      return PathPlanner.generatePath(
          new PathConstraints(2, 4),
          new PathPoint(
              robotPose.getTranslation(),
              getDirection(robotToTargetTranslation),
              robotPose.getRotation(),
              currentSpeed.getNorm()),
          new PathPoint(
              target.getTranslation(), getDirection(robotToTargetTranslation), target.getRotation()));
    }
  */

  /*
    public static Command followTrajectoryCommand(
        PathPlannerTrajectory trajectory, boolean isFirstPath, DriveSubsystem driveSubsystem) {
      return new SequentialCommandGroup(
          new InstantCommand(
              () -> {
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
              driveSubsystem));
    }
  */
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

  public static Pose2d getTransformedYAxisForAllianceColor(Pose2d pose) {
    if (AllianceColor.alliance == Alliance.Blue) {
      return pose;
    }

    Translation2d transformedTranslation =
        new Translation2d(pose.getX(), VisionConstants.FIELD_WIDTH_METERS - pose.getY());
    Rotation2d transformedHolonomicRotation = pose.getRotation().times(-1);

    return new Pose2d(transformedTranslation, transformedHolonomicRotation);
  }

  public static Rotation2d getTransformedHeadingForAllianceColor(Rotation2d rotation) {
    if (AllianceColor.alliance == Alliance.Blue) {
      return rotation;
    }

    return rotation.times(-1.0);
  }

  public static Pose2d getPoseByDistanceAndAngleToPose(
      Pose2d pose, double distance, Rotation2d angle) {
    return pose.transformBy(
        new Transform2d(new Translation2d(distance, angle).unaryMinus(), angle));
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
    return Math.abs((fromPose.getX() - toPose.getX()) + (fromPose.getY() - toPose.getY()));
  }

  public static Transform3d normalizeCameraAngle(Transform3d cameraToTarget) {
    double angle = Math.atan(cameraToTarget.getZ() / cameraToTarget.getX());
    double theta = -angle + Units.degreesToRadians(20);
    double hyp =
        Math.sqrt(
            (cameraToTarget.getX() * cameraToTarget.getX())
                + (cameraToTarget.getZ() * cameraToTarget.getZ()));
    double xPrime = hyp * Math.cos(theta);
    double zPrime = -hyp * Math.sin(theta);

    return new Transform3d(
        new Translation3d(xPrime, cameraToTarget.getY(), zPrime), cameraToTarget.getRotation());
  }
}
