package frc.robot.subsystems.pose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.fms.AllianceReadyListener;

public class PoseEstimatorSubsystem extends SubsystemBase implements AllianceReadyListener {
    private AprilTagFieldLayout layout;

   
        

    @Override
    public void updateAllianceColor(Alliance alliance) {
        // should this also re-initialize the pose estimators ?
        layout.setOrigin(
            alliance == Alliance.Blue
                ? OriginPosition.kBlueAllianceWallRightSide
                : OriginPosition.kRedAllianceWallRightSide);
      
        // TODO Auto-generated method stub
        
    }
      
    
}
