package frc.robot.fms;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.ArrayList;
import java.util.List;

public class AllianceColor {
  public static Alliance alliance = null;
  public static List<AllianceReadyListener> listeners = new ArrayList<AllianceReadyListener>();

  public static void addListener(AllianceReadyListener listener) {
    listeners.add(listener);
  }

  public static void setAllianceColor(Alliance newAlliance) {
    alliance = newAlliance;
    notifyListeners();
  }

  public static void notifyListeners() {
    for (AllianceReadyListener listener : listeners) {
      listener.updateAllianceColor(alliance);
    }
  }
}
