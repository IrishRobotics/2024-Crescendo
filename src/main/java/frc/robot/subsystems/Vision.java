// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  private PhotonPipelineResult rawResult;
  private PhotonCamera camera;

  // Shuffleboard
  private ShuffleboardTab tab;
  private ShuffleboardTab driveTab;
  private ShuffleboardLayout sTargetIds;
  private GenericEntry sDistance;

  /** Creates a new Vision. */
  public Vision() {
    camera = new PhotonCamera("shootingSideCamera");
    configureDashboard();
  }

  @Override
  public void periodic() {
    rawResult = camera.getLatestResult();
    sTargetIds.getComponents().clear();
    for (PhotonTrackedTarget target : rawResult.targets) {
      // sTargetIds.add(
      //     "Target: " + target.getFiducialId(), getPoseString(target.getBestCameraToTarget()));

      // Get target distance
      if ((DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
              && target.getFiducialId() == 7)
          || (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
              && target.getFiducialId() == 4)) {
        double distance = getTargetDistance(target);
        sDistance.setDouble(distance);
      }
    }
  }

  public PhotonTrackedTarget TargetWithID(int id) {
    for (PhotonTrackedTarget element : rawResult.getTargets()) {
      if (element.getFiducialId() == id) {
        return element;
      }
    }

    return null;
  }

  public double getTargetDistance(PhotonTrackedTarget target) {
    double distanceRaw =
        target.getBestCameraToTarget().getTranslation().getDistance(new Translation3d()) * 3.28084;
    double distance = (49.0 / 12) / Math.tan(Math.asin((49.0 / 12) / distanceRaw));
    // distance -= 0.5; // Offset
    return distance;
  }

  private String getPoseString(Transform3d pose) {
    return String.format("%1$.2f, %2$.2f, %3$.2f", pose.getX(), pose.getY(), pose.getZ());
  }

  private void configureDashboard() {
    tab = Shuffleboard.getTab("Shooter");
    driveTab = Shuffleboard.getTab("Driver");
    sTargetIds = tab.getLayout("TargetIDs", BuiltInLayouts.kList).withSize(1, 3);
    sDistance = tab.add("Target Distance", 0).getEntry();
  }
}
