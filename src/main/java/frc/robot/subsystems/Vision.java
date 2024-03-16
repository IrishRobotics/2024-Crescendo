// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.proto.Wpimath;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  PhotonCamera camera = new PhotonCamera("shootingSideCamera");
  
  /** Creates a new Vision. */
  public Vision() {}

  public PhotonPipelineResult rawResult;
  @Override
  public void periodic() {
    rawResult = camera.getLatestResult();

    for (PhotonTrackedTarget target : rawResult.targets) {
        SmartDashboard.putString("Target: "+target.getFiducialId(), target.getBestCameraToTarget().toString());
        // SmartDashboard.putNumber("Target Distance: "+ target.getFiducialId(), GetDistanceToTargetFlat(target));
    }
  }

  public PhotonTrackedTarget TargetWithID(int id){
    for (PhotonTrackedTarget element : rawResult.getTargets()) {
      if(element.getFiducialId()==id){
        return element;
      }
    }

    return null;
  }

  // public double GetDistanceToTargetFlat(PhotonTrackedTarget target){
  //   Translation3d position = target.getBestCameraToTarget().getTranslation();
  //   double hyp = Math.sqrt(Math.pow(position.getX(),2)+Math.pow(position.getZ(),2)+Math.pow(position.getY(),2));
  //   hyp = position.getDistance(new Translation3d(0,0,0));
  //   SmartDashboard.putNumber("X ROT", Math.toDegrees(target.getBestCameraToTarget().getRotation().getX()));
  //   SmartDashboard.putNumber("Y ROT", Math.toDegrees(target.getBestCameraToTarget().getRotation().getY()));
  //   SmartDashboard.putNumber("Z ROT", Math.toDegrees(target.getBestCameraToTarget().getRotation().getZ()));
  //   return hyp*Math.cos(-target.getBestCameraToTarget().getRotation().getY());
  // }
}