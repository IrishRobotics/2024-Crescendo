// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  PhotonCamera camera = new PhotonCamera("frontcamera");
  
  /** Creates a new Vision. */
  public Vision() {}

  public PhotonPipelineResult rawResult;
  @Override
  public void periodic() {
    rawResult = camera.getLatestResult();
  }

  public PhotonTrackedTarget TargetWithID(int id){
    for (PhotonTrackedTarget element : rawResult.getTargets()) {
      if(element.getFiducialId()==id){
        return element;
      }
    }

    return null;
  }
}
