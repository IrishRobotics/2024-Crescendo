// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import java.util.Map;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class PositionShoot extends Command {
  private Drivetrain sDrivetrain;
  private Vision sVision;

  GenericEntry positioningStatus;
  GenericEntry centerTagDeviation;
  GenericEntry shootingStatus;

  /** Creates a new PositionShoot. */
  public PositionShoot(Drivetrain drivetrain, Vision vision, GenericEntry shootingStatus) {
    this.shootingStatus = shootingStatus;
    sDrivetrain = drivetrain;
    this.sVision = vision;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    configureDashboard();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Shuffleboard.selectTab("Auto Shoot");
    shootingStatus.setString("Aligning robot");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonTrackedTarget centerTag = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? sVision.TargetWithID(7) : sVision.TargetWithID(4); // 7 for blue 4 for red
    PhotonTrackedTarget sideTag = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? sVision.TargetWithID(8) : sVision.TargetWithID(3); // 8 for blue 3 for red

    if(centerTag!=null){
      double movement = centerTag.getBestCameraToTarget().getY();
      if(Math.abs(movement)<.2){
        movement = .2*Math.signum(movement);
      }
      sDrivetrain.drive(0, 0, movement, false);
      positioningStatus.setString("Focusing on Center Tag");
    }else if(sideTag!=null){
      double movement = sideTag.getBestCameraToTarget().getY();
      if(Math.abs(movement)<.2){
        movement = .2*Math.signum(movement);
      }
      sDrivetrain.drive(0, 0, movement, false);
      positioningStatus.setString("Focusing on Side Tag");
    }else{
      sDrivetrain.drive(0, 0, 0.1, false);   
      positioningStatus.setString("Failed to find Tags");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    positioningStatus.setString("Done");
    centerTagDeviation.setDouble(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    PhotonTrackedTarget centerTag = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? sVision.TargetWithID(7) : sVision.TargetWithID(4); // 7 for blue 4 for red

    if(centerTag==null){
        return false;
    }

    centerTagDeviation.setDouble(centerTag.getBestCameraToTarget().getY());
    return Math.abs(centerTag.getBestCameraToTarget().getY())<0.1;
  }

  private void configureDashboard(){
    ShuffleboardTab tab = Shuffleboard.getTab("Auto Shoot");

    positioningStatus = tab.add("Shooting Positoning Status", "Disabled")
        .withWidget(BuiltInWidgets.kTextView)
        .withSize(2, 1)
        .getEntry();
    
    centerTagDeviation = tab.add("Center Tag Deviation", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withSize(2, 1)
        .withProperties(Map.of("min", -1, "max", 1))
        .getEntry();
  }
}
