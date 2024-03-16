// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import javax.tools.Diagnostic;

import org.opencv.core.Algorithm;
import org.opencv.core.Mat;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.ADXL345_I2C.AllAxes;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class PositionShoot extends Command {
  private Drivetrain sDrivetrain;
  private Vision sVision;

  /** Creates a new PositionShoot. */
  public PositionShoot(Drivetrain drivetrain, Vision vision) {
    sDrivetrain = drivetrain;
    this.sVision = vision;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("Shooting status", "Aligning robot");
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
      sDrivetrain.Drive(0, 0, movement, false);
      SmartDashboard.putString("Shooting Positioning","Focusing on Center Tag");
    }else if(sideTag!=null){
      double movement = sideTag.getBestCameraToTarget().getY();
      if(Math.abs(movement)<.2){
        movement = .2*Math.signum(movement);
      }
      sDrivetrain.Drive(0, 0, movement, false);
      SmartDashboard.putString("Shooting Positioning","Focusing on Side Tag");
    }else{
      sDrivetrain.Drive(0, 0, 0.1, false);   
      SmartDashboard.putString("Shooting Positioning","Failed to find Tags");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Shooting status", "Aligning arm");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    PhotonTrackedTarget centerTag = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? sVision.TargetWithID(7) : sVision.TargetWithID(4); // 7 for blue 4 for red

    if(centerTag==null){
        return false;
    }

    SmartDashboard.putNumber("Center tag deviation", Math.abs(centerTag.getBestCameraToTarget().getY()));
    return Math.abs(centerTag.getBestCameraToTarget().getY())<0.1;
  }
}
