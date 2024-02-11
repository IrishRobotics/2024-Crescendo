// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import javax.tools.Diagnostic;

import org.opencv.core.Mat;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class PositionShoot extends Command {
  private Drivetrain sDrivetrain;
  private Vision sVision;
  private GenericEntry shufflebordEntry;

  /** Creates a new PositionShoot. */
  public PositionShoot(Drivetrain drivetrain, Vision vision) {
    sDrivetrain = drivetrain;
    this.sVision = vision;

    shufflebordEntry = Shuffleboard.getTab("April Tags").add("Shooting auto rotate status", "Not running").getEntry();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonTrackedTarget centerTag = sVision.TargetWithID(4);
    PhotonTrackedTarget sideTag = sVision.TargetWithID(3);

    if(centerTag!=null){
      sDrivetrain.Drive(0, 0, (centerTag.getYaw()/10), false);
      shufflebordEntry.setString("Focusing on Center Tag");
    }else if(sideTag!=null){
      sDrivetrain.Drive(0, 0, (sideTag.getYaw()/10), false);
      shufflebordEntry.setString("Focusing on Side Tag");
    }else{
      sDrivetrain.Drive(0, 0, 1, false);   
      shufflebordEntry.setString("Failed to find Tags");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    PhotonTrackedTarget centerTag = sVision.TargetWithID(4);

    return Math.abs(centerTag.getYaw())<0.5;
  }
}
