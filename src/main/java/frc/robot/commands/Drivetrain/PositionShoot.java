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
  private GenericEntry rotationEntry;

  /** Creates a new PositionShoot. */
  public PositionShoot(Drivetrain drivetrain, Vision vision) {
    sDrivetrain = drivetrain;
    this.sVision = vision;

    rotationEntry = Shuffleboard.getTab("April Tags").add("Shooting rotation", 0).getEntry();
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

    }else if(sideTag!=null){

    }else{
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
