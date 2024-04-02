// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PositionShoot extends Command {
  private Drivetrain sDrivetrain;
  private Vision sVision;
  private double minSpeed = 0.25;

  /** Creates a new PositionShoot. */
  public PositionShoot(Drivetrain drivetrain, Vision vision) {
    this.sDrivetrain = drivetrain;
    this.sVision = vision;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //  if(shootingStatus != null) shootingStatus.setString("Aligning robot");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double movement = 0;
    PhotonTrackedTarget centerTag =
        (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
            ? sVision.TargetWithID(7)
            : sVision.TargetWithID(4); // 7 for blue 4 for red
    PhotonTrackedTarget sideTag =
        (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
            ? sVision.TargetWithID(8)
            : sVision.TargetWithID(3); // 8 for blue 3 for red

    if (centerTag != null) {
      movement = centerTag.getBestCameraToTarget().getY();
      if (Math.abs(movement) < minSpeed) {
        movement = minSpeed * Math.signum(movement);
      }
      sDrivetrain.drive(0, 0, movement, false);
    } else if (sideTag != null) {
      movement = sideTag.getBestCameraToTarget().getY();
      if (Math.abs(movement) < minSpeed) {
        movement = minSpeed * Math.signum(movement);
      }
    }
    sDrivetrain.drive(0, 0, -movement, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    PhotonTrackedTarget centerTag =
        (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
            ? sVision.TargetWithID(7)
            : sVision.TargetWithID(4); // 7 for blue 4 for red

    if (centerTag == null) {
      return false;
    }

    return Math.abs(centerTag.getBestCameraToTarget().getY()) < 0.1;
  }
}
