// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Vision;

public class ArmShoot extends Command {
  private Arm sArm;
  private Vision sVision;

  /** Creates a new ArmShoot. */
  public ArmShoot(Arm arm, Vision vision) {
    sArm = arm;
    sVision = vision;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Transform3d tagPosition = sVision.TargetWithID(4).getBestCameraToTarget();

    double distanceToTag = Math.sqrt(Math.pow(tagPosition.getX(), 2)+Math.pow(tagPosition.getY(), 2));
    



    // sArm.setSetpoint(Constants.Arm.kShootPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return Math.abs(sArm.getMeasurement()-sArm.getSetpoint())<.5;
  }
}
