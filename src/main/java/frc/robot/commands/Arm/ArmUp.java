// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmUp extends Command {
  private Arm sArm;
  /** Creates a new ArmUpo. */
  public ArmUp(Arm sArm) {
  this.sArm = sArm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sArm.Move(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sArm.Move(0.5);}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sArm.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
