// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class MoveArm extends Command {
private Arm sArm;
private double position;

  /** Creates a new DefaultArm. */
  public MoveArm(Arm arm, double position) {
    this.sArm = arm;
    this.position = position;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // sArm.setSetpoint(position);
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
