// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeNote extends Command {
  private Intake sIntake;

  /** Creates a new IntakeNote. */
  public IntakeNote(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    sIntake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sIntake.NoteIn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sIntake.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return sIntake.NoteDetected();
  }
}
