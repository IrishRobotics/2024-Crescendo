// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootNote extends Command {
  private Shooter sShooter;
  private Intake sIntake;

  /** Creates a new ShootNote. */
  public ShootNote(Shooter shooter, Intake intake) {
    sShooter = shooter;
    sIntake = intake;

    addRequirements(shooter, intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sShooter.EnableShooter(false);
    sIntake.NoteIn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { /* No update code to run. */}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sShooter.StopShooter();
    sIntake.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
