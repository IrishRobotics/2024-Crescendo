// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class OperatorDrive extends Command {
  private XboxController mController;
  private Drivetrain sDrive;
  private Boolean mFieldRelative;

  /** Creates a new OperatorDrive. */
  public OperatorDrive(Drivetrain drive, XboxController xboxController, boolean fieldRelative) {
    this.mController = xboxController;
    this.sDrive = drive;
    this.mFieldRelative = fieldRelative;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sDrive.drive(
        mController.getRightY(), mController.getRightX(), mController.getLeftX(), mFieldRelative);
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
