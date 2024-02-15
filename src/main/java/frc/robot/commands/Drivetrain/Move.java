// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class Move extends Command {
  private Translation2d startPosition;
  private Drivetrain sDrivetrain;
  private Translation2d movement;

  /** Creates a new Move. */
  public Move(double x, double y, Drivetrain drivetrain) {
    sDrivetrain = drivetrain;
    movement = new Translation2d(x, y);
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPosition = sDrivetrain.robotPose.getTranslation();
    sDrivetrain.Drive(movement.minus(sDrivetrain.robotPose.getTranslation().minus(startPosition)).getAngle().getCos(), movement.minus(sDrivetrain.robotPose.getTranslation().minus(startPosition)).getAngle().getSin(), 0, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sDrivetrain.Drive(movement.minus(sDrivetrain.robotPose.getTranslation().minus(startPosition)).getAngle().getCos(), movement.minus(sDrivetrain.robotPose.getTranslation().minus(startPosition)).getAngle().getSin(), 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sDrivetrain.Drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return sDrivetrain.robotPose.getTranslation().getDistance(movement)<.1;
  }
}
