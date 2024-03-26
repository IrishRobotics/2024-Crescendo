// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Drivetrain.Move;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveOut extends InstantCommand {
  private Command blueCommandSequence;
  private Command redCommandSequence;

  public MoveOut(Drivetrain drivetrain) {
    blueCommandSequence = Commands.sequence(new Move(new Pose2d(0, 1, new Rotation2d()), drivetrain));

    redCommandSequence = Commands.sequence(new Move(new Pose2d(0, 1, new Rotation2d()), drivetrain));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if ((DriverStation.getAlliance().orElse(Alliance.Blue)) == Alliance.Blue) {
      blueCommandSequence.schedule();
    } else {
      redCommandSequence.schedule();
    }
  }
}
