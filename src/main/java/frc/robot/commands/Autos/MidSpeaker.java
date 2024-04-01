// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Drivetrain.Move;
import frc.robot.commands.Groups.PickupNoteGroup;
import frc.robot.commands.Groups.ShootNoteGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MidSpeaker extends InstantCommand {
  private Command blueCommandSequence;
  private Command redCommandSequence;

  /** Creates a new MidSpeaker. */
  public MidSpeaker(Arm arm, Shooter shooter, Intake intake, Drivetrain drivetrain, Vision vision) {
    blueCommandSequence =
        Commands.sequence(
            new ShootNoteGroup(arm, shooter, intake, drivetrain, vision),
            new Move(new Pose2d(2, 1, new Rotation2d()), drivetrain)
                .alongWith(new PickupNoteGroup(arm, intake)));

    redCommandSequence =
        Commands.sequence(
            new ShootNoteGroup(arm, shooter, intake, drivetrain, vision),
            new Move(new Pose2d(2, 1, new Rotation2d()), drivetrain)
                .alongWith(new PickupNoteGroup(arm, intake)));
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
