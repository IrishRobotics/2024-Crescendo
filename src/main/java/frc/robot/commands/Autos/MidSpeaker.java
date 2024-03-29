// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Groups.ShootNoteGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MidSpeaker extends SequentialCommandGroup {
  /** Creates a new MidSpeaker. */
  public MidSpeaker(
      Arm arm,
      Shooter shooter,
      Intake intake,
      Drivetrain drivetrain,
      Vision vision,GenericEntry statusEntry) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ShootNoteGroup(arm,shooter,intake,drivetrain,vision,statusEntry),
      new MoveOut(drivetrain).alongWith(      intake.cmdAutoIn()));
  }
}
