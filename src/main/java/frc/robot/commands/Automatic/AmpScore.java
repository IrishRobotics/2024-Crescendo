// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automatic;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpScore extends InstantCommand {
  private Command blueCommandSequence;
  private Command redCommandSequence;

  /**
   * @param position the starting position of the robot 0 closest to amp - 2 furthest away from amp 
   */
  public AmpScore(int position, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.

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
