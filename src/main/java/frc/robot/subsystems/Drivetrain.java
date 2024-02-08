// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OpConstants;

public class Drivetrain extends SubsystemBase {
  private double speedValue = OpConstants.kHighGear;

  
  /** Creates a new Drivetrain. */
  public Drivetrain() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void ToggleGear() {
    if (speedValue == OpConstants.kHighGear) {
      speedValue = OpConstants.kLowGear;
      SmartDashboard.putBoolean("Gear", false);
    } else if (speedValue == OpConstants.kLowGear) {
      speedValue = OpConstants.kHighGear;
      SmartDashboard.putBoolean("Gear", true);
    }
  }


  // Commands
  public Command cmdToggleGear() {
    return this.runOnce(this::ToggleGear);
  }
}
