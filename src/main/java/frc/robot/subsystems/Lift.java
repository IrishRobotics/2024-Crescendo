// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.fasterxml.jackson.databind.introspect.ConcreteBeanPropertyBase;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lift extends SubsystemBase {
  WPI_TalonSRX motor = new WPI_TalonSRX(Constants.Lift.motorID);

  /** Creates a new Lifter. */
  public Lift() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Down(){
    motor.set(-1);
  }

  public void Up(){
    motor.set(1);
  }

  public void Stop(){
    motor.set(0);
  }
}
