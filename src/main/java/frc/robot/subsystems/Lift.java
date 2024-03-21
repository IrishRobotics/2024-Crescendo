// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.fasterxml.jackson.databind.introspect.ConcreteBeanPropertyBase;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lift extends SubsystemBase {
  WPI_TalonSRX motor;
  //Shuffleboard
  private ShuffleboardTab tab;
  private GenericEntry sMotorSpeed;

  /** Creates a new Lifter. */
  public Lift() {
    motor = new WPI_TalonSRX(Constants.Lift.motorID);

    addChild("Motor", motor);
     configureDashboard();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Down(){
    motor.set(-1);
    sMotorSpeed.setDouble(motor.get());
  }

  public void Up(){
    motor.set(1);
    sMotorSpeed.setDouble(motor.get());
  }

  public void Stop(){
    motor.set(0);
    sMotorSpeed.setDouble(motor.get());
  }

  private void configureDashboard() {
    tab = Shuffleboard.getTab("Lift");
    sMotorSpeed = tab.add("Speed",0).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
  }
}
