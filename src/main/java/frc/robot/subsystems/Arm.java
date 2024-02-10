// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private Encoder encoder = new Encoder(0, 0);

  private WPI_TalonSRX motor1 = new WPI_TalonSRX(Constants.Arm.kArm1);
  private WPI_TalonSRX motor2 = new WPI_TalonSRX(Constants.Arm.kArm2);

  /** Creates a new Arm. */
  public Arm() { }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotorPower(double speed){
    motor1.set(speed);
    motor2.set(speed);
  }

  public void stop(){
    motor1.set(0);
    motor2.set(0);
  }

  public Double getEncoder(){
    return 
  }
}
