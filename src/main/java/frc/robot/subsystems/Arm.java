// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase{
  private WPI_TalonSRX mMotor1;
  private DutyCycleEncoder angleSensor;
  //private double position;

  /** Creates a new Arm. */
  public Arm() {
    mMotor1 = new WPI_TalonSRX(Constants.Arm.kArmMotor1);
    mMotor1.setNeutralMode(NeutralMode.Brake);

    angleSensor = new DutyCycleEncoder(Constants.Arm.kAbsEncoder);
    angleSensor.setPositionOffset(Constants.Arm.kEncoderOffset);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Arm Position", String.valueOf(GetAngle()));
  }

  public void Move(double speed){
    mMotor1.set(speed);
  }

  public void Stop(){
    mMotor1.set(0);
  }

  public double GetAngle(){
    return (angleSensor.getAbsolutePosition()*360+2)%360;
  }
}
