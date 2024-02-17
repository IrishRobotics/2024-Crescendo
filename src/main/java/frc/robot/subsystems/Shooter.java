// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private CANSparkMax mShooterMotor1 = new CANSparkMax(Constants.Shooter.kShooter1ID, MotorType.kBrushless);
  private CANSparkMax mShooterMotor2 = new CANSparkMax(Constants.Shooter.kShooter2ID, MotorType.kBrushless);
  // private SparkPIDController m_pidController1;
  // private SparkPIDController m_pidController2;

  /** Creates a new Shooter. */
  public Shooter() {
    // m_pidController1 = mShooterMotor1.getPIDController();
    // m_pidController2 = mShooterMotor2.getPIDController();

    mShooterMotor1.setInverted(false);
    mShooterMotor2.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void RunMotors(double speed){
    mShooterMotor1.set(speed);
    mShooterMotor2.set(speed);
  }

  public void EnableShooter(boolean drop){
    // m_pidController1.setReference(drop?Constants.Shooter.kShooter1RPM:Constants.Shooter.kDrop1RMP, CANSparkMax.ControlType.kVelocity);
    // m_pidController2.setReference(drop?Constants.Shooter.kShooter2RPM:Constants.Shooter.kDrop2RMP, CANSparkMax.ControlType.kVelocity);
  }

  public void StopShooter(){
    mShooterMotor1.stopMotor();
    mShooterMotor2.stopMotor();
  }
}
