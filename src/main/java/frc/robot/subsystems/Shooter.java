// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import java.util.Map;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private CANSparkMax mShooterMotor1 ;
  private CANSparkMax mShooterMotor2 ;
  private SparkPIDController m_pidController1;
  private SparkPIDController m_pidController2;

  // Shuffleboard
  private ShuffleboardTab tab;
  private GenericEntry sMotor1Speed;
  private GenericEntry sMotor2Speed;

  /** Creates a new Shooter. */
  public Shooter() {
    mShooterMotor1 = new CANSparkMax(ShooterConstants.kShooter1ID, MotorType.kBrushless);
    mShooterMotor2 = new CANSparkMax(ShooterConstants.kShooter2ID, MotorType.kBrushless);

    mShooterMotor1.setInverted(false);
    mShooterMotor2.setInverted(false);

    m_pidController1 = mShooterMotor1.getPIDController();
    m_pidController2 = mShooterMotor2.getPIDController();

    m_pidController1.setP(0.0002);
    m_pidController1.setI(0.000001);
    m_pidController1.setD(0.0001);
    m_pidController1.setOutputRange(-6000, 6000);

    m_pidController2.setP(0.0002);
    m_pidController2.setI(0.000001);
    m_pidController2.setD(0.000001);
    m_pidController1.setOutputRange(-6000, 6000);

    configureDashboard();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    sMotor1Speed.setDouble(mShooterMotor1.getEncoder().getVelocity());
    sMotor2Speed.setDouble(mShooterMotor2.getEncoder().getVelocity());
  }

  public void setSpeed(double speed){
    mShooterMotor1.set(speed);
    mShooterMotor2.set(speed);
  }

  public void enableShooter(boolean drop){
    if(drop){
      m_pidController1.setReference(ShooterConstants.kDropRMP, ControlType.kVelocity);
      m_pidController2.setReference(ShooterConstants.kDropRMP, ControlType.kVelocity);
    }else{
      m_pidController1.setReference(ShooterConstants.kShootRPM,ControlType.kVelocity);
      m_pidController2.setReference(ShooterConstants.kShootRPM,ControlType.kVelocity);
    }
  }

  public void stop(){
    mShooterMotor1.stopMotor();
    mShooterMotor2.stopMotor();
  }

  public double getSpeed() {
    return (mShooterMotor1.getEncoder().getVelocity()+mShooterMotor2.getEncoder().getVelocity())/2;
  }


  private void configureDashboard() {
    tab = Shuffleboard.getTab("Shooter");
    sMotor1Speed = tab.add("Motor 1",0)
          .withWidget(BuiltInWidgets.kDial)
          .withProperties(Map.of("min", 0, "max", 6000)).getEntry();
    sMotor2Speed = tab.add("Motor 2",0)
          .withWidget(BuiltInWidgets.kDial)
          .withProperties(Map.of("min", 0, "max", 6000)).getEntry();
  }
}
