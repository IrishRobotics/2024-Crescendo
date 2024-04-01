// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import java.util.Map;

public class Shooter extends SubsystemBase {
  private CANSparkMax mShooterMotor1;
  private CANSparkMax mShooterMotor2;
  private SparkPIDController m_pidController1;
  private SparkPIDController m_pidController2;

  // Shuffleboard
  private ShuffleboardTab tab;
  private ShuffleboardTab driveTab;
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

    m_pidController1.setP(ShooterConstants.kP);
    m_pidController1.setI(ShooterConstants.kI);
    m_pidController1.setD(ShooterConstants.kD);
    m_pidController1.setOutputRange(ShooterConstants.kMinRpm, ShooterConstants.kMaxRpm);

    m_pidController2.setP(ShooterConstants.kP);
    m_pidController2.setI(ShooterConstants.kI);
    m_pidController2.setD(ShooterConstants.kD);
    m_pidController2.setOutputRange(ShooterConstants.kMinRpm, ShooterConstants.kMaxRpm);

    configureDashboard();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    sMotor1Speed.setDouble(mShooterMotor1.getEncoder().getVelocity());
    sMotor2Speed.setDouble(mShooterMotor2.getEncoder().getVelocity());
  }

  public void setSpeed(double speed) {
    mShooterMotor1.set(speed);
    mShooterMotor2.set(speed);
  }

  public void enableShooter(boolean drop) {
    if (drop) {
      m_pidController1.setReference(ShooterConstants.kDropRMP, ControlType.kVelocity);
      m_pidController2.setReference(ShooterConstants.kDropRMP, ControlType.kVelocity);
    } else {
      m_pidController1.setReference(ShooterConstants.kShootRPM, ControlType.kVelocity);
      m_pidController2.setReference(ShooterConstants.kShootRPM, ControlType.kVelocity);
    }
  }

  public void stop() {
    mShooterMotor1.stopMotor();
    mShooterMotor2.stopMotor();
  }

  public double getSpeed() {
    return (mShooterMotor1.getEncoder().getVelocity() + mShooterMotor2.getEncoder().getVelocity())
        / 2;
  }

  public Command cmdRun(double speed) {
    return this.runEnd(() -> this.setSpeed(speed), this::stop);
  }

  private void configureDashboard() {
    tab = Shuffleboard.getTab("Shooter");
    driveTab = Shuffleboard.getTab("Driver");
    sMotor1Speed =
        tab.add("Motor 1", 0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", 0, "max", ShooterConstants.kMaxRpm))
            .getEntry();
    sMotor2Speed =
        tab.add("Motor 2", 0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", 0, "max", ShooterConstants.kMaxRpm))
            .getEntry();
  }
}
