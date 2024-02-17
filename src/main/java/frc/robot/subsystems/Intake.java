// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  //Motors
  private WPI_TalonSRX intakeMotor = new WPI_TalonSRX(Constants.IntakeConstants.kIntakeID);

  //Sensors
  private DigitalInput noteDetector = new DigitalInput(Constants.IntakeConstants.kNoteDetectorID);

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Note Detector", NoteDetected());
    // This method will be called once per scheduler run
  }

  public void NoteIn(){
    intakeMotor.set(Constants.IntakeConstants.kIntakeSpeed);
  }

  public void Stop(){
    intakeMotor.set(0);
  }

  public void NoteOut(){
    intakeMotor.set(-Constants.IntakeConstants.kIntakeSpeed);
  }

  public Boolean NoteDetected(){
    return noteDetector.get();
  }
}
