// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  //Motors
  private WPI_TalonSRX intakeMotor;
  //Sensors
  private DigitalInput noteDetector ;
  //Shuffleboard
  private ShuffleboardTab tab;
  private GenericEntry sMotorSpeed;
  private GenericEntry sNoteDetected;

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new WPI_TalonSRX(Constants.IntakeConstants.kIntakeID);
    intakeMotor.setNeutralMode(NeutralMode.Brake); 

    noteDetector = new DigitalInput(Constants.IntakeConstants.kNoteDetectorID);

    addChild("Motor", intakeMotor);
    addChild("Sensor", noteDetector);

    configureDashboard();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    sNoteDetected.setBoolean(NoteDetected());
  }
  public void set(double speed){
    intakeMotor.set(speed);
    sMotorSpeed.setDouble(speed);
  }

  public void NoteIn(){
    this.set(Constants.IntakeConstants.kIntakeSpeed);
  }

  public void Stop(){
    this.set(0);
  }

  public void NoteOut(){
    this.set(-Constants.IntakeConstants.kIntakeSpeed);
  }

  public Boolean NoteDetected(){
    return !noteDetector.get();
  }


  private void configureDashboard() {
    tab = Shuffleboard.getTab("Intake");
    sMotorSpeed = tab.add("Speed",0).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
    sNoteDetected = tab.add("Note Detected", this.NoteDetected()).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
  }
}
