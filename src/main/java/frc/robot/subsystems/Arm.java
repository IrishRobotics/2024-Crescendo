// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// public class Arm extends PIDSubsystem {
//   public DutyCycleEncoder angleSensor = new DutyCycleEncoder(1);
//   public WPI_TalonSRX mMotor = new WPI_TalonSRX(Constants.Arm.kArmMotor1);

//   /** Creates a new Arm. */
//   public Arm() {
//     super(
//       // The PIDController used by the subsystem
//       new PIDController(0.5, 0, 0));

//     mMotor.setNeutralMode(NeutralMode.Brake);

//     angleSensor.setPositionOffset((90)/360);
//   }

//   public boolean encoderReset = false;
  
//   @Override public void periodic() {
//     SmartDashboard.putString("Arm Position", String.valueOf(GetAngle()));
//   }

//   @Override
//   public void useOutput(double output, double setpoint) {
//     // Use the output here
//     mMotor1.set(output);
//   }

//   @Override
//   public double getMeasurement() {
//     // Return the process variable measurement here
//     return mEncoder.getDistance();
//   }

//   public double GetAngle(){
//     return (angleSensor.getAbsolutePosition()*360+10)%360;
//   }
// }

public class Arm extends SubsystemBase{
  public WPI_TalonSRX mMotor1 = new WPI_TalonSRX(Constants.Arm.kArmMotor1);
  public DutyCycleEncoder angleSensor = new DutyCycleEncoder(1);
  //private double position;

  /** Creates a new Arm. */
  public Arm() {
    mMotor1.setNeutralMode(NeutralMode.Brake);

    angleSensor.setPositionOffset((90)/360);
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
