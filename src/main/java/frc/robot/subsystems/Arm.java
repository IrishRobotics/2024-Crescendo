// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Arm.ArmReset;

public class Arm extends PIDSubsystem {
  public Encoder mEncoder = new Encoder(Constants.Arm.kEncoderPin1, Constants.Arm.kEncoderPin2);

  public WPI_TalonSRX mMotor1 = new WPI_TalonSRX(Constants.Arm.kArmMotor1);
  public WPI_TalonSRX mMotor2 = new WPI_TalonSRX(Constants.Arm.kArmMotor2);

  public DigitalInput mMinLimit = new DigitalInput(Constants.Arm.kMinLimit);
  public DigitalInput mMaxLimit = new DigitalInput(Constants.Arm.kMaxLimit);

  /** Creates a new Arm. */
  public Arm() {
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));

    mEncoder.setDistancePerPulse(.5);// TODO get value

    mMotor2.follow(mMotor1);
  }

  public boolean encoderReset = false;
  
  @Override public void periodic() {
    if(atMin()){
      mEncoder.reset();
      encoderReset = true;
    }
    if(atMax()){
      this.disable();
    }else{
      this.enable();
    }

    super.periodic();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    mMotor1.set(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return mEncoder.getDistance();
  }

  public void resetEncoder(){
    mEncoder.reset();
  }

  private boolean atMax(){
    return mMaxLimit.get();
  }

  private boolean atMin(){
    return mMinLimit.get();
  }
}

// public class Arm extends SubsystemBase{
//   public WPI_TalonSRX mMotor1 = new WPI_TalonSRX(Constants.Arm.kArmMotor1);
  
//   /** Creates a new Arm. */
//   public Arm() {
//     mMotor1.setNeutralMode(NeutralMode.Brake);
//   }

//   @Override
//   public void periodic() {

//   }

//   public void Move(double speed){
//     mMotor1.set(speed);
//   }

//   public void Stop(){
//     mMotor1.set(0);
//   }
// }
