// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OpConstants;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private double speedValue = OpConstants.kHighGear;
  private AHRS mNavx = new AHRS();

  //Motors
  private CANSparkMax mFrontLeftMotor  = new CANSparkMax(Constants.OpConstants.kFrontLeftID, MotorType.kBrushless);
  private CANSparkMax mFrontRightMotor = new CANSparkMax(Constants.OpConstants.kFrontRightID, MotorType.kBrushless);
  private CANSparkMax mRearLeftMotor   = new CANSparkMax(Constants.OpConstants.kRearLeftID, MotorType.kBrushless);
  private CANSparkMax mRearRightMotor  = new CANSparkMax(Constants.OpConstants.kRearRightID, MotorType.kBrushless);

  //Menanum Drive
  private MecanumDrive mMecanumDrive = new MecanumDrive(mFrontLeftMotor, mRearLeftMotor, mFrontRightMotor, mRearRightMotor);
  
  /** Creates a new Drivetrain. */
  public Drivetrain() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void ToggleGear() {
    if (speedValue == OpConstants.kHighGear) {
      speedValue = OpConstants.kLowGear;
      SmartDashboard.putBoolean("Gear", false);
    } else if (speedValue == OpConstants.kLowGear) {
      speedValue = OpConstants.kHighGear;
      SmartDashboard.putBoolean("Gear", true);
    }
  }

  // Commands
  public Command cmdToggleGear() {
    return this.runOnce(this::ToggleGear);
  }

  public void Drive(double x, double y, double turn, boolean fieldRelitave){
    if(fieldRelitave){
      mMecanumDrive.driveCartesian(x*speedValue, y*speedValue, turn*speedValue, mNavx.getRotation2d());
    }else{
      mMecanumDrive.driveCartesian(x*speedValue, y*speedValue, turn*speedValue);
    }
  }
}
