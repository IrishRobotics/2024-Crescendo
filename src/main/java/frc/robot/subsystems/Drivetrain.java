// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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

  private RelativeEncoder mFrontLeftEncoder  = mFrontLeftMotor.getEncoder();
  private RelativeEncoder mFrontRightEncoder = mFrontRightMotor.getEncoder();
  private RelativeEncoder mRearLeftEncoder   = mRearLeftMotor.getEncoder();
  private RelativeEncoder mRearRightEncoder  = mRearRightMotor.getEncoder();

  private Translation2d frontLeftTranslate = new Translation2d(0.26, 0.26);
  private Translation2d frontRightTranslate = new Translation2d(0.26, -0.26);
  private Translation2d rearLeftTranslate = new Translation2d(-0.26, 0.26);
  private Translation2d rearRightTranslate = new Translation2d(-0.26, -0.26);

  private MecanumDriveKinematics kinematics =
    new MecanumDriveKinematics(
      frontLeftTranslate, frontRightTranslate, rearLeftTranslate, rearRightTranslate);

  private MecanumDriveOdometry odometry;
  public Pose2d robotPose;

  //Menanum Drive
  private MecanumDrive mMecanumDrive = new MecanumDrive(mFrontLeftMotor, mRearLeftMotor, mFrontRightMotor, mRearRightMotor);
  
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    mFrontLeftMotor.setInverted(true);
    mRearLeftMotor.setInverted(true);

    mFrontLeftMotor.setIdleMode(IdleMode.kBrake);
    mFrontRightMotor.setIdleMode(IdleMode.kBrake);
    mRearLeftMotor.setIdleMode(IdleMode.kBrake);
    mRearRightMotor.setIdleMode(IdleMode.kBrake);

    double countsPerRev = 42;
    double conversionFactor = 4 * 64 * Math.PI; //1:4 gearbox plus 8in wheels
    mFrontLeftEncoder.setPositionConversionFactor(conversionFactor/countsPerRev);
    mFrontRightEncoder.setPositionConversionFactor(conversionFactor/countsPerRev);
    mRearLeftEncoder.setPositionConversionFactor(conversionFactor/countsPerRev);
    mRearRightEncoder.setPositionConversionFactor(conversionFactor/countsPerRev);

    robotPose = new Pose2d(0.0, 0.0, new Rotation2d()); // Inital pose of the robot
    odometry = new MecanumDriveOdometry(kinematics, mNavx.getRotation2d(), new MecanumDriveWheelPositions(
      mFrontLeftMotor.getEncoder().getPosition(), mFrontRightMotor.getEncoder().getPosition(),
      mRearLeftMotor.getEncoder().getPosition(), mRearRightMotor.getEncoder().getPosition()
    ), robotPose);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Test motor movement", mRearRightMotor.getEncoder().getPosition());

    var wheelPositions = new MecanumDriveWheelPositions(
      mFrontLeftMotor.getEncoder().getPosition(), mFrontRightMotor.getEncoder().getPosition(),
      mRearLeftMotor.getEncoder().getPosition(), mRearRightMotor.getEncoder().getPosition()
    );
  
    // Get the rotation of the robot from the gyro.
    var gyroAngle = mNavx.getRotation2d();
  
    // Update the pose
    robotPose = odometry.update(gyroAngle, wheelPositions);
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
      mMecanumDrive.driveCartesian(x*Math.abs(x)*speedValue, y*Math.abs(y)*speedValue, turn*Math.abs(turn)*speedValue, mNavx.getRotation2d());
    }else{
      mMecanumDrive.driveCartesian(x*Math.abs(x)*speedValue, y*Math.abs(y)*speedValue, turn*Math.abs(turn)*speedValue);
    }
  }
}
