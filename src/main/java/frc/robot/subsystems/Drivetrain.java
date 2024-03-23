// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  // Parameters
  private double speedValue;
  double countsPerRev = 42;

  // Motors
  private CANSparkMax mFrontLeftMotor;
  private CANSparkMax mFrontRightMotor;
  private CANSparkMax mRearLeftMotor;
  private CANSparkMax mRearRightMotor;

  // Menanum Drive
  private MecanumDrive mMecanumDrive;

  // IMU
  private AHRS mNavx;

  // Kinematics
  private Translation2d frontLeftTranslate;
  private Translation2d frontRightTranslate;
  private Translation2d rearLeftTranslate;
  private Translation2d rearRightTranslate;
  private MecanumDriveKinematics kinematics;
  private MecanumDriveOdometry odometry;
  private Pose2d robotPose;
  private Field2d field;

  // Shuffleboard

  private ShuffleboardTab tab;
  private GenericEntry sFrontLeftMotorDistances;
  private GenericEntry sFrontRightMotorDistances;
  private GenericEntry sRearLeftMotorDistances;
  private GenericEntry sRearRightMotorDistances;
  private GenericEntry sPoseX;
  private GenericEntry sPoseY;
  private GenericEntry sForwardSpeed;
  private GenericEntry sStrafeSpeed;
  private GenericEntry sTurnSpeed;
  private GenericEntry sField;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    speedValue = DriveConstants.kHighGear;

    // Init hardware devices
    mFrontLeftMotor = new CANSparkMax(Constants.DriveConstants.kFrontLeftID, MotorType.kBrushless);
    mFrontRightMotor = new CANSparkMax(Constants.DriveConstants.kFrontRightID, MotorType.kBrushless);
    mRearLeftMotor = new CANSparkMax(Constants.DriveConstants.kRearLeftID, MotorType.kBrushless);
    mRearRightMotor = new CANSparkMax(Constants.DriveConstants.kRearRightID, MotorType.kBrushless);
    mMecanumDrive = new MecanumDrive(mFrontLeftMotor, mRearLeftMotor, mFrontRightMotor, mRearRightMotor);
    mNavx = new AHRS();
    this.addChild("Drivetrain", mMecanumDrive);
    this.addChild("NavX", mNavx);

    // Kinematics variables
    frontLeftTranslate = new Translation2d(0.26, 0.26);
    frontRightTranslate = new Translation2d(0.26, -0.26);
    rearLeftTranslate = new Translation2d(-0.26, 0.26);
    rearRightTranslate = new Translation2d(-0.26, -0.26);
    kinematics = new MecanumDriveKinematics(frontLeftTranslate, frontRightTranslate,
        rearLeftTranslate, rearRightTranslate);

    // Set motor parameters
    mFrontLeftMotor.setInverted(true);
    mFrontRightMotor.setInverted(false);
    mRearLeftMotor.setInverted(true);
    mRearRightMotor.setInverted(false);

    mFrontLeftMotor.setIdleMode(IdleMode.kBrake);
    mFrontRightMotor.setIdleMode(IdleMode.kBrake);
    mRearLeftMotor.setIdleMode(IdleMode.kBrake);
    mRearRightMotor.setIdleMode(IdleMode.kBrake);

    mFrontLeftMotor.getEncoder().setPositionConversionFactor(1);
    mFrontRightMotor.getEncoder().setPositionConversionFactor(1);
    mRearLeftMotor.getEncoder().setPositionConversionFactor(1);
    mRearRightMotor.getEncoder().setPositionConversionFactor(1);

    field = new Field2d();
    robotPose = new Pose2d(0.0, 0.0, mNavx.getRotation2d()); // Inital pose of the robot
    odometry = new MecanumDriveOdometry(kinematics, mNavx.getRotation2d(), new MecanumDriveWheelPositions(
        getWheelDistance(mFrontLeftMotor), getWheelDistance(mFrontRightMotor),
        getWheelDistance(mRearLeftMotor), getWheelDistance(mRearRightMotor)), robotPose);

    speedValue = DriveConstants.kHighGear;
    toggleGear();

    configureDashboard();
  }

  @Override
  public void periodic() {
    // get the updated distance the wheels have moved
    MecanumDriveWheelPositions wheelPositions = new MecanumDriveWheelPositions(
        getWheelDistance(mFrontLeftMotor), getWheelDistance(mFrontRightMotor),
        getWheelDistance(mRearLeftMotor), getWheelDistance(mRearRightMotor));

    // Get the rotation of the robot from the gyro.
    Rotation2d gyroAngle = new Rotation2d(mNavx.getYaw());

    // Update the pose
    robotPose = odometry.update(gyroAngle, wheelPositions);
    field.setRobotPose(odometry.getPoseMeters());

    // Update Shuffleboard
    sFrontLeftMotorDistances.setDouble(getWheelDistance(mFrontLeftMotor));
    sFrontRightMotorDistances.setDouble(getWheelDistance(mFrontRightMotor));
    sRearLeftMotorDistances.setDouble(getWheelDistance(mRearLeftMotor));
    sRearRightMotorDistances.setDouble(getWheelDistance(mRearRightMotor));
    sPoseX.setDouble(robotPose.getX());
    sPoseY.setDouble(robotPose.getY());
    SmartDashboard.putNumber("Gyro", mNavx.getRotation2d().getDegrees());

    SmartDashboard.updateValues();
  }

  public void toggleGear() {
    if (speedValue == DriveConstants.kHighGear) {
      speedValue = DriveConstants.kLowGear;
      SmartDashboard.putBoolean("Gear", false);
    } else if (speedValue == DriveConstants.kLowGear) {
      speedValue = DriveConstants.kHighGear;
      SmartDashboard.putBoolean("Gear", true);
    }
  }

  public void drive(double x, double y, double turn, boolean fieldRelitave) {
    double _x = x * Math.abs(x) * speedValue;
    double _y = y * Math.abs(y) * speedValue;
    double _z = turn * Math.abs(turn) * speedValue;
    if (fieldRelitave) {
      mMecanumDrive.driveCartesian(_x, _y, _z, mNavx.getRotation2d());
    } else {
      mMecanumDrive.driveCartesian(_x, _y, _z);
    }

    // Update shuffleboard
    sForwardSpeed.setDouble(_y);
    sStrafeSpeed.setDouble(_x);
    sTurnSpeed.setDouble(_z);
  }

  public void stop() {
    drive(0, 0, 0, false);
  }

  public double getWheelDistance(CANSparkMax motor) {
    double rawVal = motor.getEncoder().getPosition();
    return -(rawVal) / 8.45 * 0.2032 * Math.PI;
  }

  public Pose2d getPose() {
    return robotPose;
  }

  // Commands
  public Command cmdToggleGear() {
    return this.runOnce(this::toggleGear);
  }


  private void configureDashboard() {
    tab = Shuffleboard.getTab("Drivetrain");
    tab.add("Drivetrain", mMecanumDrive);
    tab.add("NavX", mNavx.getRotation2d().getDegrees());
    sFrontLeftMotorDistances = tab.add("Front Left(m)", getWheelDistance(mFrontLeftMotor)).getEntry();
    sFrontRightMotorDistances = tab.add("Front Right(m)", getWheelDistance(mFrontRightMotor)).getEntry();
    sRearLeftMotorDistances = tab.add("Rear Left(m)", getWheelDistance(mRearLeftMotor)).getEntry();
    sRearRightMotorDistances = tab.add("Rear Right(m)", getWheelDistance(mRearRightMotor)).getEntry();
    sForwardSpeed = tab.add("Y Speed", 0).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
    sStrafeSpeed = tab.add("X Speed", 0).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
    sTurnSpeed = tab.add("Z Speed", 0).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
    sPoseX = tab.add("Pose X (m)",robotPose.getX()).getEntry();
    sPoseY = tab.add("Pose Y (m)",robotPose.getY()).getEntry();

    SmartDashboard.putData(field);
  }
}
