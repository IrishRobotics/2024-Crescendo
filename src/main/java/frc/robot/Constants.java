// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int kDriverControllerPort = 0;
  public static final int kCoopControllerPort = 1;
  public static final int kDEBUGControllerPost = 2;

  public static class OpConstants {

    //Input
    public static final int GearButton = XboxController.Button.kStart.value;
  }

  public static class DriveConstants{

    // Drivetrain Constants
    public static final int kFrontLeftID = 1;
    public static final int kFrontRightID = 2;
    public static final int kRearLeftID = 3;
    public static final int kRearRightID = 4;
    public static final double kHighGear = 0.8;
    public static final double kLowGear = 0.4;
    public static final double kMaxSpeed = 0.8;

    //Auto PID Control
    public static final double minSpeed = .3;
    public static final double moveKP = 1;
    public static final double moveKI = 0.001;
    public static final double moveKD = 0;

  }

  public static class IntakeConstants{
    //Motors
    public static final int kIntakeID = 5;
    public static final double kIntakeSpeed = 0.6;

    //Sensors
    public static final int kNoteDetectorID = 0;

    //Input
    public static final int intakeButton = XboxController.Button.kRightBumper.value;
    public static final int ejectButton = XboxController.Axis.kRightTrigger.value;
  }
  
  public static class Shooter{
    //Motors
    public static final int kShooter1ID = 6;
    public static final double kShooter1RPM = 1000;
    public static final double kDrop1RMP = 100;
    public static final int kShooter2ID = 7;
    public static final double kShooter2RPM = 1000;
    public static final double kDrop2RMP = 100;

    //Input
    public static final int kShootButton = XboxController.Button.kLeftBumper.value;
    public static final int kDropButton = XboxController.Button.kRightBumper.value;
  }
  
  public static class Arm{
    //Motors
    public static final int kArmMotor1 = 8;

    //Encoders
    public static final int kAbsEncoder = 1;
    public static final double kEncoderOffset = 90/360;

    //Limit switches
    public static final int kMinLimit = 3;
    public static final int kMaxLimit = 4;

    //Positions
    public static final double kDrivePosition = 60;
    public static final double kPickupPosition = 6;
    public static final double kAmpPosition = 60;
  }

  public static class Lift{
    public static final int motorID = 9;
  }

  public static class DEBUG{
    public static final int intakeInButton = XboxController.Button.kA.value;
    public static final int intakeOutButton = XboxController.Button.kB.value;

    public static final int armJoystick = XboxController.Axis.kRightY.value;

    public static final int runShooterMotors = XboxController.Button.kX.value;
  }
}
