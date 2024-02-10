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
  public static final int kCoopControllerPort = 2;

  public static class OpConstants {
    // Drivetrain Constants
    public static final int kFrontLeftID = 1;
    public static final int kFrontRightID = 2;
    public static final int kRearLeftID = 3;
    public static final int kRearRightID = 4;
    public static final double kHighGear = 1;
    public static final double kLowGear = 0.6;
    public static final double kMaxSpeed = 0.8;

    //Input
    public static final int GearButton = XboxController.Button.kStart.value;
  }

  public static class IntakeConstants{
    //Motors
    public static final int kIntakeID = 5;
    public static final double kIntakeSpeed = 0.5;

    //Sensors
    public static final int kNoteDetectorID = 0;

    //Input
    public static final int intakeButton = XboxController.Button.kA.value;
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
    public static final int kShootButton = XboxController.Button.kB.value;
    public static final int kDropButton = XboxController.Button.kX.value;
  }
  
  public static class Arm{
    //Motors
    public static final int kArm1 = 8;
    public static final int kArm2 = 9;
  }
}
