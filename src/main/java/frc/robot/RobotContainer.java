// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.commands.Arm.MoveArm;
import frc.robot.commands.Autos.MidSpeaker;
import frc.robot.commands.Autos.MoveOut;
import frc.robot.commands.Drivetrain.OperatorDrive;
import frc.robot.commands.Drivetrain.PotatoAuto;
import frc.robot.commands.Groups.AmpGroup;
import frc.robot.commands.Groups.PickupNoteGroup;
import frc.robot.commands.Groups.ShootNoteGroup;
import frc.robot.commands.Shooter.ShootNote;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private Drivetrain sDrivetrain;
  private Intake sIntake;
  private Shooter sShooter;
  private Arm sArm;
  private Vision sVision;
  private Lift sLift;

  // Joysticks
  private XboxController mOpController;
  private XboxController mCoopController;

  //// Joystick Buttons
  /// Driver
  private Trigger toggleGearBtn;
  private Trigger liftUpTrigger;
  private Trigger liftDownTrigger;
  /// Coop
  // Groups
  private Trigger intakeNoteTrigger;
  private Trigger shootNoteTrigger;
  private Trigger ampTrigger;
  // Intake
  private Trigger ejectNodeTrigger;
  // Shooter
  private Trigger overrideShootNodeTrigger;
  // Arm
  private Trigger armUpTrigger;
  private Trigger armDownTrigger;

  // Auto Chooser
  private SendableChooser<Command> autoSelect;

  // Shuffleboard
  private ShuffleboardTab driveTab;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Init Joysticks
    mOpController = new XboxController(Constants.kDriverControllerPort);
    mCoopController = new XboxController(Constants.kCoopControllerPort);

    // INit subsystems
    sVision = new Vision();
    sDrivetrain = new Drivetrain();
    sIntake = new Intake();
    sShooter = new Shooter();
    sArm = new Arm();
    sLift = new Lift();

    // Default Commands
    sDrivetrain.setDefaultCommand(new OperatorDrive(sDrivetrain, mOpController, false));

    // Setup auto selector
    autoSelect = new SendableChooser<Command>();
    autoSelect.setDefaultOption("Nothing", null);
    autoSelect.addOption("Potato Move", new PotatoAuto(sDrivetrain));
    autoSelect.addOption("Move Out", new MoveOut(sDrivetrain));
    autoSelect.addOption("Speaker Mid", new MidSpeaker(sArm, sShooter, sIntake, sDrivetrain, sVision));

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // shuffleboard
    driveTab = Shuffleboard.getTab("Driver");
    driveTab.add(autoSelect);

    /// Op
    toggleGearBtn = new JoystickButton(mOpController, OpConstants.kGearButton);
    toggleGearBtn.onTrue(sDrivetrain.cmdToggleGear());

    liftUpTrigger = new JoystickButton(mOpController, OpConstants.kLiftDownButton);
    liftUpTrigger.whileTrue(sLift.cmdUp());

    liftDownTrigger = new JoystickButton(mOpController, OpConstants.kLiftUpButton);
    liftDownTrigger.whileTrue(sLift.cmdDown());

    /// Coop
    intakeNoteTrigger = new JoystickButton(mCoopController, OpConstants.kIntakeButton);
    intakeNoteTrigger.whileTrue(new PickupNoteGroup(sArm, sIntake));
    intakeNoteTrigger.onFalse(new MoveArm(sArm, ArmConstants.kDrivePosition));

    shootNoteTrigger = new JoystickButton(mCoopController, OpConstants.kShootButton);
    shootNoteTrigger.whileTrue(new ShootNoteGroup(sArm, sShooter, sIntake, sDrivetrain, sVision));

    ejectNodeTrigger =
        new Trigger(
            () -> {
              return mCoopController.getRawAxis(OpConstants.kEjectButton) >= 0.01;
            });
    ejectNodeTrigger.whileTrue(sIntake.cmdOut());

    ampTrigger = new JoystickButton(mCoopController, OpConstants.kAmpButton);
    ampTrigger.whileTrue(new AmpGroup(sArm, sShooter, sIntake));
    ampTrigger.onFalse(new MoveArm(sArm, ArmConstants.kDrivePosition));

    // Override
    overrideShootNodeTrigger =
        new Trigger(
            () -> {
              return mCoopController.getRawAxis(OpConstants.overrideShootButton) >= 0.01;
            });
    overrideShootNodeTrigger.whileTrue(new ShootNote(sShooter, sIntake));

    armUpTrigger =
        new Trigger(
            () -> {
              return mCoopController.getPOV() == 0;
            });
    armUpTrigger.whileTrue(sArm.cmdUp());

    armDownTrigger =
        new Trigger(
            () -> {
              return mCoopController.getPOV() == 180;
            });
    armDownTrigger.whileTrue(sArm.cmdDown());
  }

  // Getter Methods

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSelect.getSelected();
  }

  public Drivetrain getDrivetrain() {
    return sDrivetrain;
  }

  public Intake getIntake() {
    return sIntake;
  }

  public Shooter getShooter() {
    return sShooter;
  }

  public Arm getArm() {
    return sArm;
  }

  public XboxController getOpController() {
    return mOpController;
  }
}
