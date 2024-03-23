// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Lift;
import frc.robot.commands.Arm.ArmDown;
import frc.robot.commands.Arm.VisionAim;
import frc.robot.commands.Arm.ArmUp;
import frc.robot.commands.Arm.MoveArm;
import frc.robot.commands.Automatic.MoveOut;
import frc.robot.commands.Drivetrain.DumbMove;
import frc.robot.commands.Drivetrain.OperatorDrive;
import frc.robot.commands.Drivetrain.PotatoAuto;
import frc.robot.commands.Groups.AmpGroup;
import frc.robot.commands.Groups.PickupNoteGroup;
import frc.robot.commands.Groups.ShootNoteGroup;
import frc.robot.commands.Intake.IntakeIn;
import frc.robot.commands.Intake.IntakeNote;
import frc.robot.commands.Intake.IntakeOut;
import frc.robot.commands.Lift.LiftDown;
import frc.robot.commands.Lift.LiftUp;
import frc.robot.commands.Shooter.RunShooterMotors;
import frc.robot.commands.Shooter.ShootNote;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  // Subsystems
  private Drivetrain sDrivetrain = new Drivetrain();
  private Intake sIntake = new Intake();
  private Shooter sShooter = new Shooter();
  private Arm sArm  = new Arm();
  private Vision sVision = new Vision();
  private Lift sLift = new Lift();

  // Joysticks
  private XboxController mOpController = new XboxController(Constants.kDriverControllerPort);
  private XboxController mCoopController = new XboxController(Constants.kCoopControllerPort);

  //// Joystick Buttons
  /// Driver
  private Trigger toggleGearBtn;
  private Trigger liftUpTrigger;
  private Trigger liftDownTrigger;
  /// Coop
  //Groups
  private Trigger intakeNoteTrigger;
  private Trigger shootNoteTrigger;
  private Trigger ampTrigger;
  //Intake
  private Trigger ejectNodeTrigger;
  //Shooter
  private Trigger overrideShootNodeTrigger;
  //Arm
  private Trigger armUpTrigger;
  private Trigger armDownTrigger;

  // Auto Chooser
  SendableChooser<Command> autoSelect = new SendableChooser<>();
  
  // Autonomous
  // TODO - Add auto commands


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Default Commands
    sDrivetrain.setDefaultCommand(new OperatorDrive(sDrivetrain, mOpController, false));
    //sArm.setDefaultCommand(new MoveArm(sArm, Constants.Arm.kDrivePosition));

    // autoSelect.setDefaultOption("Move out", new MoveOut(sDrivetrain));
    autoSelect.setDefaultOption("Nothing", null);
    autoSelect.addOption("Potato Move",new PotatoAuto(sDrivetrain) );
    autoSelect.addOption("Move Out", new MoveOut(sDrivetrain));

    SmartDashboard.putData(autoSelect);
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
    /// Op
    toggleGearBtn = new JoystickButton(mOpController, Constants.OpConstants.GearButton);
    toggleGearBtn.onTrue(sDrivetrain.cmdToggleGear());

    liftUpTrigger = new JoystickButton(mOpController, Constants.Lift.downButton);
    liftUpTrigger.whileTrue(new LiftDown(sLift));

    liftDownTrigger = new JoystickButton(mOpController, Constants.Lift.upButton);
    liftDownTrigger.whileTrue(new LiftUp(sLift));

    /// Coop

    // Groups
    intakeNoteTrigger = new JoystickButton(mCoopController, Constants.IntakeConstants.intakeButton);
    intakeNoteTrigger.whileTrue(new PickupNoteGroup(sArm, sIntake));
    intakeNoteTrigger.onFalse(new MoveArm(sArm, Constants.Arm.kDrivePosition));

    GenericEntry shootNodeStatusEntry = Shuffleboard.getTab("Auto Shoot").add("Status", "Not Running").withSize(3, 2).getEntry();
    shootNoteTrigger = new JoystickButton(mCoopController, Constants.Shooter.kShootButton);
    shootNoteTrigger.whileTrue(new ShootNoteGroup(sArm, sShooter, sIntake, sDrivetrain, sVision, shootNodeStatusEntry));
    shootNoteTrigger.onFalse(new InstantCommand(()->{shootNodeStatusEntry.setString("Not Running");}));

    ejectNodeTrigger = new Trigger(() -> {return mCoopController.getRawAxis(Constants.IntakeConstants.ejectButton)>=0.01; });
    ejectNodeTrigger.whileTrue(new IntakeOut(sIntake));

    ampTrigger = new JoystickButton(mCoopController, Constants.Shooter.kAmpButton);
    ampTrigger.whileTrue(new AmpGroup(sArm, sShooter, sIntake));

    // Override
    overrideShootNodeTrigger = new Trigger(() -> {return mCoopController.getRawAxis(Constants.Shooter.overrideShootButton)>=0.01; });
    overrideShootNodeTrigger.whileTrue(new ShootNote(sShooter, sIntake));

    armUpTrigger = new Trigger(() -> {return mCoopController.getPOV()==0;});
    armUpTrigger.whileTrue(new ArmUp(sArm));

    armDownTrigger = new Trigger(() -> {return mCoopController.getPOV()==180;});
    armDownTrigger.whileTrue(new ArmDown(sArm));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    SmartDashboard.putData(autoSelect);
  }

  
  // Getter Methods

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSelect.getSelected();
    }
  public Drivetrain getDrivetrain(){return sDrivetrain;}
  public Intake getIntake(){return sIntake;}
  public Shooter getShooter(){return sShooter;}
  public Arm getArm(){return sArm;}
  public XboxController getOpController(){return mOpController;}
}
