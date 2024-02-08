// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.commands.Drivetrain.OperatorDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
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

  // Joysticks
  private XboxController mOpController = new XboxController(Constants.kDriverControllerPort);

  // Joystick Buttons
  
  private Trigger toggleGearBtn;
  // TODO - Add buttons

  // Auto Chooser
  SendableChooser<Command> autoSelect = new SendableChooser<>();
  
  // Autonomous
  // TODO - Add auto commands
  



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Default Commands
    sDrivetrain.setDefaultCommand(new OperatorDrive(sDrivetrain, mOpController, false));
    
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
    
    toggleGearBtn = new JoystickButton(mOpController, XboxController.Button.kStart.value);
    toggleGearBtn.onTrue(sDrivetrain.cmdToggleGear());

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  
  // Getter Methods

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {return autoSelect.getSelected();  }
  public Drivetrain getDrivetrain(){return sDrivetrain;}
  public Intake getIntake(){return sIntake;}
  public Shooter getShooter(){return sShooter;}
  public Arm getArm(){return sArm;}
  public XboxController getOpController(){return mOpController;}
}
