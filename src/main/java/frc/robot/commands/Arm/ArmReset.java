// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Arm;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Arm;

// public class ArmReset extends Command {
//   private Arm sArm;

//   /** Creates a new Armreset. */
//   public ArmReset(Arm arm) {
//     sArm = arm;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(arm);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     sArm.setSetpoint(-1000);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {}

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     if(sArm.encoderReset){
//       sArm.encoderReset = false;
//       return true;
//     }
//     return false;
//   }
// }
