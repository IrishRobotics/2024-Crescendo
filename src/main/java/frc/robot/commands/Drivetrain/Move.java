// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.OpConstants;
import frc.robot.subsystems.Drivetrain;

public class Move extends Command {
  private Drivetrain sDrivetrain;
  PIDController xController;
  PIDController yController;
  Pose2d target, current;

  /** Creates a new Move. */
  public Move(Pose2d target, Drivetrain drivetrain) {
    sDrivetrain = drivetrain;
    this.target = target;

    xController = new PIDController(Constants.OpConstants.moveKP, Constants.OpConstants.moveKI, Constants.OpConstants.moveKD);
    yController = new PIDController(Constants.OpConstants.moveKP, Constants.OpConstants.moveKI, Constants.OpConstants.moveKD);

    xController.setTolerance(0.02);
    yController.setTolerance(0.02);

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.setSetpoint(target.getX());
    yController.setSetpoint(target.getY());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed, ySpeed;
    this.current = sDrivetrain.robotPose;

    xSpeed = xController.calculate(current.getX());
    ySpeed = yController.calculate(current.getY());

    if(xController.atSetpoint()){
      xSpeed = 0;
    }else{
      xSpeed = Math.max(Math.abs(xSpeed), OpConstants.minSpeed)*Math.signum(xSpeed);
    }

    if(yController.atSetpoint()){
      ySpeed = 0;
    }else{
      ySpeed = Math.max(Math.abs(ySpeed), OpConstants.minSpeed)*Math.signum(ySpeed);
    }

    sDrivetrain.Drive(xSpeed, ySpeed, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sDrivetrain.Drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint();
  }
}
