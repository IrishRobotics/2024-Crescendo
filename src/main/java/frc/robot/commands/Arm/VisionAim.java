// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Vision;

public class VisionAim extends Command {
  private Arm sArm;
  private Vision sVision;
  private PIDController pidController;
  private double position;


  /** Creates a new ArmShoot. */
  public VisionAim(Arm arm, Vision vision) {
    sArm = arm;
    sVision = vision;

    pidController = new PIDController(0.2, 0, 0);
    pidController.setTolerance(0.5);

    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int id = DriverStation.getAlliance().orElse(Alliance.Blue)==Alliance.Blue ? 7 : 4; // 7 for blue, 4 for red

    PhotonTrackedTarget target = sVision.TargetWithID(id);
    if(target==null){
        this.cancel();
        return;
    }

    double distanceRaw = target.getBestCameraToTarget().getTranslation().getDistance(new Translation3d()) * 3.28084;

    double distance = (49.0/12)/Math.tan(Math.asin((49.0/12)/distanceRaw));
    distance -= 0.5; //Offset
    position = 10.3677*Math.pow((distance-3),0.308731)+12;

    SmartDashboard.putNumber("Shooting Distance", distance);
    SmartDashboard.putNumber("Shooting Angle", position);

    pidController.setSetpoint(position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double setSpeed = pidController.calculate(sArm.getAngle(),position);

    //double setspeed if we are less than motor can output.
    if(Math.abs(setSpeed) < 0.3) setSpeed *= 2;

    if((sArm.getAngle()>90&&setSpeed>0)||(sArm.getAngle()<5&&setSpeed<0)){
        sArm.move(0);
    }else{
        sArm.move(setSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sArm.move(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
    // return Math.abs(sArm.GetAngle()-position)<.5;
    return pidController.atSetpoint();
  }
}
