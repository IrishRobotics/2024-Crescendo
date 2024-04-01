// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Vision;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionAim extends Command {
  private Arm sArm;
  private Vision sVision;
  private PIDController pidController;
  private double position;
  private boolean valuesCalculated = false;

  /** Creates a new ArmShoot. */
  public VisionAim(Arm arm, Vision vision) {
    sArm = arm;
    sVision = vision;

    pidController = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
    pidController.setTolerance(ArmConstants.kTolerance);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    valuesCalculated = false;
    Shuffleboard.selectTab("Driver");
    calculateValues();
  }

  void calculateValues() {
    int id =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? 7
            : 4; // 7 for blue, 4 for red

    PhotonTrackedTarget target = sVision.TargetWithID(id);
    if (target == null) {
      return;
    }
    valuesCalculated = true;

    double distance = sVision.getTargetDistance(target);
    position = sArm.getTargetAngle(distance);

    pidController.setSetpoint(position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!valuesCalculated) {
      calculateValues();
    }
    double setSpeed = pidController.calculate(sArm.getAngle(), position);

    // double setspeed if we are less than motor can output.
    if (Math.abs(setSpeed) < 0.3) setSpeed *= 2;

    sArm.move(setSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sArm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
