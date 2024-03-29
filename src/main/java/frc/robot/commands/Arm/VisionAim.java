// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Vision;
import java.util.Map;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionAim extends Command {
  private Arm sArm;
  private Vision sVision;
  private PIDController pidController;
  private double position;

  // private GenericEntry shootingDistance;
  // private GenericEntry shootingAngle;
  // private GenericEntry statusEntry;

  /** Creates a new ArmShoot. */
  public VisionAim(Arm arm, Vision vision, GenericEntry statusEntry) {
    sArm = arm;
    // this.statusEntry = statusEntry;
    sVision = vision;

    pidController = new PIDController(0.2, 0, 0);
    pidController.setTolerance(0.5);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);

    // configureDashboard();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    valuesCalculated = false;
    Shuffleboard.selectTab("Auto Shoot");
    // if(statusEntry != null )    statusEntry.setString("Moving Arm");

    calculateValues();
  }

  boolean valuesCalculated = false;

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

    double distanceRaw =
        target.getBestCameraToTarget().getTranslation().getDistance(new Translation3d()) * 3.28084;

    double distance = (49.0 / 12) / Math.tan(Math.asin((49.0 / 12) / distanceRaw));
    distance -= 0.5; // Offset
    position = 10.3677 * Math.pow((distance - 3), 0.308731) + 12;

    // shootingDistance.setDouble(distance);
    // shootingAngle.setDouble(position);

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

    // shootingAngle.setDouble(0);
    // shootingDistance.setDouble(0);

    // if(statusEntry != null )  statusEntry.setString("Shooting");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }

  // private void configureDashboard() {
  //   ShuffleboardTab tab = Shuffleboard.getTab("Auto Shoot");

  //   shootingAngle =
  //       tab.add("Shooting Angle", 0)
  //           .withWidget(BuiltInWidgets.kGyro)
  //           .withSize(2, 2)
  //           .withProperties(Map.of("startingAngle", 270))
  //           .getEntry();

  //   shootingDistance =
  //       tab.add("Shooting Distance", 0)
  //           .withWidget(BuiltInWidgets.kNumberBar)
  //           .withSize(2, 1)
  //           .getEntry();
  // }
}
