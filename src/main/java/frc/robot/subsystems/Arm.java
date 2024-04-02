// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import java.util.Map;

public class Arm extends SubsystemBase {
  // Motors
  private WPI_TalonSRX mMotor1;
  // Sensors
  private DutyCycleEncoder angleSensor;
  // Shuffleboard
  private ShuffleboardTab tab;
  private ShuffleboardTab driveTab;
  private GenericEntry sArmPosition;
  private GenericEntry sTargetAngle;
  private GenericEntry sArmSpeed;

  /** Creates a new Arm. */
  public Arm() {
    mMotor1 = new WPI_TalonSRX(ArmConstants.kArmMotor1);
    mMotor1.setNeutralMode(NeutralMode.Brake);

    angleSensor = new DutyCycleEncoder(ArmConstants.kAbsEncoder);
    angleSensor.setPositionOffset(ArmConstants.kEncoderOffset);

    this.addChild("Motor", mMotor1);
    this.addChild("Encoder", angleSensor);

    configureDashboard();
  }

  @Override
  public void periodic() {
    sArmPosition.setDouble(getAngle());
  }

  public void move(double speed) {
    // Check if angle is past soft limits
    if ((getAngle() > ArmConstants.kMaxAngle && speed > 0)
        || (getAngle() < ArmConstants.kMinAngle && speed < 0)) {
      speed = 0;
    }

    if(Math.abs(speed) < 0.4) speed = 0.4 * Math.signum(speed);

    mMotor1.set(speed);
    sArmSpeed.setDouble(speed);
  }

  public void stop() {
    mMotor1.set(0);
    sArmSpeed.setDouble(0);
  }

  public double getAngle() {
    return (angleSensor.getAbsolutePosition() * 360 + 7) % 360;
  }

  public double getTargetAngle(double feet) {
    double val = 10.3677 * Math.pow((feet - 3), 0.308731) + 12;
    sTargetAngle.setDouble(val);
    return val;
  }

  private void configureDashboard() {
    tab = Shuffleboard.getTab("Arm");
    driveTab = Shuffleboard.getTab("Driver");

    sArmPosition =
        tab.add("Position", this.getAngle())
            .withSize(2, 2)
            .withWidget(BuiltInWidgets.kGyro)
            .withProperties(Map.of("startingAngle", 270))
            .getEntry();

    sArmSpeed =
        tab.add("Speed", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -1, "max", 1))
            .getEntry();
    
    sTargetAngle = 
        tab.add("Target Angle", this.getAngle())
            .withSize(2, 2)
            .withWidget(BuiltInWidgets.kGyro)
            .withProperties(Map.of("startingAngle", 270))
            .getEntry();
  }

  // Commands
  public Command cmdUp() {
    return this.runEnd(() -> this.move(0.5), this::stop);
  }

  public Command cmdDown() {
    return this.runEnd(() -> this.move(-0.5), this::stop);
  }
}
