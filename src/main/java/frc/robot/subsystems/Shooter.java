// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final TalonFX rightFlyWheelM;
  private final TalonFX leftFlyWheelM; 

  public Shooter() {
    rightFlyWheelM = new TalonFX(32);
    leftFlyWheelM = new TalonFX(33);
    leftFlyWheelM.setControl(new Follower(rightFlyWheelM.getDeviceID(), MotorAlignmentValue.Opposed));
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
