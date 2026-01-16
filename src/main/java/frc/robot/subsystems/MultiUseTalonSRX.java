// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MultiUseTalonSRX extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  private final TalonSRX talon1;

  public MultiUseTalonSRX(int CANID) {
    talon1 = new TalonSRX(CANID);
  }

  public void runAtSpeed(double speed) {
    talon1.set(ControlMode.PercentOutput, speed);
  }

  public void stop() {
    talon1.set(ControlMode.PercentOutput, 0);
  }

  //public double getPosition() {return talon1.getPosition().getValue();}
  public int getID() {
    return talon1.getDeviceID();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    if (Constants.debugMode) {
      builder.setSmartDashboardType("Talon " + (getID()));
      //builder.addDoubleProperty("Encoder Position", this::getPosition, null);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}