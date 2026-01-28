// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
  /** Creates a new Hopper. */

  private TalonFX talonHopperDown;
  private TalonFX talonHopperUp;

  double hopperSpeedDown = 0.5;
  double hopperSpeedUp = 0.5;

  public void hopperRunDown() {
    talonHopperDown.set(hopperSpeedDown);
  }

  public void hopperRunInvertedDown() {
    talonHopperDown.set(-hopperSpeedDown);
  }

  public void hopperStopDown() {
    talonHopperDown.set(0);
  }

  public void hopperRunUp() {
    talonHopperUp.set(hopperSpeedUp);
  }

  public void hopperRunInvertedUp() {
    talonHopperUp.set(-hopperSpeedUp);
  }

  public void hopperStopUp() {
    talonHopperUp.set(0);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
