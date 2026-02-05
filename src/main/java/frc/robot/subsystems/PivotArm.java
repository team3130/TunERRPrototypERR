// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotArm extends SubsystemBase {
  public boolean brokeBottomLimit = false;
  public boolean brokeTopLimit = false;
  private final TalonFX pivotArm;
  private double rotSpeed = 0.1;

  /** Creates a new PivotArm. */
  public PivotArm(){
    pivotArm = new TalonFX(1);
    pivotArm.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

  }

  public void pivotUp(){
    pivotArm.set(rotSpeed);
  }

  public void pivotDown(double rotSpeed){
    pivotArm.set(-rotSpeed);
  }

  public void pivotStop(){
    pivotArm.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
