// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
  private final TalonFX verticalHopper;
  private final TalonFX horizontalHopper;
  private double verticalSpeed = 0.1;
  private double horizontalSpeed = 0.1;
  /** Creates a new Hopper. */
  public Hopper() {
    verticalHopper = new TalonFX(34);
    horizontalHopper = new TalonFX(35);

    verticalHopper.getConfigurator().apply(new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive)));
 
    horizontalHopper.getConfigurator().apply(new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive)));
  }

  public void runHopperHorizontal() {
    horizontalHopper.set(horizontalSpeed);
  }

  public void reverseHopperHorizontal() {
    horizontalHopper.set(-horizontalSpeed);
  }

  public void stopHopperHorizontal() {
    horizontalHopper.set(0);
  }

    public void runHopperVertical() {
    verticalHopper.set(verticalSpeed);
  }

  public void reverseHopperVertical() {
    verticalHopper.set(-verticalSpeed);
  }

  public void stopHopperVertical() {
    verticalHopper.set(0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
