// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
    private final TalonFX intake;
    private double intakeSpeed = 0.5;
    public intake() {
        intake = new TalonFX(Constants.CAN.Falcon);
        intake.configFactoryDefault();
        intake.setInverted(true);
    } 

    public void runIntake() {
        intake.set(intakeSpeed);
    }

    public void reverseIntake() {
        intake.set(-intakeSpeed);
    }

    public void stopIntake() {
        intake.set(0);
    }

    public double getIntakeSpeed() {
        return intakeSpeed;
    }

    public void setIntakeSpeed(double speed) {
        intakeSpeed = speed;
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
}