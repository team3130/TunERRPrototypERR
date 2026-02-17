// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivot extends SubsystemBase {
  /** Creates a new IntakePivot. */
  private final TalonFX pivotM;
  
  private final double IntakeAngleDown = 0;
  private final double IntakeRotDown = IntakeAngleDown*(Math.PI/180);

  private final double IntakeAngleUp = 90;
  private final double IntakeRotUp = IntakeAngleUp*(Math.PI/180);

  private final double IntakePivotVelo = 0.1;

  private final Slot0Configs slot0Configs;
  private double slot0kG = 0;
  private double slot0kV = 0.12; 
  private double slot0kA = 0.01;
  private double slot0kP = 0; 
  private double slot0kI = 0;
  private double slot0kD = 0.0;
  private final MotionMagicDutyCycle voltRequest0;
  private final TalonFXConfiguration configM;

  private double targetVelocityMetersPerSec = 15;
  private final double targetVelocityRotations = Units.radiansToRotations(targetVelocityMetersPerSec/Units.inchesToMeters(2));
  private double accelerationMetersPerSecSquared = 5;
  private final double accelerationRotations = Units.radiansToRotations(accelerationMetersPerSecSquared/Units.inchesToMeters(2));

  public IntakePivot() {
    pivotM = new TalonFX(0);
    pivotM.setPosition(IntakeRotUp);

    voltRequest0 = new MotionMagicDutyCycle(0);
    slot0Configs = new Slot0Configs().withGravityType(GravityTypeValue.Arm_Cosine);
    slot0Configs.kG = slot0kG;
    slot0Configs.kP = slot0kP;
    slot0Configs.kI = slot0kI;
    slot0Configs.kD = slot0kD;

    //Motor Config and Motion Magic Pivot
    configM = new TalonFXConfiguration();
    configM.MotorOutput = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake).withInverted(InvertedValue.Clockwise_Positive);
    configM.MotionMagic = new MotionMagicConfigs().withMotionMagicAcceleration(accelerationRotations);
    configM.CurrentLimits.withStatorCurrentLimitEnable(true).withStatorCurrentLimit(50);
    configM.Slot0 = slot0Configs;
    pivotM.getConfigurator().apply(configM);
  }

  public void IntakeUpPID() {
    pivotM.setControl(voltRequest0.withPosition(IntakeAngleUp));
  }

  public void IntakeDownPID() {
    pivotM.setControl(voltRequest0.withPosition(IntakeAngleDown));
  }

  public void IntakeDown() {
    pivotM.set(-IntakePivotVelo);
  }

  public void IntakeUp() {
    pivotM.set(IntakePivotVelo);
  }

  public void IntakeStop() {
    pivotM.set(0);
  }

  public double getPosition() {return pivotM.getPosition().getValueAsDouble();}

  public void IntakeSetUp() {
    if (getPosition() < IntakeRotUp) {
      IntakeUp();
    }
    if (getPosition() >= IntakeRotUp) {
      IntakeStop();
    }
  }

  public double getShooterkG() {return slot0kG;}
  public double getShooterkP() {return slot0kP;}
  public double getShooterkI() {return slot0kI;}
  public double getShooterkD() {return slot0kD;}
  public void setShooterkG(double value) {slot0kG = value;}
  public void setShooterkP(double value) {slot0kP = value;}
  public void setShooterkI(double value) {slot0kI = value;}
  public void setShooterkD(double value) {slot0kD = value;}
  public double getTargetIntakeUp() {return IntakeRotUp;}
  public double getTargetIntakeDown() {return IntakeRotDown;}

  public double getProfileVelocity() {
    double rotsPerSec = pivotM.getClosedLoopReference().getValueAsDouble();
    double radsPerSec = Units.rotationsToRadians(rotsPerSec);
    return radsPerSec;
  }

  public double getVelocity() {
    double rotsPerSec = pivotM.getVelocity().getValueAsDouble();
    double radsPerSec = Units.rotationsToRadians(rotsPerSec);
    return radsPerSec;
}

  public double getTargetVelocity() {return targetVelocityMetersPerSec;}
  public void setTargetVelocity(double value) {targetVelocityMetersPerSec = value;}

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Intake Pivot");
    builder.addDoubleProperty("Motor Velocity", this::getVelocity, null);
    builder.addDoubleProperty("Target Velocity", this::getTargetVelocity, this::setTargetVelocity);
    builder.addDoubleProperty("Profile Velocity", this::getProfileVelocity, null);
    builder.addDoubleProperty("Shooter kA", this::getShooterkG, this::setShooterkG);
    builder.addDoubleProperty("Shooter kP", this::getShooterkP, this::setShooterkP);
    builder.addDoubleProperty("Shooter kI", this::getShooterkI, this::setShooterkI);
    builder.addDoubleProperty("Shooter kD", this::getShooterkD, this::setShooterkD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
