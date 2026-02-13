// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VHopperNew extends SubsystemBase {
  private final TalonFX Vmotor;
  private final double speedHopper = 0.2;
  private final MotionMagicVelocityVoltage voltRequest;
  private double targetAccelerationMetersPerSecSquared = 10; // m/s^2 (target linear acceleration)
  private double targetAccelerationRotsPerSecond = targetAccelerationMetersPerSecSquared/((0.025)*2*Math.PI);

  private double targetVelocityMetersPerSec = 11.25; // target linear velocity (m/s)
  private double targetVelocityRotationsPerSec = targetVelocityMetersPerSec/((0.025)*2*Math.PI);
  
  private double kV = 0.0;   
  private double kA = 0.00;
  private double kP = 0.0;
  private double kI = 0.00;
  private double kD = 0.00;
  private final Slot0Configs slot0Configs;
  
  private final TalonFXConfiguration motorConfig;

  public VHopperNew() {
    Vmotor = new TalonFX(34);

    slot0Configs = new Slot0Configs();
    slot0Configs.kV = kV;
    slot0Configs.kA = kA;
    slot0Configs.kP = kP;
    slot0Configs.kI = kI;
    slot0Configs.kD = kD;

    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast);
    motorConfig.MotorOutput = new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive);
    motorConfig.MotionMagic = new MotionMagicConfigs().withMotionMagicAcceleration(targetAccelerationRotsPerSecond);
    motorConfig.Feedback = new FeedbackConfigs();
    motorConfig.Slot0 = slot0Configs;

    Vmotor.getConfigurator().apply(motorConfig);

    voltRequest = new MotionMagicVelocityVoltage(0);
  }

  public void setPIDSpeed() {
    Vmotor.setControl(voltRequest.withVelocity(targetVelocityRotationsPerSec));
  }

  public void setPIDInvertedSpeed() {
    Vmotor.setControl(voltRequest.withVelocity(-targetVelocityRotationsPerSec)); 
  }

  public void setSpeedBasic() {
    Vmotor.set(speedHopper);
  }

  public void setInverseSpeedBasic() {
    Vmotor.set(-speedHopper);
  }

  public void setVMotorStop() {
    Vmotor.set(0);
  }

  public void updatePID() {
    slot0Configs.kV = kV;
    slot0Configs.kA = kA;
    slot0Configs.kP = kP;
    slot0Configs.kI = kI;
    slot0Configs.kD = kD;
    motorConfig.Slot0 = slot0Configs;
    motorConfig.MotionMagic = new MotionMagicConfigs().withMotionMagicAcceleration(targetAccelerationRotsPerSecond);
    Vmotor.getConfigurator().apply(motorConfig);
  }

  public double getTargetVelocity() {return targetVelocityMetersPerSec;}
  public double getTargetAcceleration() {return targetAccelerationMetersPerSecSquared;}
  public void setTargetAcceleration(double value) {targetAccelerationMetersPerSecSquared = value;}
  public void setTargetVelocity(double value) {targetVelocityMetersPerSec = value;}

  public double getVelocity() {
    double rotsPerSec = Vmotor.getVelocity().getValueAsDouble();
    double linVelo = (rotsPerSec*2*Math.PI*0.025);
    return linVelo;
  }

  public double getAcceleration() {
    double rotsPerSec = Vmotor.getAcceleration().getValueAsDouble();
    double linAcc = (rotsPerSec*2*Math.PI*0.025);
    return linAcc;
  }

  public double getkV() {return kV;}
  public double getkA() {return kA;}
  public double getkP() {return kP;}
  public double getkI() {return kI;}
  public double getkD() {return kD;}
  public void setkV(double value) {kV = value;}
  public void setkA(double value) {kA = value;}
  public void setkP(double value) {kP = value;}
  public void setkI(double value) {kI = value;}
  public void setkD(double value) {kD = value;}

  public double getProfileAcceleration() {
    double rotsPerSec = Vmotor.getClosedLoopReferenceSlope().getValueAsDouble();
    return (rotsPerSec*0.025*Math.PI*2);
  }

  public double getProfileVelocity() {
    double rotsPerSec = Vmotor.getClosedLoopReference().getValueAsDouble();
    return (rotsPerSec*0.025*Math.PI*2);
  }

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("VHopper New");

    builder.addDoubleProperty("Velocity (m/s)", this::getVelocity, null);
    builder.addDoubleProperty("Acceleration (m/s^2)", this::getAcceleration, null);

    builder.addDoubleProperty("Target Acceleration (m/s^2)", this::getTargetAcceleration, this::setTargetAcceleration);
    builder.addDoubleProperty("Target Velocity (m/s)", this::getTargetVelocity, this::setTargetVelocity);

    builder.addDoubleProperty("Profile Velocity", this::getProfileVelocity, null);
    builder.addDoubleProperty("Profile Acceleration", this::getProfileAcceleration, null);

    builder.addDoubleProperty("kV", this::getkV, this::setkV);
    builder.addDoubleProperty("kA", this::getkA, this::setkA);
    builder.addDoubleProperty("kP", this::getkP, this::setkP);
    builder.addDoubleProperty("kI", this::getkI, this::setkI);
    builder.addDoubleProperty("kD", this::getkD, this::setkD);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
