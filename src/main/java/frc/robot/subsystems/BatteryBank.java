// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Array;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BatteryBank extends SubsystemBase {

  Account[] accounts = {
    new Account("Shooter", 0, 0, 0, 0), 
    new Account("Hopper", 0 ,0 ,0 ,0), 
    new Account("Intake", 0, 0, 0, 0), 
    new Account("Chassis", 0, 0, 0, 0),
    new Account("Climber", 0, 0, 0, 0)
  };

  public Account openAccount(String name, double priority){
    Account newAccount = new Account(name, priority, 0, 0, 0);
    return newAccount;
  }
  /** Creates a new BatterBank. */
  public BatteryBank() {} 
  @Override
  public void periodic() {
  
    // This method will be called once per scheduler run
  }
}
