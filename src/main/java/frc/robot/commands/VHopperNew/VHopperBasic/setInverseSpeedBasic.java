// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.VHopperNew.VHopperBasic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VHopperNew;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class setInverseSpeedBasic extends Command {
  /** Creates a new setInverseSpeedBasic. */
  private final VHopperNew hopper;
  public setInverseSpeedBasic(VHopperNew hopper) {
    this.hopper = hopper;
    addRequirements(hopper);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hopper.setInverseSpeedBasic();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.setVMotorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
