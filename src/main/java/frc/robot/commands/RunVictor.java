// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MultiUseVictor;

/** An example command that uses an example subsystem. */
public class RunVictor extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final MultiUseVictor victor;
  private final double speed = -0.5;

  /**
   * Creates a new ExampleCommand.
   *
   * @param victor The subsystem used by this command.
   */
  public RunVictor(MultiUseVictor victor) {
    this.victor = victor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(victor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    victor.runAtSpeed(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    victor.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
