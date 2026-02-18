// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakePivot.IntakePivotBasic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeSetDownBasic extends Command {
  /** Creates a new IntakeSetUpBasic. */
  private final IntakePivot pivot;
  public IntakeSetDownBasic(IntakePivot pivot) {
    this.pivot = pivot;
    addRequirements(pivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (pivot.getPosition() > pivot.getTargetIntakeDown()) {
      pivot.IntakeDown();
    }
    if (pivot.getPosition() < pivot.getTargetIntakeDown() + 1 && pivot.getPosition() > pivot.getTargetIntakeDown() - 1) {
      pivot.IntakeStop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivot.IntakeStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
