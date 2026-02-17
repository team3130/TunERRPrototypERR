// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OmnidirectionalShot extends Command {
  /** Creates a new OmnidirectionalShot. */
  private final Shooter shooter;
  private final CommandSwerveDrivetrain driveTrain;
  private Translation2d hubVector = new Translation2d(3.9, 0);
  // Constants for Gravity(g), Height(h) and Entry slope(m)
  private final double g = -9.8;
  private final double h = 1.25;
  private final double m = -0.5;

  public OmnidirectionalShot(Shooter shooter, CommandSwerveDrivetrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.driveTrain = driveTrain;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      Translation2d robotVector = driveTrain.getState().Pose.getTranslation();
      Translation2d targetVector = hubVector.minus(robotVector);
      Translation2d robotVelocityVector = new Translation2d(driveTrain.getState().Speeds.vxMetersPerSecond, driveTrain.getState().Speeds.vyMetersPerSecond);

      double targetAngle = targetVector.getAngle().getDegrees();
      double d = targetVector.getNorm();
      targetVector = targetVector.plus(new Translation2d(Math.sqrt(g * d/2 * d/(m*d-h)), targetAngle));
      double vx = targetVector.getNorm();
      double shotVelocity = Math.sqrt(vx*vx + (2*h/d - m)*(2*h/d - m)*(g * d/2 * d/(m*d-h)));
      targetAngle = targetVector.getAngle().getDegrees();
      double robotAngle = driveTrain.getState().Pose.getRotation().getDegrees();
      if(targetAngle - robotAngle > 180) {
        targetAngle -= 360;
      } else if(targetAngle - robotAngle < -180) {
        targetAngle += 360;
      }
      double angleInput = pidController.calculate(robotAngle, targetAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
