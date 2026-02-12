// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.AccelLimiter;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleopDrive extends Command {
  private final CommandSwerveDrivetrain driveTrain;
  private final CommandPS5Controller controller;
  private final double maxSpeed;
  private final double maxAngularRate;
  private final SwerveRequest.FieldCentric drive;
  private final AccelLimiter accelLimiter;
  private Translation2d hubVector = new Translation2d(3.9, 0);
  private final PIDController pidController;
  /** Creates a new TeleopDrive. */
  public TeleopDrive(CommandSwerveDrivetrain driveTrain, CommandPS5Controller controller, 
                    double maxSpeed, double maxAngularRate, 
                    SwerveRequest.FieldCentric drive) {
    this.driveTrain = driveTrain;
    this.controller = controller;
    this.maxSpeed = maxSpeed;
    this.maxAngularRate = maxAngularRate;
    this.drive = drive;
    accelLimiter = new AccelLimiter(2, -5, 0, 0);
    pidController = new PIDController(0.05, 0, 0);
    SmartDashboard.putData(pidController);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(driveTrain.getHubToggle()) {
      Translation2d robotVector = driveTrain.getState().Pose.getTranslation();
      Translation2d targetVector = hubVector.minus(robotVector);

      double targetAngle = targetVector.getAngle().getDegrees();
      double robotAngle = driveTrain.getState().Pose.getRotation().getDegrees();
      if(targetAngle - robotAngle > 180) {
        targetAngle -= 360;
      } else if(targetAngle - robotAngle < -180) {
        targetAngle += 360;
      }

      double xAxis = -controller.getLeftY();
      double yAxis = -controller.getLeftX();
      xAxis = MathUtil.applyDeadband(xAxis, Constants.Swerve.kDeadband);
      yAxis = MathUtil.applyDeadband(yAxis, Constants.Swerve.kDeadband);

      ChassisSpeeds limitedSpeeds = accelLimiter.accelLimitVectorDrive(new ChassisSpeeds(xAxis, yAxis, pidController.calculate(robotAngle, targetAngle)));
      driveTrain.setControl(drive
              .withVelocityX(limitedSpeeds.vxMetersPerSecond)
              .withVelocityY(limitedSpeeds.vyMetersPerSecond)
              .withRotationalRate(limitedSpeeds.omegaRadiansPerSecond)
      );
      } else {
      pidController.reset();
      driveTrain.setControl(driveTrain.applyDeadband(drive, controller, maxSpeed, maxAngularRate));
    }
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
