// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Swerve;
import frc.robot.commands.DriveWithTransPID;
import frc.robot.commands.RunTalonFX;
import frc.robot.commands.RunTalonSRX;
import frc.robot.commands.RunVictor;
import frc.robot.commands.UpdateOdoFromVision;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.MultiUseTalonFX;
import frc.robot.subsystems.MultiUseTalonSRX;
import frc.robot.subsystems.MultiUseVictor;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(5).in(RotationsPerSecond); // 3/4 of a rotation per second max angular velocity
    //used to be 0.75 but changed to 0.25

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandPS5Controller commandDriverController = new CommandPS5Controller(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Limelight limelights = new Limelight(drivetrain);
    public final MultiUseTalonSRX talon1;
    public final MultiUseTalonSRX talon2;
    public final MultiUseTalonSRX talon3;
    public final MultiUseVictor victor4;
    public final MultiUseTalonSRX talon5;

    public final MultiUseTalonFX falcon1;
    public final MultiUseTalonFX falcon2;

    private final SendableChooser<Command> autoChooser;

    private final DriveWithTransPID command = new DriveWithTransPID(drivetrain, drive);

    public RobotContainer() {
        talon1 = new MultiUseTalonSRX(1);
        talon2 = new MultiUseTalonSRX(2);
        talon3 = new MultiUseTalonSRX(3);
        victor4 = new MultiUseVictor(4);
        talon5 = new MultiUseTalonSRX(5);

        falcon1 = new MultiUseTalonFX(30);
        falcon2 = new MultiUseTalonFX(31);

        configureBindings();

        SmartDashboard.putData(command);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public SwerveRequest applyDeadband() {
        double xJoystick = -commandDriverController.getLeftY();
        double yJoystick = -commandDriverController.getLeftX();
        double rotationJoystick = -commandDriverController.getRightX();
        
        xJoystick = MathUtil.applyDeadband(xJoystick, 0.04);
        yJoystick = MathUtil.applyDeadband(yJoystick, 0.04);
        rotationJoystick = MathUtil.applyDeadband(rotationJoystick, 0.04);

        xJoystick = xJoystick * Math.abs(xJoystick);
        yJoystick = yJoystick * Math.abs(yJoystick);
        rotationJoystick = rotationJoystick * Math.abs(rotationJoystick);

        return drive.withVelocityX(xJoystick * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(yJoystick * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(rotationJoystick * MaxAngularRate); // Drive counterclockwise with negative X (left)
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(this::applyDeadband));

        //Vision Updates
        limelights.setDefaultCommand(new UpdateOdoFromVision(drivetrain, limelights, logger));

        //commandDriverController.L2().whileTrue(new UpdateOdoFromVisionCommand(drivetrain));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        commandDriverController.create().and(commandDriverController.triangle()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        commandDriverController.create().and(commandDriverController.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        commandDriverController.options().and(commandDriverController.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        commandDriverController.options().and(commandDriverController.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        commandDriverController.circle().whileTrue(new RunTalonSRX(talon5, 1));
        commandDriverController.square().whileTrue(new RunVictor(victor4));
        commandDriverController.L1().whileTrue(new RunTalonFX(falcon1, 1));
        commandDriverController.L1().whileTrue(new RunTalonFX(falcon2, 1));

        // reset the field-centric heading
        commandDriverController.povUp().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //ommandDriverController.povUp().whileTrue(command);

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
