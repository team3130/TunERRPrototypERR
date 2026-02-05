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
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.ToggleHubTargeting;
import frc.robot.commands.UpdateOdoFromVision;
import frc.robot.commands.Hopper.ReverseHopperHorizontal;
import frc.robot.commands.Hopper.ReverseHopperVertical;
import frc.robot.commands.Hopper.RunHopperHorizontal;
import frc.robot.commands.Hopper.RunHopperVertical;
import frc.robot.commands.Intake.ReverseIntake;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Shooter.Basic.ReverseShooter;
import frc.robot.commands.Shooter.Basic.RunShooter;
import frc.robot.commands.Shooter.PID.Rev;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.MultiUseTalonFX;
import frc.robot.subsystems.MultiUseTalonSRX;
import frc.robot.subsystems.MultiUseVictor;
import frc.robot.subsystems.Shooter;

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

    public final Intake intake;
    public final Shooter shooter;
    public final Hopper hopper;

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

        intake = new Intake();
        hopper = new Hopper();
        shooter = new Shooter();

        configureBindings();

        SmartDashboard.putData(command);
        SmartDashboard.putData(shooter);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(new TeleopDrive(drivetrain, commandDriverController, MaxSpeed, MaxAngularRate, drive));

        //Vision Updates
        limelights.setDefaultCommand(new UpdateOdoFromVision(drivetrain, limelights, logger));

        //commandDriverController.L2().whileTrue(new UpdateOdoFromVisionCommand(drivetrain));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        commandDriverController.create().and(commandDriverController.triangle()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        commandDriverController.create().and(commandDriverController.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        commandDriverController.options().and(commandDriverController.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        commandDriverController.options().and(commandDriverController.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        //commandDriverController.circle().whileTrue(new RunTalonSRX(talon5, 1));
        //commandDriverController.square().whileTrue(new RunVictor(victor4));
        //commandDriverController.L1().whileTrue(new RunTalonFX(falcon1, 1));
        //commandDriverController.L1().whileTrue(new RunTalonFX(falcon2, 1));

        commandDriverController.R1().whileTrue(new RunIntake(intake));
        commandDriverController.L1().whileTrue(new ReverseIntake(intake));

        commandDriverController.R2().whileTrue(new RunShooter(shooter));
        //commandDriverController.L2().whileTrue(new ReverseShooter(shooter));
        commandDriverController.L2().whileTrue(new Rev(shooter));

        commandDriverController.triangle().whileTrue(new RunHopperVertical(hopper));
        commandDriverController.cross().whileTrue(new ReverseHopperVertical(hopper));
        commandDriverController.circle().whileTrue(new RunHopperHorizontal(hopper));
        commandDriverController.square().whileTrue(new ReverseHopperHorizontal(hopper));


        //commandDriverController.R1().onTrue(new ToggleHubTargeting(drivetrain));

        // reset the field-centric heading
        commandDriverController.povUp().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //ommandDriverController.povUp().whileTrue(command);

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
