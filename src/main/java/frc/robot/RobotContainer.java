// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.lang.ModuleLayer.Controller;

import javax.naming.AuthenticationNotSupportedException;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.RunIntake;
import frc.robot.commands.UpdateOdoFromVision;
import frc.robot.commands.Shooter.ShootForward;
import frc.robot.commands.Shooter.ShootForwardBasic;
import frc.robot.commands.Shooter.ShootInverted;
import frc.robot.commands.Shooter.ShootInvertedBasic;
import frc.robot.commands.VHopperNew.VHopperBasic.setInverseSpeedBasic;
import frc.robot.commands.VHopperNew.VHopperBasic.setSpeedBasic;
import frc.robot.commands.VHopperNew.VHopperPID.setPIDInvertedSpeed;
import frc.robot.commands.VHopperNew.VHopperPID.setPIDSpeed;
//import frc.robot.commands.VHopperNew.VHopperBasic.setInverseSpeedBasic;
///import frc.robot.commands.VHopperNew.VHopperBasic.setSpeedBasic;
//import frc.robot.commands.VHopperNew.VHopperPID.setPIDInvertedSpeed;
//import frc.robot.commands.VHopperNew.VHopperPID.setPIDSpeed;
import frc.robot.commands.RunTalonFX;
import frc.robot.commands.RunTalonSRX;
import frc.robot.commands.RunVictor;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.ToggleHubTargeting;
import frc.robot.commands.UpdateOdoFromVision;
import frc.robot.commands.Hopper.ReverseHopperVertical;
import frc.robot.commands.Hopper.Reversehopper;
import frc.robot.commands.Hopper.RunHopper;
import frc.robot.commands.Hopper.RunHoppervertical;
import frc.robot.commands.IntakePivot.IntakePivotBasic.IntakeSetDownBasic;
import frc.robot.commands.IntakePivot.IntakePivotBasic.IntakeSetUpBasic;
import frc.robot.commands.IntakePivot.IntakePivotPID.IntakeSetDownPID;
import frc.robot.commands.IntakePivot.IntakePivotPID.IntakeSetUpPID;
import frc.robot.commands.RunIntake;
import frc.robot.commands.ReverseIntake;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VHopperNew;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.MultiUseTalonFX;
import frc.robot.subsystems.MultiUseTalonSRX;
import frc.robot.subsystems.MultiUseVictor;
import frc.robot.subsystems.VerticalHopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;

import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(5).in(RotationsPerSecond); // 3/4 of a rotation per second max angular velocity
    //used to be 0.75 but changed to 0.25

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
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

    public final Hopper hopper;
    public final VHopperNew vHopperNew;
    public final VerticalHopper verticalHopper;
    private final Intake intake;
    private final RunIntake runIntake;
    private final ReverseIntake reverseIntake;

    private final SendableChooser<Command> autoChooser;

    private final DriveWithTransPID command = new DriveWithTransPID(drivetrain, drive);

    private final Shooter shooter;

    private final IntakePivot pivot;

    public RobotContainer() {
        talon1 = new MultiUseTalonSRX(1);
        talon2 = new MultiUseTalonSRX(2);
        talon3 = new MultiUseTalonSRX(3);
        victor4 = new MultiUseVictor(4);
        talon5 = new MultiUseTalonSRX(5);

        vHopperNew = new VHopperNew();

        pivot = new IntakePivot();

        hopper = new Hopper();
        verticalHopper = new VerticalHopper();
        //SmartDashboard.putData("Vertical Hopper", verticalHopper);
        SmartDashboard.putData("Vertical Hopper", vHopperNew);

        falcon1 = new MultiUseTalonFX(30);
        falcon2 = new MultiUseTalonFX(31);

        intake = new Intake();
        runIntake = new RunIntake(intake, commandDriverController);
        reverseIntake = new ReverseIntake(intake, commandDriverController);

        
        shooter = new Shooter();

        SmartDashboard.putData(command);
        SmartDashboard.putData(shooter);
        
        SmartDashboard.putData(vHopperNew);
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
//intake
        NamedCommands.registerCommand("Run Intake", new RunIntake(intake, commandDriverController));
        NamedCommands.registerCommand("Reverse Intake", new ReverseIntake(intake, commandDriverController));
//hopper
        NamedCommands.registerCommand("Run Hopper", new RunHopper(hopper));
        NamedCommands.registerCommand("Reverse Hopper", new Reversehopper(hopper));
        NamedCommands.registerCommand("Run Vertical Hopper", new RunHoppervertical(verticalHopper));
        NamedCommands.registerCommand("Reverse Vertical Hopper", new ReverseHopperVertical(verticalHopper));
//Ansh hopper
        NamedCommands.registerCommand("Run Vertical Hopper Basic", new setSpeedBasic(vHopperNew));
        NamedCommands.registerCommand("Reverse Vertical Hopper Basic", new setInverseSpeedBasic(vHopperNew));
        NamedCommands.registerCommand("Run Vertical Hopper PID", new setPIDSpeed(vHopperNew));
        NamedCommands.registerCommand("Reverse Vertical Hopper PID", new setPIDInvertedSpeed(vHopperNew));
//shooter
        NamedCommands.registerCommand("Shoot Foward Basic", new ShootForwardBasic(shooter));
        NamedCommands.registerCommand("Shoot Inverted Basic", new ShootInvertedBasic(shooter));
        NamedCommands.registerCommand("Shoot Foward PID", new ShootForward(shooter));
        NamedCommands.registerCommand("Shoot Inverted PID", new ShootInverted(shooter));
//hood
        NamedCommands.registerCommand("null", command);
        NamedCommands.registerCommand("null", command);
//Intake Pivot
        NamedCommands.registerCommand("Intake Set Up Basic", new IntakeSetUpBasic(pivot));
        NamedCommands.registerCommand("Intake Set Down Basic", new IntakeSetDownBasic(pivot));
        NamedCommands.registerCommand("Intake Set Up PID", new IntakeSetUpPID(pivot));
        NamedCommands.registerCommand("Intake Set Down PID", new IntakeSetDownPID(pivot));
//climber
        //NamedCommands.registerCommand("null", command);
        //NamedCommands.registerCommand("null", command);
        //NamedCommands.registerCommand("null", command);
        //NamedCommands.registerCommand("null", command);
        //NamedCommands.registerCommand("null", command);
        //NamedCommands.registerCommand("null", command);  
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

        //commandDriverController.R1().onTrue(new ToggleHubTargeting(drivetrain));
        //if triangle is pressed hopper should run until triangle is pressed again, same for vert.hopper but with the x button
        //wrong
        //commandDriverController.triangle().whileTrue(new RunHopper(hopper));
        //commandDriverController.cross().whileTrue(new RunHoppervertical(verticalHopper));
        
        //NEW VHOPPER
        //commandDriverController.square().whileTrue(new setInverseSpeedBasic(vHopperNew));
        //commandDriverController.circle().whileTrue(new setSpeedBasic(vHopperNew));

        //NEW VHOPPER PID
        commandDriverController.square().whileTrue(new setPIDSpeed(vHopperNew));
        commandDriverController.circle().whileTrue(new setInverseSpeedBasic(vHopperNew));

        
        //commandDriverController.square().whileTrue(new Reversehopper(hopper));
        //commandDriverController.circle().whileTrue(new ReverseHopperVertical(verticalHopper));
        //commandDriverController.circle().whileTrue(new RunTalonSRX(talon5, 1));
        //commandDriverController.square().whileTrue(new RunVictor(victor4));
        //commandDriverController.L1().whileTrue(new RunTalonFX(falcon1, 1));
        //commandDriverController.L1().whileTrue(new RunTalonFX(falcon2, 1));

        //commandDriverController.R1().onTrue(new ToggleHubTargeting(drivetrain));

        commandDriverController.L1().whileTrue(new RunIntake(intake, commandDriverController));
        commandDriverController.R1().whileTrue(new ReverseIntake(intake, commandDriverController));

        // reset the field-centric heading
        commandDriverController.povUp().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //ommandDriverController.povUp().whileTrue(command);

        drivetrain.registerTelemetry(logger::telemeterize);

        //commandDriverController.R2().whileTrue(new ShootForwardBasic(shooter));
        //commandDriverController.L2().whileTrue(new ShootInvertedBasic(shooter));
        commandDriverController.R2().whileTrue(new ShootForward(shooter));
        commandDriverController.L2().whileTrue(new ShootInverted(shooter));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    }
