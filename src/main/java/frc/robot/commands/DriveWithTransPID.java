package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class DriveWithTransPID extends Command {
    private final CommandSwerveDrivetrain driveTrain;
    private final ProfiledPIDController pidController;
    private final SwerveRequest.FieldCentric drive;
    private double kP = 5.5;
    private double kI = 0;
    private double kD = 0;
    private double setpoint = 0;

    public DriveWithTransPID(CommandSwerveDrivetrain driveTrain, SwerveRequest.FieldCentric drive) {
        this.driveTrain = driveTrain;
        this.drive = drive;
        pidController = new ProfiledPIDController(kP ,kI, kD, new Constraints(3, 4));
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveTrain);
        SmartDashboard.putData(pidController);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        pidController.reset(driveTrain.getStatePose().getX());
        setpoint = driveTrain.getStatePose().getX() + 5;
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        double value = pidController.calculate(driveTrain.getStatePose().getX(), setpoint);
        driveTrain.setControl(drive
                .withVelocityX(value)
                .withVelocityY(0)
                .withRotationalRate(0));
    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {

    }

    public void setP(double value) {
        kP = value;
    }

    public void setI(double value) {
        kI = value;
    }

    public void setD(double value) {
        kD = value;
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Trans PID Command");

        builder.addDoubleProperty("Setpoint Velocity", () -> pidController.getSetpoint().velocity, null);
        builder.addDoubleProperty("Setpoint Position", () -> pidController.getSetpoint().position, null);

        builder.addDoubleProperty("P", () -> this.kP, this::setP);
        builder.addDoubleProperty("I", () -> this.kI, this::setI);
        builder.addDoubleProperty("D", () -> this.kD, this::setD);
    }
}
