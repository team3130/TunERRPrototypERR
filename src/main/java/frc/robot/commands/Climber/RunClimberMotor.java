package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class RunClimberMotor extends Command{
    private final Climber climber;

    public RunClimberMotor(Climber climber) {
        this.climber = climber;
        addRequirements(climber);

    }

    public void initialize() {
        climber.runClimber();
    }

    public void execute() {

    }

    public void end(boolean interrupted) {
        climber.stopClimber();
    }

    public boolean isFinished() {
        return false;
    }
}
