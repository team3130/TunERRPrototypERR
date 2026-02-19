package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ReverseClimberMotor extends Command {
    private final Climber climber;

    public ReverseClimberMotor(Climber climber) {
        this.climber = climber;
        addRequirements(climber);

    }

    public void initialize() {
        climber.reverseClimber();
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
