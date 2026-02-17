package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ExtendTopHooks extends Command {
    private final Climber climber;

    public ExtendTopHooks(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.extendTopHooks();
    }

    @Override
    public void execute() {
        
    }
    
    public void end(boolean interrupted) {}

    public boolean isFinished() {
        return false;
    }
}
