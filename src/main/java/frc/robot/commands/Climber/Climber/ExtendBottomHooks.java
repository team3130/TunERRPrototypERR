package frc.robot.commands.Climber.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ExtendBottomHooks extends Command {
    private final Climber climber;

    public ExtendBottomHooks(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.extendBottomHooks();
    }

    @Override
    public void execute() {
        
    }
    
    public void end(boolean interrupted) {}

    public boolean isFinished() {
        return false;
    }
}
