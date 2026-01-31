package frc.robot.commands.Hopper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hoppervertical;

public class RunHoppervertical extends Command {
    private final Hoppervertical hoppervertical;

    public RunHoppervertical(Hoppervertical hoppervertical) {
        this.hoppervertical = hoppervertical;
        addRequirements(hoppervertical);
    }

    @Override
    public void initialize() {
        hoppervertical.runHoppervertical();
    }

    @Override
    public void execute() {}
    
    @Override
    public void end(boolean interrupted){
        hoppervertical.stopHoppervertical();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}