package frc.robot.commands.Hopper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;

public class HopperCommand extends Command {
    private final Hopper hopper;

    public HopperCommand(Hopper hopper) {
        this.hopper = hopper;
        addRequirements(hopper);
    }

    @Override
    public void initialize() {
        hopper.runHopper();
    }

    @Override
    public void execute() {}
    
    @Override
    public void end(boolean interrupted){
        hopper.stopHopper();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}