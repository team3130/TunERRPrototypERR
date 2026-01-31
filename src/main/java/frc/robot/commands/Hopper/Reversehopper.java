package frc.robot.commands.Hopper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;

public class Reversehopper extends Command{
    private final Hopper hopper;

    public Reversehopper(Hopper hopper) {
        this.hopper = hopper;
        addRequirements(hopper);
    }

    @Override
    public void initialize() {
        hopper.reverseHopper();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        hopper.stopHopper();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
