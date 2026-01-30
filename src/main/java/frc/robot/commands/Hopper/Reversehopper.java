package frc.robot.commands.Hopper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;

public class Reversehopper extends Command{
    private final Hopper reverseHopper;

    public Reversehopper(Hopper hopper) {
        this.reverseHopper = hopper;
    }

    @Override
    public void initialize() {
        // Example usage of reverseHopper
        reverseHopper.reverseHopper();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        reverseHopper.stopHopper();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
