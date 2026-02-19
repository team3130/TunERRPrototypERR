package frc.robot.commands.Hopper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VerticalHopper;

public class ReverseHopperVertical extends Command{
    private final VerticalHopper hopper;

    public ReverseHopperVertical(VerticalHopper hopper) {
        this.hopper = hopper;
        addRequirements(hopper);
    }

    @Override
    public void initialize() {
        hopper.updatePID();
        hopper.reverseHopperVertical();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        hopper.stopHopperVertical();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
