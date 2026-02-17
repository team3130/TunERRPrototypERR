package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class RunClimberMotor {
    private final Climber climber;

    public RunClimberMotor(Climber climber) {
        this.climber = climber;
        addRequirements(climber);

    }
}
