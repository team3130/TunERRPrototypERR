package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    private final TalonFX climberMotor;
    private final Solenoid bottomHook;
    private double climberSpeed = 0.5;
    private boolean bottomHookState;

    public Climber() {
        climberMotor = new TalonFX(0);
        bottomHook = new Solenoid(0);
        climberMotor.getConfigurator().apply(new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)));
    }

    public void runClimber() {
        climberMotor.set(climberSpeed);
    }

    public void reverseClimber() {
        climberMotor.set(-climberSpeed);
    }

    public void stopClimber() {
        climberMotor.set(0);
    }

    public double getClimberSpeed() {
        return climberSpeed;
    }


    public void setClimberSpeed(double value) {
        climberSpeed = value;
    }

    @Override
    public void periodic() {
        // run every 20 millisecond
    }
}
