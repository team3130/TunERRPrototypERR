package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase{
    private final TalonFX climberMotor;
    private final Solenoid bottomHooks;
    private final Solenoid topHooks;
    private double climberSpeed = 0.1;
    private boolean bottomHooksState;
    private boolean topHooksState;
    private final double targetL1 = 75; //Rotations we need to test for L1 for Autons

    public Climber() {
        climberMotor = new TalonFX(0);
        bottomHooks = new Solenoid(Constants.CAN.PCM,PneumaticsModuleType.CTREPCM, 0);
        topHooks = new Solenoid(Constants.CAN.PCM,PneumaticsModuleType.CTREPCM, 0);
        climberMotor.getConfigurator().apply(new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)));

        bottomHooks.set(false);
        topHooks.set(false);
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

    public double getPosition() {
        return climberMotor.getPosition().getValueAsDouble();
    }

    public double getTargetL1() {
        return targetL1;
    }

    public void extendBottomHooks() {
        bottomHooks.set(true);
        bottomHooksState = true;
    }
    public void retractBottomHooks(){
        bottomHooks.set(false);
        bottomHooksState = false;
    }
    public void extendTopHooks() {
        topHooks.set(true);
        topHooksState = true;
    }
    public void retractTopmHooks(){
        topHooks.set(false);
        topHooksState = false;
    }

    public double getClimberSpeed() {
        return climberSpeed;
    }
    public void setClimberSpeed(double value) {
        climberSpeed = value;
    }

    public boolean getBottomHooksState() {
        return bottomHooksState;
    }
    public boolean getTopHooksState() {
        return topHooksState;
    }

    @Override
    public void periodic() {
        // run every 20 millisecond
    }
}
