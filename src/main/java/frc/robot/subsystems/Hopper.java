package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants; // unnecessary, but not an issue
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Hopper extends SubsystemBase {
    private final TalonFX hopperMotor;
    private double hopperSpeed = 0.3;
    public Hopper() {
        hopperMotor = new TalonFX(35); // not necessary but just in case
        hopperMotor.getConfigurator().apply(new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast).withInverted(InvertedValue.Clockwise_Positive)));
    }  

    public void runHopper() {
        hopperMotor.set(hopperSpeed);
    }

    public void reverseHopper() {
        hopperMotor.set(-hopperSpeed);
    }

    public void stopHopper() {
        hopperMotor.set(0);
    }

    public double getHopperSpeed() {
        return hopperSpeed;
    }


    public void setHopperSpeed(double value) {
        hopperSpeed = value;
    }

    @Override
    public void periodic() {
        // run every 20 millisecond
    }
}
