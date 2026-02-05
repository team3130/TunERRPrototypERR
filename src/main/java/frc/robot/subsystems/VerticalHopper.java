package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants; // unnecessary, but not an issue
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class VerticalHopper extends SubsystemBase {
    private final TalonFX hoppervertical;
    private double verticalSpeed = 0.3;
    public VerticalHopper() {
        hoppervertical = new TalonFX(34); // not necessary but just in case
        hoppervertical.getConfigurator().apply(new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast).withInverted(InvertedValue.Clockwise_Positive)));
    }  

    public void runHopperVertical() {
        hoppervertical.set(verticalSpeed);
    } 
    public void reverseHopperVertical() {
        hoppervertical.set(-verticalSpeed);
    }

    public void stopHopperVertical() {
        hoppervertical.set(0);
    }

    public double getVerticalSpeed() {
        return verticalSpeed;
    }

    public void setVerticalSpeed(double value) {
        verticalSpeed = value;
    }

    @Override
    public void periodic() {
        // run every 20 millisecond
    }
}
